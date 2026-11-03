#include "pango_fun.h"

#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <iostream>
#include <limits>
#include <sstream>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <unistd.h>

namespace
{
const unsigned int kStartCaptureValue = 0xffffffe5U;
const unsigned int kStopCaptureValue = 0xffffff00U;
}

PangoFun::PangoFun()
{
    resetState();
}

PangoFun::~PangoFun()
{
    closeDevice();
}

void PangoFun::resetState()
{
    dev_fd_ = -1;
    mem_fd_ = -1;
    bar_mapping_ = NULL;
    bar_mapping_length_ = 0;
    mem_pio_addr_ = NULL;
    std::memset(&command_operation_, 0, sizeof(command_operation_));
    std::memset(&bar_command_, 0, sizeof(bar_command_));
    std::memset(&dma_operation_, 0, sizeof(dma_operation_));
    dma_mapped_ = false;
    capture_started_ = false;
    pcie_device_.clear();
    mem_device_.clear();
}

bool PangoFun::fail(const std::string &operation)
{
    const int error = errno;
    std::ostringstream message;
    message << operation;
    if (error != 0)
        message << ": " << std::strerror(error) << " (errno=" << error << ")";
    last_error_ = message.str();
    std::cerr << "[pango_fun] " << last_error_ << std::endl;
    return false;
}

bool PangoFun::mapBar0(unsigned long physical_address, unsigned int size)
{
    if (physical_address == 0 || size == 0)
    {
        errno = EINVAL;
        return fail("invalid BAR0 mapping parameters");
    }

    const long page_size_value = sysconf(_SC_PAGESIZE);
    if (page_size_value <= 0)
        return fail("sysconf(_SC_PAGESIZE) failed");

    const std::size_t page_size = static_cast<std::size_t>(page_size_value);
    const unsigned long aligned_address =
        physical_address & ~(static_cast<unsigned long>(page_size) - 1UL);
    const std::size_t page_offset =
        static_cast<std::size_t>(physical_address - aligned_address);

    if (page_offset > std::numeric_limits<std::size_t>::max() - size)
    {
        errno = EOVERFLOW;
        return fail("BAR0 mapping length overflow");
    }

    const std::size_t requested_length = page_offset + size;
    if (requested_length > std::numeric_limits<std::size_t>::max() - (page_size - 1U))
    {
        errno = EOVERFLOW;
        return fail("BAR0 page-aligned length overflow");
    }
    bar_mapping_length_ =
        (requested_length + page_size - 1U) & ~(page_size - 1U);

    mem_fd_ = open(mem_device_.c_str(), O_RDWR | O_SYNC);
    if (mem_fd_ < 0)
        return fail("open " + mem_device_ + " failed");

    bar_mapping_ = mmap(NULL,
                        bar_mapping_length_,
                        PROT_READ | PROT_WRITE,
                        MAP_SHARED,
                        mem_fd_,
                        static_cast<off_t>(aligned_address));
    const int mmap_error = errno;
    close(mem_fd_);
    mem_fd_ = -1;
    errno = mmap_error;

    if (bar_mapping_ == MAP_FAILED)
    {
        bar_mapping_ = NULL;
        bar_mapping_length_ = 0;
        return fail("mmap BAR0 through " + mem_device_ + " failed");
    }

    unsigned char *mapped_bytes = static_cast<unsigned char *>(bar_mapping_);
    mem_pio_addr_ = reinterpret_cast<volatile unsigned int *>(mapped_bytes + page_offset);
    std::cerr << "[pango_fun] BAR0 mapped: phys=0x" << std::hex
              << physical_address << std::dec << ", size=" << size << std::endl;
    return true;
}

bool PangoFun::dmaSetConfig()
{
    if (dev_fd_ < 0)
    {
        errno = EBADF;
        return fail("PCI_SET_CONFIG called before opening the device");
    }

    if (ioctl(dev_fd_, PCI_SET_CONFIG, &dma_operation_) < 0)
        return fail("PCI_SET_CONFIG failed");
    return true;
}

bool PangoFun::getDevice(const std::string &pcie_device,
                         const std::string &mem_device)
{
    closeDevice();
    last_error_.clear();
    pcie_device_ = pcie_device.empty() ? PCIE_DRIVER_FILE_PATH : pcie_device;
    mem_device_ = mem_device.empty() ? MEM_FILE_PATH : mem_device;
    std::memset(&command_operation_, 0, sizeof(command_operation_));
    std::memset(&bar_command_, 0, sizeof(bar_command_));
    std::memset(&dma_operation_, 0, sizeof(dma_operation_));

    dev_fd_ = open(pcie_device_.c_str(), O_RDWR);
    if (dev_fd_ < 0)
        return fail("open " + pcie_device_ + " failed");

    // Keep the same order as the working Qt pango_fun::getDevice().
    if (ioctl(dev_fd_, PCI_READ_DATA_CMD, &command_operation_) < 0)
    {
        fail("PCI_READ_DATA_CMD failed");
        closeDevice();
        return false;
    }

    std::cerr << "[pango_fun] device: vendor=0x" << std::hex
              << command_operation_.get_pci_dev_info.vendor_id
              << ", device=0x" << command_operation_.get_pci_dev_info.device_id
              << std::dec << ", link=Gen"
              << command_operation_.get_pci_dev_info.link_speed << " x"
              << command_operation_.get_pci_dev_info.link_width
              << ", max_payload=" << command_operation_.get_pci_dev_info.mps
              << std::endl;

    // The reference code issues PCI_SET_CONFIG before MainWindow assigns
    // current_len and offset_addr. Preserve that driver-visible sequence.
    if (!dmaSetConfig())
    {
        closeDevice();
        return false;
    }

    if (ioctl(dev_fd_, PCI_MAP_BAR0_CMD, &bar_command_) < 0)
    {
        fail("PCI_MAP_BAR0_CMD failed");
        closeDevice();
        return false;
    }

    if (!mapBar0(bar_command_.get_pci_dev_info.bar[0].bar_base, BAR0_MAX))
    {
        closeDevice();
        return false;
    }

    return true;
}

bool PangoFun::configureDma(unsigned int dma_chunk_bytes,
                            unsigned int offset_addr)
{
    if (dev_fd_ < 0)
    {
        errno = EBADF;
        return fail("configureDma called before opening the device");
    }
    if (dma_mapped_)
    {
        errno = EBUSY;
        return fail("configureDma called while DMA is mapped");
    }
    if (dma_chunk_bytes == 0 ||
        dma_chunk_bytes > DMA_MAX_PACKET_SIZE ||
        (dma_chunk_bytes & 0x3U) != 0)
    {
        errno = EINVAL;
        return fail("DMA chunk size must be a non-zero multiple of 4 not exceeding DMA_MAX_PACKET_SIZE");
    }

    // Same values assigned in MainWindow::on_Key1_clicked().
    dma_operation_.current_len = dma_chunk_bytes >> 2;
    dma_operation_.offset_addr = offset_addr;
    return true;
}

bool PangoFun::dmaMap(bool enable)
{
    if (dev_fd_ < 0)
    {
        errno = EBADF;
        return fail("DMA map called before opening the device");
    }

    if (enable == dma_mapped_)
        return true;

    const unsigned long command = enable ? PCI_MAP_ADDR_CMD : PCI_UMAP_ADDR_CMD;
    if (ioctl(dev_fd_, command, &dma_operation_) < 0)
        return fail(enable ? "PCI_MAP_ADDR_CMD failed" : "PCI_UMAP_ADDR_CMD failed");

    dma_mapped_ = enable;
    return true;
}

bool PangoFun::writeRegister(unsigned int value,
                             int pio_base_addr,
                             int user_reg_addr)
{
    if (!mem_pio_addr_ || pio_base_addr < 0 || user_reg_addr < 0)
    {
        errno = EINVAL;
        return fail("invalid BAR0 register write");
    }

    mem_pio_addr_[pio_base_addr + user_reg_addr] = value;
    return true;
}

bool PangoFun::startCapture()
{
    if (!dma_mapped_)
    {
        errno = EINVAL;
        return fail("startCapture requires an active DMA mapping");
    }
    if (capture_started_)
        return true;

    if (!writeRegister(kStartCaptureValue, 0, 0))
        return false;

    capture_started_ = true;
    std::cerr << "[pango_fun] capture started (BAR0[0]=0xffffffe5)" << std::endl;
    return true;
}

bool PangoFun::stopCapture()
{
    bool ok = true;

    if (capture_started_ && mem_pio_addr_)
    {
        if (!writeRegister(kStopCaptureValue, 0, 0))
            ok = false;
        capture_started_ = false;
    }

    if (dma_mapped_ && !dmaMap(false))
        ok = false;

    return ok;
}

ssize_t PangoFun::dmaRead(void *buffer, std::size_t length)
{
    if (dev_fd_ < 0 || !dma_mapped_ || !capture_started_ || !buffer || length == 0)
    {
        errno = EINVAL;
        return -1;
    }
    return read(dev_fd_, buffer, length);
}

void PangoFun::closeDevice()
{
    stopCapture();

    if (bar_mapping_)
    {
        munmap(bar_mapping_, bar_mapping_length_);
        bar_mapping_ = NULL;
        bar_mapping_length_ = 0;
        mem_pio_addr_ = NULL;
    }

    if (mem_fd_ >= 0)
    {
        close(mem_fd_);
        mem_fd_ = -1;
    }

    if (dev_fd_ >= 0)
    {
        close(dev_fd_);
        dev_fd_ = -1;
    }

    dma_mapped_ = false;
    capture_started_ = false;
}

const PCI_DEVICE_INFO &PangoFun::deviceInfo() const
{
    return command_operation_.get_pci_dev_info;
}
