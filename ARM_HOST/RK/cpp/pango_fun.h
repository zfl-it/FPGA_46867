#ifndef PANGO_FUN_H
#define PANGO_FUN_H

#include <cstddef>
#include <string>
#include <sys/types.h>

#include "pango_pci.h"

// Non-Qt version of the original pango_fun used by ZYNQ_video_show.
// The caller owns the capture thread; this class only owns PCIe/DMA resources.
class PangoFun
{
public:
    PangoFun();
    ~PangoFun();

    PangoFun(const PangoFun &) = delete;
    PangoFun &operator=(const PangoFun &) = delete;

    // Equivalent to pango_fun::getDevice(): open the driver, query PCIe
    // information, apply the driver's base DMA configuration and map BAR0.
    bool getDevice(const std::string &pcie_device, const std::string &mem_device);

    // Equivalent to MainWindow's DMA setup before dma_map(1).
    // dma_chunk_bytes is converted from bytes to 32-bit words for the driver.
    bool configureDma(unsigned int dma_chunk_bytes, unsigned int offset_addr = 0);
    bool dmaMap(bool enable);

    // Equivalent to the MainWindow capture button register writes.
    bool startCapture();
    bool stopCapture();

    // Equivalent to pango_fun::dma_read().
    ssize_t dmaRead(void *buffer, std::size_t length);

    void closeDevice();

    bool isOpen() const { return dev_fd_ >= 0; }
    bool isDmaMapped() const { return dma_mapped_; }
    const PCI_DEVICE_INFO &deviceInfo() const;
    const std::string &lastError() const { return last_error_; }

private:
    bool dmaSetConfig();
    bool mapBar0(unsigned long physical_address, unsigned int size);
    bool writeRegister(unsigned int value, int pio_base_addr, int user_reg_addr);
    bool fail(const std::string &operation);
    void resetState();

    int dev_fd_;
    int mem_fd_;
    void *bar_mapping_;
    std::size_t bar_mapping_length_;
    volatile unsigned int *mem_pio_addr_;

    COMMAND_OPERATION command_operation_;
    COMMAND_OPERATION bar_command_;
    DMA_OPERATION dma_operation_;

    bool dma_mapped_;
    bool capture_started_;
    std::string pcie_device_;
    std::string mem_device_;
    std::string last_error_;
};

#endif // PANGO_FUN_H
