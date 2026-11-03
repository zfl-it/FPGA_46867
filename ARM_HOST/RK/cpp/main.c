#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <errno.h>

#include "pango_pci.h"

#define PAGE_ROUND_DOWN(x) ((x) & ~(getpagesize() - 1))
#define PAGE_ROUND_UP(x)   (PAGE_ROUND_DOWN((x) + getpagesize() - 1))

int dev_fd = -1;
void *mem_pio_addr = NULL;
COMMAND_OPERATION command_operation;  // 对应 Qt 的 pango_fun::command_operation
COMMAND_OPERATION cmd_op;            // 对应 Qt 的全局 cmd_op
DMA_OPERATION dma_operation;         // 对应 Qt 的 pango_fun::dma_operation
DMA_OPERATION *dma_oper = &dma_operation; // 对应 Qt 的指针用法

/*
 * map_mem: 完全按照 Qt 原版 pango_fun::map_mem 的逻辑翻译
 */
void *map_mem(unsigned long addr, unsigned int size) {
    int fd = -1;
    void *vaddr = NULL;
    off_t offset;
    size_t len;

    offset = PAGE_ROUND_DOWN(addr);
    len    = PAGE_ROUND_UP(size);

    fd = open(MEM_FILE_PATH, O_RDWR | O_SYNC);
    if (fd < 0) {
        fprintf(stderr, "open '%s' failed!\n", MEM_FILE_PATH);
        return NULL;
    }

    vaddr = mmap(NULL, len, PROT_READ | PROT_WRITE, MAP_SHARED, fd, offset);
    close(fd);

    if ((vaddr == NULL) || (vaddr == MAP_FAILED)) {
        fprintf(stderr, "mmap '%s' failed!\n", MEM_FILE_PATH);
        return NULL;
    }

    fprintf(stderr, "bar0 vir addr mapped OK\n");
    return vaddr;
}

/*
 * pio_write: 完全按照 Qt 原版翻译
 */
void pio_write(unsigned int write_data, int pio_bse_addr, int user_reg_addr) {
    *((unsigned int *)mem_pio_addr + pio_bse_addr + user_reg_addr) = write_data;
}

/*
 * dma_set_config: 对应 Qt 的 pango_fun::dma_set_config
 */
void dma_set_config(void) {
    ioctl(dev_fd, PCI_SET_CONFIG, dma_oper);
}

/*
 * dma_map: 对应 Qt 的 pango_fun::dma_map
 */
void dma_map(int flag) {
    if (flag) {
        ioctl(dev_fd, PCI_MAP_ADDR_CMD, dma_oper);
    } else {
        ioctl(dev_fd, PCI_UMAP_ADDR_CMD, dma_oper);
    }
}

/*
 * dma_read: 对应 Qt 的 pango_fun::dma_read
 */
long long int dma_read(unsigned int len, unsigned char *img_buf) {
    return read(dev_fd, img_buf, len);
}

int main() {
    /*
     * ========== 第一阶段: getDevice() ==========
     * 完全按照 pango_fun::getDevice() 的顺序
     */

    // 1. open
    dev_fd = open(PCIE_DRIVER_FILE_PATH, O_RDWR);
    if (dev_fd < 0) {
        fprintf(stderr, "open PCIe driver failed: %s\n", strerror(errno));
        return -1;
    }
    fprintf(stderr, "[init] open success, dev_fd = %d\n", dev_fd);

    // 2. PCI_READ_DATA_CMD — 读取设备信息 (Qt原版第一步就做这个)
    ioctl(dev_fd, PCI_READ_DATA_CMD, &command_operation);
    fprintf(stderr, "[init] vendor_id: 0x%04x\n", command_operation.get_pci_dev_info.vendor_id);
    fprintf(stderr, "[init] device_id: 0x%04x\n", command_operation.get_pci_dev_info.device_id);
    fprintf(stderr, "[init] link: gen%d x%d\n",
            command_operation.get_pci_dev_info.link_speed,
            command_operation.get_pci_dev_info.link_width);
    fprintf(stderr, "[init] max payload: %d\n", command_operation.get_pci_dev_info.mps);

    // 3. dma_set_config() — Qt原版在 PCI_READ_DATA_CMD 之后立即调用
    dma_set_config();

    // 4. PCI_MAP_BAR0_CMD — 获取 bar0 地址映射
    memset(&cmd_op, 0, sizeof(cmd_op));
    ioctl(dev_fd, PCI_MAP_BAR0_CMD, &cmd_op);

    // 5. map_mem — 映射 BAR0 到用户空间
    mem_pio_addr = map_mem(cmd_op.get_pci_dev_info.bar[0].bar_base, BAR0_MAX);
    if (!mem_pio_addr) {
        fprintf(stderr, "[init] map_mem failed!\n");
        close(dev_fd);
        return -1;
    }

    /*
     * ========== 第二阶段: on_Key1_clicked() 打开按钮 ==========
     * 完全按照 mainwindow.cpp 第 167~169 行
     */

    // 6. 重新配置 DMA 长度 (和 Qt 一模一样: 3840>>2)
    dma_oper->current_len = 3840 >> 2;  // conv to DW
    dma_oper->offset_addr = 0;

    // 7. dma_map(1) — 映射 DMA 地址
    dma_map(1);

    /*
     * ========== 第三阶段: on_connect_clicked() 采集视频按钮 ==========
     * 完全按照 mainwindow.cpp 第 223~228 行
     */

    // 8. pio_write 启动采集
    pio_write(0xffffffe5, 0, 0);

    fprintf(stderr, "[C Reader] Started successfully. Waiting for frames...\n");

    // 9. 分配图像缓冲区 (Qt原版用 new uchar[1280*720*2])
    unsigned char *img_buf = (unsigned char *)malloc(1280 * 720 * 2);
    if (!img_buf) {
        fprintf(stderr, "malloc img_buf failed!\n");
        return -1;
    }

    /*
     * ========== 第四阶段: run() 线程循环 ==========
     * 完全按照 pango_fun::run() 第 258~277 行
     */
    long long int rd_len;
    while (1) {
        rd_len = dma_read(1280 * 720 * 2, img_buf);
        if (rd_len && 0x01 == 1) {
            // 和 Qt 一样的判断条件 (原版: if(rd_len && 0x01== 1))
            // 写到 stdout 管道
            fwrite(img_buf, 1, 1280 * 720 * 2, stdout);
            fflush(stdout);
        }
        rd_len = 0;
    }

    // cleanup (实际走不到这里)
    free(img_buf);
    close(dev_fd);
    return 0;
}
