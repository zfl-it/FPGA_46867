#ifndef PANGO_PCIE_LIB_H
#define PANGO_PCIE_LIB_H

#include <stddef.h>
#include <sys/types.h>

#include "pango_pci.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    unsigned int width;
    unsigned int height;
    unsigned int bytes_per_pixel;
    unsigned int frame_size;

    /*
     * 对应原代码：
     * dma_oper->current_len = 3840 >> 2;
     *
     * 这里填写字节数，库内部会自动 >> 2 转成 DW。
     */
    unsigned int dma_chunk_bytes;

    /*
     * 对应原代码：
     * pio_write(0xffffffe5, 0, 0);
     */
    unsigned int start_reg_value;
    int pio_base_addr;
    int user_reg_addr;

    /*
     * 默认 0：按当前 main.c 的行为，部分 ioctl 失败时打印告警但继续走，
     * 方便先确认 PCIe read 是否有数据输出。
     *
     * 设置为非 0：任意初始化 ioctl 失败都直接返回错误。
     */
    int strict_ioctl;

} PangoPcieConfig;

typedef struct {
    int dev_fd;
    void *mem_pio_addr;

    COMMAND_OPERATION command_operation;
    COMMAND_OPERATION cmd_op;
    DMA_OPERATION dma_operation;

    PangoPcieConfig cfg;

    int dma_mapped;
    int initialized;

    unsigned long long read_count;
    unsigned long long read_error_count;

} PangoPcieContext;

/*
 * 默认配置：
 * 1280x720 RGB565
 * DMA_CHUNK_BYTES = 3840
 */
void pango_pcie_default_config(PangoPcieConfig *cfg);

/*
 * 初始化 PCIe：
 * 1. open 驱动
 * 2. PCI_READ_DATA_CMD
 * 3. PCI_SET_CONFIG，默认失败只告警并继续
 * 4. PCI_MAP_BAR0_CMD
 * 5. mmap BAR0
 * 6. 设置 current_len
 * 7. PCI_MAP_ADDR_CMD，默认失败只告警并继续
 */
int pango_pcie_init(PangoPcieContext *ctx, const PangoPcieConfig *cfg);

/*
 * 启动采集：
 * 默认执行 pio_write(0xffffffe5, 0, 0)
 */
int pango_pcie_start_capture(PangoPcieContext *ctx);

/*
 * 写 BAR0 用户寄存器。
 *
 * 注意：
 * pio_base_addr 和 user_reg_addr 沿用你原来的写法，
 * 是 32bit word index，不是 byte offset。
 */
int pango_pcie_write_reg(PangoPcieContext *ctx,
                         int pio_base_addr,
                         int user_reg_addr,
                         unsigned int value);

/*
 * 读 BAR0 用户寄存器。
 */
int pango_pcie_read_reg(PangoPcieContext *ctx,
                        int pio_base_addr,
                        int user_reg_addr,
                        unsigned int *value);

/*
 * 普通 DMA read。
 *
 * 这个函数对应你原来的：
 * read(dev_fd, img_buf, len)
 *
 * 当前阶段先用这个，不用 read_full。
 */
ssize_t pango_pcie_read_once(PangoPcieContext *ctx,
                             unsigned char *buf,
                             size_t len);

/*
 * 释放资源：
 * 1. PCI_UMAP_ADDR_CMD
 * 2. munmap BAR0
 * 3. close 驱动
 */
void pango_pcie_close(PangoPcieContext *ctx);

#ifdef __cplusplus
}
#endif

#endif
