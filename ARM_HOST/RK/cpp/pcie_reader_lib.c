#include "pcie_reader_lib.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <errno.h>

#define PAGE_ROUND_DOWN(x) ((x) & ~(getpagesize() - 1))
#define PAGE_ROUND_UP(x)   (PAGE_ROUND_DOWN((x) + getpagesize() - 1))

/*
 * 保持你原来的 map_mem 行为，不在第一阶段改动 mmap 逻辑。
 */
static void *pango_map_mem(unsigned long addr, unsigned int size)
{
    int fd = -1;
    void *vaddr = NULL;
    off_t offset;
    size_t len;

    offset = PAGE_ROUND_DOWN(addr);
    len = PAGE_ROUND_UP(size);

    fd = open(MEM_FILE_PATH, O_RDWR | O_SYNC);
    if (fd < 0) {
        fprintf(stderr,
                "open '%s' failed: %s\n",
                MEM_FILE_PATH,
                strerror(errno));
        return NULL;
    }

    vaddr = mmap(NULL,
                 len,
                 PROT_READ | PROT_WRITE,
                 MAP_SHARED,
                 fd,
                 offset);

    close(fd);

    if (vaddr == MAP_FAILED || vaddr == NULL) {
        fprintf(stderr,
                "mmap '%s' failed: %s\n",
                MEM_FILE_PATH,
                strerror(errno));
        return NULL;
    }

    fprintf(stderr,
            "[PCIeLib] BAR0 mapped OK, phys=0x%lx, size=%u\n",
            addr,
            size);

    return vaddr;
}

void pango_pcie_default_config(PangoPcieConfig *cfg)
{
    if (!cfg) {
        return;
    }

    memset(cfg, 0, sizeof(*cfg));

    cfg->width = 1280;
    cfg->height = 720;
    cfg->bytes_per_pixel = 2;
    cfg->frame_size = cfg->width * cfg->height * cfg->bytes_per_pixel;

    /*
     * 第一阶段库化：保持你已经跑通的配置。
     */
    cfg->dma_chunk_bytes = 3840;

    cfg->start_reg_value = 0xffffffe5;
    cfg->pio_base_addr = 0;
    cfg->user_reg_addr = 0;
    cfg->strict_ioctl = 0;
}

static int pango_pcie_dma_set_config(PangoPcieContext *ctx)
{
    if (!ctx || ctx->dev_fd < 0) {
        return -1;
    }

    if (ioctl(ctx->dev_fd, PCI_SET_CONFIG, &ctx->dma_operation) < 0) {
        fprintf(stderr,
                "[PCIeLib] PCI_SET_CONFIG failed: %s\n",
                strerror(errno));
        if (ctx->cfg.strict_ioctl) {
            return -1;
        }
        fprintf(stderr,
                "[PCIeLib] continue after PCI_SET_CONFIG failure to match main.c\n");
    }

    return 0;
}

static int pango_pcie_dma_map(PangoPcieContext *ctx, int flag)
{
    int cmd;

    if (!ctx || ctx->dev_fd < 0) {
        return -1;
    }

    cmd = flag ? PCI_MAP_ADDR_CMD : PCI_UMAP_ADDR_CMD;

    if (ioctl(ctx->dev_fd, cmd, &ctx->dma_operation) < 0) {
        fprintf(stderr,
                "[PCIeLib] %s failed: %s\n",
                flag ? "PCI_MAP_ADDR_CMD" : "PCI_UMAP_ADDR_CMD",
                strerror(errno));
        if (ctx->cfg.strict_ioctl) {
            return -1;
        }
        fprintf(stderr,
                "[PCIeLib] continue after %s failure to match main.c\n",
                flag ? "PCI_MAP_ADDR_CMD" : "PCI_UMAP_ADDR_CMD");
        return 0;
    }

    ctx->dma_mapped = flag ? 1 : 0;

    return 0;
}

int pango_pcie_init(PangoPcieContext *ctx, const PangoPcieConfig *cfg)
{
    PangoPcieConfig local_cfg;

    if (!ctx) {
        fprintf(stderr, "[PCIeLib] ctx is NULL\n");
        return -1;
    }

    memset(ctx, 0, sizeof(*ctx));
    ctx->dev_fd = -1;

    if (cfg) {
        local_cfg = *cfg;
    } else {
        pango_pcie_default_config(&local_cfg);
    }

    if (local_cfg.width == 0 ||
        local_cfg.height == 0 ||
        local_cfg.bytes_per_pixel == 0) {
        fprintf(stderr, "[PCIeLib] invalid frame config\n");
        return -1;
    }

    local_cfg.frame_size =
        local_cfg.width * local_cfg.height * local_cfg.bytes_per_pixel;

    if (local_cfg.dma_chunk_bytes == 0) {
        local_cfg.dma_chunk_bytes = 3840;
    }

    ctx->cfg = local_cfg;

    /*
     * 1. open PCIe driver
     */
    ctx->dev_fd = open(PCIE_DRIVER_FILE_PATH, O_RDWR);
    if (ctx->dev_fd < 0) {
        fprintf(stderr,
                "[PCIeLib] open PCIe driver failed: %s\n",
                strerror(errno));
        return -1;
    }

    fprintf(stderr,
            "[PCIeLib] open success, dev_fd=%d\n",
            ctx->dev_fd);

    /*
     * 2. PCI_READ_DATA_CMD
     */
    memset(&ctx->command_operation, 0, sizeof(ctx->command_operation));

    if (ioctl(ctx->dev_fd,
              PCI_READ_DATA_CMD,
              &ctx->command_operation) < 0) {
        fprintf(stderr,
                "[PCIeLib] PCI_READ_DATA_CMD failed: %s\n",
                strerror(errno));
        if (ctx->cfg.strict_ioctl) {
            pango_pcie_close(ctx);
            return -1;
        }
        fprintf(stderr,
                "[PCIeLib] continue after PCI_READ_DATA_CMD failure to match main.c\n");
    }

    fprintf(stderr,
            "[PCIeLib] vendor_id: 0x%04x\n",
            ctx->command_operation.get_pci_dev_info.vendor_id);

    fprintf(stderr,
            "[PCIeLib] device_id: 0x%04x\n",
            ctx->command_operation.get_pci_dev_info.device_id);

    fprintf(stderr,
            "[PCIeLib] link: gen%d x%d\n",
            ctx->command_operation.get_pci_dev_info.link_speed,
            ctx->command_operation.get_pci_dev_info.link_width);

    fprintf(stderr,
            "[PCIeLib] max payload: %d\n",
            ctx->command_operation.get_pci_dev_info.mps);

    /*
     * 3. PCI_SET_CONFIG
     *
     * 保持原代码顺序：
     * memset dma_operation 后，立即 PCI_SET_CONFIG。
     */
    memset(&ctx->dma_operation, 0, sizeof(ctx->dma_operation));

    if (pango_pcie_dma_set_config(ctx) < 0) {
        pango_pcie_close(ctx);
        return -1;
    }

    /*
     * 4. PCI_MAP_BAR0_CMD
     */
    memset(&ctx->cmd_op, 0, sizeof(ctx->cmd_op));

    if (ioctl(ctx->dev_fd,
              PCI_MAP_BAR0_CMD,
              &ctx->cmd_op) < 0) {
        fprintf(stderr,
                "[PCIeLib] PCI_MAP_BAR0_CMD failed: %s\n",
                strerror(errno));
        if (ctx->cfg.strict_ioctl ||
            ctx->cmd_op.get_pci_dev_info.bar[0].bar_base == 0) {
            fprintf(stderr,
                    "[PCIeLib] BAR0 base is 0, cannot safely continue\n");
            pango_pcie_close(ctx);
            return -1;
        }
        fprintf(stderr,
                "[PCIeLib] continue after PCI_MAP_BAR0_CMD failure to match main.c\n");
    }

    /*
     * 5. mmap BAR0
     */
    fprintf(stderr,
            "[PCIeLib] BAR0 base from driver: 0x%lx\n",
            ctx->cmd_op.get_pci_dev_info.bar[0].bar_base);

    ctx->mem_pio_addr =
        pango_map_mem(ctx->cmd_op.get_pci_dev_info.bar[0].bar_base,
                      BAR0_MAX);

    if (!ctx->mem_pio_addr) {
        fprintf(stderr, "[PCIeLib] map BAR0 failed\n");
        pango_pcie_close(ctx);
        return -1;
    }

    /*
     * 6. 设置 DMA current_len
     *
     * current_len 单位是 DW。
     * 所以字节数需要 >> 2。
     */
    ctx->dma_operation.current_len = ctx->cfg.dma_chunk_bytes >> 2;
    ctx->dma_operation.offset_addr = 0;

    fprintf(stderr,
            "[PCIeLib] frame size: %u x %u x %u = %u bytes\n",
            ctx->cfg.width,
            ctx->cfg.height,
            ctx->cfg.bytes_per_pixel,
            ctx->cfg.frame_size);

    fprintf(stderr,
            "[PCIeLib] dma current_len: %u bytes, %u DW\n",
            ctx->cfg.dma_chunk_bytes,
            ctx->dma_operation.current_len);

    /*
     * 7. PCI_MAP_ADDR_CMD
     */
    if (pango_pcie_dma_map(ctx, 1) < 0) {
        pango_pcie_close(ctx);
        return -1;
    }

    ctx->initialized = 1;

    fprintf(stderr, "[PCIeLib] init finished\n");

    return 0;
}

int pango_pcie_start_capture(PangoPcieContext *ctx)
{
    if (!ctx || !ctx->initialized) {
        fprintf(stderr,
                "[PCIeLib] start_capture failed: not initialized\n");
        return -1;
    }

    /*
     * 默认等价于：
     * pio_write(0xffffffe5, 0, 0);
     */
    if (pango_pcie_write_reg(ctx,
                             ctx->cfg.pio_base_addr,
                             ctx->cfg.user_reg_addr,
                             ctx->cfg.start_reg_value) < 0) {
        return -1;
    }

    fprintf(stderr,
            "[PCIeLib] capture started, waiting for PCIe read data\n");

    return 0;
}

// #define PIO_WORD_ADDR(byte_addr) ((byte_addr) >> 2)

// #define PIO_START_REG      0x000
// #define PIO_DMA_ADDR_L     0x050
// #define PIO_DMA_ADDR_H     0x054
// #define PIO_DMA_SET_EN     0x060
// #define PIO_WR_FRAME_DONE  0x140

// int pango_pcie_start_capture(PangoPcieContext *ctx)
// {
//     if (!ctx || !ctx->initialized) {
//         fprintf(stderr, "[PCIeLib] start_capture failed: not initialized\n");
//         return -1;
//     }

//     /*
//      * 这里先临时用 0。
//      * 如果你的 FPGA DMA 需要真实 host DMA bus address，
//      * 这里必须换成驱动映射后的 DMA 地址。
//      */
//     unsigned int dma_addr_low  = 0x00000000;
//     unsigned int dma_addr_high = 0x00000000;

//     /*
//      * 1. 写 DMA base addr
//      */
//     if (pango_pcie_write_reg(ctx, 0, PIO_WORD_ADDR(PIO_DMA_ADDR_L), dma_addr_low) < 0)
//         return -1;

//     if (pango_pcie_write_reg(ctx, 0, PIO_WORD_ADDR(PIO_DMA_ADDR_H), dma_addr_high) < 0)
//         return -1;

//     /*
//      * 2. 触发 DMA 配置
//      */
//     if (pango_pcie_write_reg(ctx, 0, PIO_WORD_ADDR(PIO_DMA_SET_EN), 0x00000001) < 0)
//         return -1;

//     /*
//      * 3. 启动视频链路
//      */
//     if (pango_pcie_write_reg(ctx, 0, PIO_WORD_ADDR(PIO_START_REG), 0xffffffe5) < 0)
//         return -1;

//     fprintf(stderr, "[PCIeLib] capture started\n");
//     return 0;
// }

int pango_pcie_write_reg(PangoPcieContext *ctx,
                         int pio_base_addr,
                         int user_reg_addr,
                         unsigned int value)
{
    volatile unsigned int *reg;

    if (!ctx || !ctx->mem_pio_addr) {
        fprintf(stderr,
                "[PCIeLib] write_reg failed: BAR0 not mapped\n");
        return -1;
    }

    reg = (volatile unsigned int *)ctx->mem_pio_addr;

    reg[pio_base_addr + user_reg_addr] = value;

    return 0;
}

int pango_pcie_read_reg(PangoPcieContext *ctx,
                        int pio_base_addr,
                        int user_reg_addr,
                        unsigned int *value)
{
    volatile unsigned int *reg;

    if (!ctx || !ctx->mem_pio_addr || !value) {
        fprintf(stderr,
                "[PCIeLib] read_reg failed: invalid argument\n");
        return -1;
    }

    reg = (volatile unsigned int *)ctx->mem_pio_addr;

    *value = reg[pio_base_addr + user_reg_addr];

    return 0;
}

ssize_t pango_pcie_read_once(PangoPcieContext *ctx,
                             unsigned char *buf,
                             size_t len)
{
    ssize_t n;

    if (!ctx || ctx->dev_fd < 0 || !buf || len == 0) {
        errno = EINVAL;
        return -1;
    }

    n = read(ctx->dev_fd, buf, len);

    if (n > 0) {
        ctx->read_count++;
        if (ctx->read_count <= 3 || (ctx->read_count % 60ULL) == 0ULL) {
            fprintf(stderr,
                    "[PCIeLib] read_once #%llu: got %zd bytes, requested %zu\n",
                    ctx->read_count,
                    n,
                    len);
        }
    } else if (n < 0) {
        ctx->read_error_count++;
        if (ctx->read_error_count <= 3 ||
            (ctx->read_error_count % 60ULL) == 0ULL) {
            fprintf(stderr,
                    "[PCIeLib] read_once error #%llu: errno=%d (%s), requested %zu\n",
                    ctx->read_error_count,
                    errno,
                    strerror(errno),
                    len);
        }
    } else {
        ctx->read_error_count++;
        if (ctx->read_error_count <= 3 ||
            (ctx->read_error_count % 60ULL) == 0ULL) {
            fprintf(stderr,
                    "[PCIeLib] read_once empty #%llu: got 0 bytes, requested %zu\n",
                    ctx->read_error_count,
                    len);
        }
    }

    return n;
}

void pango_pcie_close(PangoPcieContext *ctx)
{
    if (!ctx) {
        return;
    }

    if (ctx->dev_fd >= 0 && ctx->dma_mapped) {
        pango_pcie_dma_map(ctx, 0);
        ctx->dma_mapped = 0;
    }

    if (ctx->mem_pio_addr) {
        munmap(ctx->mem_pio_addr, BAR0_MAX);
        ctx->mem_pio_addr = NULL;
    }

    if (ctx->dev_fd >= 0) {
        close(ctx->dev_fd);
        ctx->dev_fd = -1;
    }

    ctx->initialized = 0;

    fprintf(stderr, "[PCIeLib] closed\n");
}
