#ifndef PANGO_PCI_H
#define PANGO_PCI_H

#include <stdint.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <sys/mman.h>

#define PCIE_DRIVER_FILE_PATH           "/dev/pango_pci_driver"
#define MEM_FILE_PATH                   "/dev/mem"

#define TYPE                            'S'
#define PCI_READ_DATA_CMD               _IOWR(TYPE, 0, int)
#define PCI_MAP_ADDR_CMD                _IOWR(TYPE, 2, int)
#define PCI_DMA_READ_CMD                _IOWR(TYPE, 4, int)
#define PCI_UMAP_ADDR_CMD               _IOWR(TYPE, 7, int)
#define PCI_MAP_BAR0_CMD                _IOWR(TYPE,10, int)
#define PCI_SET_CONFIG                  _IOWR(TYPE,11, int)

#define BAR0_MAX                        0x500
#define DMA_MAX_PACKET_SIZE             4096
#define MAX_BLOCK_SIZE                  1024

// 图像参数
#define IMG_WIDTH 1280
#define IMG_HEIGHT 720
#define IMG_BYTES_PER_PIXEL 2
#define FRAME_SIZE (IMG_WIDTH * IMG_HEIGHT * IMG_BYTES_PER_PIXEL)

typedef struct _BAR_INFO_
{
    unsigned long bar_base;
    unsigned long bar_len;
} BAR_BASE_INFO;

typedef struct _CAP_INFO_
{
    unsigned char flag;
    unsigned char id;
    unsigned char addr_offset;
    unsigned char next_offset;
} CAP_INFO;

typedef struct _CAP_LIST_
{
    unsigned char cap_status;
    unsigned char cap_error;
    CAP_INFO cap_buf[256];
} CAP_LIST;

typedef struct _LOAD_DATA_
{
    unsigned int num_words;
    unsigned int block_words[MAX_BLOCK_SIZE];
} LOAD_DATA_INFO;

typedef struct _PCI_LOAD_
{
    unsigned char link_status;
    unsigned int  crc;
    unsigned char axi_direction;
    unsigned char load_status;
    unsigned int total_num_words;
    LOAD_DATA_INFO data_block;
} PCI_LOAD_INFO;

typedef struct _PCI_INFO_
{
    unsigned int vendor_id;
    unsigned int device_id;
    unsigned int cmd_reg;
    unsigned int status_reg;
    unsigned int revision_id;
    unsigned int class_prog;
    unsigned int class_device;
    BAR_BASE_INFO bar[6];
    unsigned int min_gnt;
    unsigned int max_lat;
    unsigned int link_speed;
    unsigned int link_width;
    unsigned int mps;
    unsigned int mrrs;
    unsigned int data[1024];
} PCI_DEVICE_INFO;

typedef struct _COMMAND_
{
    unsigned char w_r;
    unsigned char step;
    unsigned int addr;
    unsigned int data;
    unsigned int cnt;
    unsigned int delay;
    PCI_DEVICE_INFO get_pci_dev_info;
    CAP_LIST cap_info;
    PCI_LOAD_INFO load_info;
} COMMAND_OPERATION;

typedef struct _DMA_DATA_
{
    unsigned char read_buf[DMA_MAX_PACKET_SIZE];
    unsigned char write_buf[DMA_MAX_PACKET_SIZE];
} DMA_DATA;

typedef struct _DMA_OPERATION_
{
    unsigned int current_len;
    unsigned int offset_addr;
    unsigned int cmd;
    DMA_DATA data;
} DMA_OPERATION;

#endif // PANGO_PCI_H
