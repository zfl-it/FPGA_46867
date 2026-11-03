# 适配 RK3568 板端（内核 6.1.99，头文件在 /usr/src/linux-headers-6.1-rockchip）
# 用法：make            # 使用下面的默认 KERNELDIR
#      make KERNELDIR=/lib/modules/$(uname -r)/build   # 如需覆盖
CONFIG_MODULE_SIG=n

ifeq ($(KERNELRELEASE),)
KERNELDIR ?= /usr/src/linux-headers-6.1-rockchip
PWD := $(shell pwd)

modules:
	$(MAKE) -C $(KERNELDIR) M=$(PWD) modules

modules_install:
	$(MAKE) -C $(KERNELDIR) M=$(PWD) modules_install

clean:
	rm -rf *.o *~ core .depend .*.cmd *.ko *.mod *.mod.c .tmp_versions Module* modules*
.PHONY: modules modules_install clean insmod rmmod reload

insmod:
	/sbin/insmod pango_pci_driver.ko

rmmod:
	/sbin/rmmod pango_pci_driver

reload:
	-/sbin/rmmod pango_pci_driver
	/sbin/insmod pango_pci_driver.ko
else
obj-m := pango_pci_driver.o
endif
