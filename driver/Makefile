MODULE_NAME = dw3000-drv

SRC := dw3000.c

# Path to target Linux Kernel
KDIR        := /lib/modules/$(shell uname -r)/build # <--- Fill in with path to kernel you're compiling against

$(MODULE_NAME)-objs = $(SRC:.c=.o)

obj-m       := $(MODULE_NAME).o

ccflags-y := -I$(src)/inc -I$(src)/platform -I$(src)/decadriver -I$(src)


all:
	$(MAKE) -C $(KDIR) M=$(shell pwd) modules

clean:
	$(MAKE) -C $(KDIR) M=$(shell pwd) clean
