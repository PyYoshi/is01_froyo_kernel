#
# Makefile for the linux kernel.
#

EXTRA_CFLAGS += -Wall

#ifeq ($(CONFIG_SHIRDA),y)
EXTRA_CFLAGS += -DIR_FEATURE_WAIT_FOR_SLEEP
#endif

#obj-$(CONFIG_SHIRDA) += shirda_LR388G7.o
obj-y += shirda_LR388G7.o
shirda_LR388G7-objs := irda_kdrv.o ir_LR388G7.o

