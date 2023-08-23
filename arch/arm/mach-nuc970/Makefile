#
# Makefile for the linux kernel.
#

# Object file lists.

obj-y				:= irq.o cpu.o mfp.o mach-nuc970.o pm.o suspend.o dev.o
obj-$(CONFIG_NUC970_CLK_TIMER)	+= time.o
obj-$(CONFIG_NUC970_FIQ)	+= fiq.o
obj-$(CONFIG_USE_OF)		+= dt-nuc970-dev-v1.0.o
