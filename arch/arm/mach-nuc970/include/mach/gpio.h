/*
 *  Nuvoton NUC970 GPIO API definitions
 *
 *
 *  This program is free software; you can redistribute it and/or modify it
 *  under the terms of the GNU General Public License version 2 as published
 *  by the Free Software Foundation.
 *
 */

#ifndef __ASM_MACH_NUC970_GPIO_H
#define __ASM_MACH_NUC970_GPIO_H

#define ARCH_NR_GPIOS		512
#include <mach/irqs.h>
#include <linux/interrupt.h>
#include <asm-generic/gpio.h>

#define	NUC970_PA0	(0x00 + 0)
#define	NUC970_PA1	(0x00 + 1)
#define	NUC970_PA2	(0x00 + 2)
#define	NUC970_PA3	(0x00 + 3)
#define	NUC970_PA4	(0x00 + 4)
#define	NUC970_PA5	(0x00 + 5)
#define	NUC970_PA6	(0x00 + 6)
#define	NUC970_PA7	(0x00 + 7)
#define	NUC970_PA8	(0x00 + 8)
#define	NUC970_PA9	(0x00 + 9)
#define	NUC970_PA10	(0x00 + 10)
#define	NUC970_PA11	(0x00 + 11)
#define	NUC970_PA12	(0x00 + 12)
#define	NUC970_PA13	(0x00 + 13)
#define	NUC970_PA14	(0x00 + 14)
#define	NUC970_PA15	(0x00 + 15)

#define	NUC970_PB0	(0x20 + 0)
#define	NUC970_PB1	(0x20 + 1)
#define	NUC970_PB2	(0x20 + 2)
#define	NUC970_PB3	(0x20 + 3)
#define	NUC970_PB4	(0x20 + 4)
#define	NUC970_PB5	(0x20 + 5)
#define	NUC970_PB6	(0x20 + 6)
#define	NUC970_PB7	(0x20 + 7)
#define	NUC970_PB8	(0x20 + 8)
#define	NUC970_PB9	(0x20 + 9)
#define	NUC970_PB10	(0x20 + 10)
#define	NUC970_PB11	(0x20 + 11)
#define	NUC970_PB12	(0x20 + 12)
#define	NUC970_PB13	(0x20 + 13)
#define	NUC970_PB14	(0x20 + 14)
#define	NUC970_PB15	(0x20 + 15)

#define	NUC970_PC0	(0x40 + 0)
#define	NUC970_PC1	(0x40 + 1)
#define	NUC970_PC2	(0x40 + 2)
#define	NUC970_PC3	(0x40 + 3)
#define	NUC970_PC4	(0x40 + 4)
#define	NUC970_PC5	(0x40 + 5)
#define	NUC970_PC6	(0x40 + 6)
#define	NUC970_PC7	(0x40 + 7)
#define	NUC970_PC8	(0x40 + 8)
#define	NUC970_PC9	(0x40 + 9)
#define	NUC970_PC10	(0x40 + 10)
#define	NUC970_PC11	(0x40 + 11)
#define	NUC970_PC12	(0x40 + 12)
#define	NUC970_PC13	(0x40 + 13)
#define	NUC970_PC14	(0x40 + 14)
#define	NUC970_PC15	(0x40 + 15)

#define	NUC970_PD0	(0x60 + 0)
#define	NUC970_PD1	(0x60 + 1)
#define	NUC970_PD2	(0x60 + 2)
#define	NUC970_PD3	(0x60 + 3)
#define	NUC970_PD4	(0x60 + 4)
#define	NUC970_PD5	(0x60 + 5)
#define	NUC970_PD6	(0x60 + 6)
#define	NUC970_PD7	(0x60 + 7)
#define	NUC970_PD8	(0x60 + 8)
#define	NUC970_PD9	(0x60 + 9)
#define	NUC970_PD10	(0x60 + 10)
#define	NUC970_PD11	(0x60 + 11)
#define	NUC970_PD12	(0x60 + 12)
#define	NUC970_PD13	(0x60 + 13)
#define	NUC970_PD14	(0x60 + 14)
#define	NUC970_PD15	(0x60 + 15)

#define	NUC970_PE0	(0x80 + 0)
#define	NUC970_PE1	(0x80 + 1)
#define	NUC970_PE2	(0x80 + 2)
#define	NUC970_PE3	(0x80 + 3)
#define	NUC970_PE4	(0x80 + 4)
#define	NUC970_PE5	(0x80 + 5)
#define	NUC970_PE6	(0x80 + 6)
#define	NUC970_PE7	(0x80 + 7)
#define	NUC970_PE8	(0x80 + 8)
#define	NUC970_PE9	(0x80 + 9)
#define	NUC970_PE10	(0x80 + 10)
#define	NUC970_PE11	(0x80 + 11)
#define	NUC970_PE12	(0x80 + 12)
#define	NUC970_PE13	(0x80 + 13)
#define	NUC970_PE14	(0x80 + 14)
#define	NUC970_PE15	(0x80 + 15)

#define	NUC970_PF0	(0xA0 + 0)
#define	NUC970_PF1	(0xA0 + 1)
#define	NUC970_PF2	(0xA0 + 2)
#define	NUC970_PF3	(0xA0 + 3)
#define	NUC970_PF4	(0xA0 + 4)
#define	NUC970_PF5	(0xA0 + 5)
#define	NUC970_PF6	(0xA0 + 6)
#define	NUC970_PF7	(0xA0 + 7)
#define	NUC970_PF8	(0xA0 + 8)
#define	NUC970_PF9	(0xA0 + 9)
#define	NUC970_PF10	(0xA0 + 10)
#define	NUC970_PF11	(0xA0 + 11)
#define	NUC970_PF12	(0xA0 + 12)
#define	NUC970_PF13	(0xA0 + 13)
#define	NUC970_PF14	(0xA0 + 14)
#define	NUC970_PF15	(0xA0 + 15)

#define	NUC970_PG0	(0xC0 + 0)
#define	NUC970_PG1	(0xC0 + 1)
#define	NUC970_PG2	(0xC0 + 2)
#define	NUC970_PG3	(0xC0 + 3)
#define	NUC970_PG4	(0xC0 + 4)
#define	NUC970_PG5	(0xC0 + 5)
#define	NUC970_PG6	(0xC0 + 6)
#define	NUC970_PG7	(0xC0 + 7)
#define	NUC970_PG8	(0xC0 + 8)
#define	NUC970_PG9	(0xC0 + 9)
#define	NUC970_PG10	(0xC0 + 10)
#define	NUC970_PG11	(0xC0 + 11)
#define	NUC970_PG12	(0xC0 + 12)
#define	NUC970_PG13	(0xC0 + 13)
#define	NUC970_PG14	(0xC0 + 14)
#define	NUC970_PG15	(0xC0 + 15)

#define	NUC970_PH0	(0xE0 + 0)
#define	NUC970_PH1	(0xE0 + 1)
#define	NUC970_PH2	(0xE0 + 2)
#define	NUC970_PH3	(0xE0 + 3)
#define	NUC970_PH4	(0xE0 + 4)
#define	NUC970_PH5	(0xE0 + 5)
#define	NUC970_PH6	(0xE0 + 6)
#define	NUC970_PH7	(0xE0 + 7)
#define	NUC970_PH8	(0xE0 + 8)
#define	NUC970_PH9	(0xE0 + 9)
#define	NUC970_PH10	(0xE0 + 10)
#define	NUC970_PH11	(0xE0 + 11)
#define	NUC970_PH12	(0xE0 + 12)
#define	NUC970_PH13	(0xE0 + 13)
#define	NUC970_PH14	(0xE0 + 14)
#define	NUC970_PH15	(0xE0 + 15)

#define	NUC970_PI0	(0x100 + 0)
#define	NUC970_PI1	(0x100 + 1)
#define	NUC970_PI2	(0x100 + 2)
#define	NUC970_PI3	(0x100 + 3)
#define	NUC970_PI4	(0x100 + 4)
#define	NUC970_PI5	(0x100 + 5)
#define	NUC970_PI6	(0x100 + 6)
#define	NUC970_PI7	(0x100 + 7)
#define	NUC970_PI8	(0x100 + 8)
#define	NUC970_PI9	(0x100 + 9)
#define	NUC970_PI10	(0x100 + 10)
#define	NUC970_PI11	(0x100 + 11)
#define	NUC970_PI12	(0x100 + 12)
#define	NUC970_PI13	(0x100 + 13)
#define	NUC970_PI14	(0x100 + 14)
#define	NUC970_PI15	(0x100 + 15)

#define	NUC970_PJ0	(0x120 + 0)
#define	NUC970_PJ1	(0x120 + 1)
#define	NUC970_PJ2	(0x120 + 2)
#define	NUC970_PJ3	(0x120 + 3)
#define	NUC970_PJ4	(0x120 + 4)

typedef struct nuc970_eint_pins{
	u32	pin;
	irq_handler_t handler;
        u32   trigger;
        char *name;
}eint_wakeup_pins;



#endif /* __ASM_MACH_NUC970_GPIO_H*/
