/* linux/include/asm-arm/arch-hi3511_v100/platform.h
*
* Copyright (c) 2006 Hisilicon Co., Ltd. 
*
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation; either version 2 of the License, or
* (at your option) any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with this program; if not, write to the Free Software
* Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307  USA
*
*/

#include <linux/autoconf.h>

#ifndef	__HI_CHIP_REGS_H__
#define	__HI_CHIP_REGS_H__

#define REG_BASE_SMI	0x10100000
#define REG_SMI_IOSIZE	SZ_64K

#define REG_BASE_DMAC	0x10130000
#define REG_DMAC_IOSIZE	SZ_64K

#define REG_BASE_INTC	0x10140000
#define REG_INTC_IOSIZE 	SZ_64K

#define REG_BASE_DDRC	0x10150000
#define REG_DDRC_IOSIZE	SZ_64K

#define REG_BASE_SCTL	0x101E0000
#define REG_SCTL_IOSIZE	SZ_4K

#define REG_BASE_WATCHDOG	0x101E1000
#define REG_WATCHDOG_IOSIZE	SZ_4K

#define REG_BASE_TIMER12	0x101E2000
#define REG_TIMER12_IOSIZE	SZ_4K

#define REG_BASE_TIMER34	0x101E3000
#define REG_TIMER34_IOSIZE	SZ_4K

#define REG_BASE_GPIO0	0x101E4000
#define REG_GPIO0_IOSIZE	SZ_4K

#define REG_BASE_GPIO1	0x101E5000
#define REG_GPIO1_IOSIZE	SZ_4K

#define REG_BASE_GPIO2	0x101E6000
#define REG_GPIO2_IOSIZE	SZ_4K

#define REG_BASE_GPIO3	0x101E7000
#define REG_GPIO3_IOSIZE	SZ_4K

#define REG_BASE_RTC	0x101E8000
#define REG_RTC_IOSIZE	SZ_4K

#define REG_BASE_IR	0x101E9000
#define REG_IR_IOSIZE	SZ_4K

#define REG_BASE_UART0	0x101F1000	/* Uart 0 */
#define REG_UART0_IOSIZE	SZ_4K

#define REG_BASE_UART1	0x101F2000	/* Uart 1 */
#define REG_UART1_IOSIZE	SZ_4K

#define REG_BASE_UART2	0x101F3000	/* Uart 2 */
#define REG_UART2_IOSIZE	SZ_4K

#define REG_BASE_SSP	0x101F4000
#define REG_SSP_IOSIZE	SZ_4K

#define REG_BASE_I2C	0x101F6000
#define REG_I2C_IOSIZE	SZ_4K

#define REG_BASE_GPIO4	0x101F7000
#define REG_GPIO4_IOSIZE	SZ_4K

#define REG_BASE_GPIO5	0x101F8000
#define REG_GPIO5_IOSIZE	SZ_4K

#define REG_BASE_GPIO6	0x101F9000
#define REG_GPIO6_IOSIZE	SZ_4K

#define REG_BASE_GPIO7	0x101FA000
#define REG_GPIO7_IOSIZE	SZ_4K

#define REG_BASE_SIO_0	0x80080000
#define REG_SIO_0_IOSIZE	SZ_64K

#define REG_BASE_USBOTG	0x80090000
#define REG_USBOTG_IOSIZE	SZ_64K

#define REG_BASE_MMC	0x90020000
#define REG_MMC_IOSIZE	SZ_64K

#define REG_BASE_ETH	0x90030000
#define REG_ETH_IOSIZE	SZ_64K

#define REG_BASE_CIPHER	0x90040000
#define REG_CIPHER_IOSIZE	SZ_64K

#define REG_BASE_VOU	0x90050000
#define REG_VOU_IOSIZE	SZ_64K

#define REG_BASE_VIU	0x90060000
#define REG_VIU_IOSIZE	SZ_64K

#define REG_BASE_DSU	0x90070000
#define REG_DSU_IOSIZE	SZ_64K

#define REG_BASE_VEDU	0x90080000
#define REG_VEDU_IOSIZE	SZ_64K

#define REG_BASE_SIO_1	0x900A0000
#define REG_SIO_1_IOSIZE	SZ_64K

#define REG_BASE_USB11	0xA0000000
#define REG_USB11_IOSIZE	SZ_64K

#define REG_BASE_ExAHB_ARBITER	0xA0010000
#define REG_ExAHB_ARBITER_IOSIZE	SZ_64K

#define REG_BASE_DMAMEM_AHB_ARBITER	0xA0020000
#define REG_DMAMEM_AHB_ARBITER_IOSIZE	SZ_64K

#define REG_BASE_PCI_BR	0xB0000000
#define REG_PCI_BR_IOSIZE	SZ_4K

#define REG_BASE_PCI_IO	0xB0010000
#define REG_PCI_IO_IOSIZE	SZ_64K

#define REG_BASE_PCI_NP	0xB0020000
#define REG_PCI_NP_IOSIZE	0x2000000
#define REG_BASE_PCI_NP_VIRT	0xF8000000

#define REG_BASE_PCI_PF	0xB8000000
#define REG_PCI_PF_IOSIZE	0x2000000
#define REG_BASE_PCI_PF_VIRT	0xFA000000
#define HISILICON_PCI_BR_BASE	0xB0000000
#define HISILICON_PCI_BR_IOSIZE	SZ_4K

#define HISILICON_PCI_IO_BASE	0xB0010000
#define HISILICON_PCI_IO_IOSIZE	SZ_64K

#define HISILICON_PCI_NP_BASE	0xB0020000
#define HISILICON_PCI_NP_IOSIZE	0x2000000
#define HISILICON_PCI_NP_VIRT_BASE	0xF8000000
/* SYSTEM CONTROL REG */
#define REG_SC_CTRL	0x000
#define REG_SC_SYSSTAT	0x004
#define REG_SC_ITMCTRL	0x008
#define REG_SC_IMSTAT	0x00C
#define REG_SC_XTALCTRL	0x010
#define REG_SC_PLLCTRL	0x014
#define REG_SC_PLLFCTRL	0x018
#define REG_SC_PERCTRL0	0x01C
#define REG_SC_PERCTRL1	0x020
#define REG_SC_PEREN	0x024
#define REG_SC_PERDIS	0x028
#define REG_SC_PERCLKEN	0x02C
#define REG_SC_RESERVED	0x030
#define REG_SC_PERCTRL2	0x034
#define REG_SC_PERCTRL3	0x038
#define REG_SC_PERCTRL4	0x03C
#define REG_SC_PERLOCK	0x044
#define REG_SC_SYSID	0xEE0

/* SMI REG */
#define REG_SMI_BIDCYR1			0x000
#define REG_SMI_BWSTRDR1		0x004
#define REG_SMI_BWSTWRR1		0x008
#define REG_SMI_BWSTOENR1		0x00C
#define REG_SMI_BWSTWENR1		0x010
#define REG_SMI_BCR1			0x014
#define REG_SMI_BSR1			0x018
#define REG_SMI_BWSTBRDR1		0x01C
#define REG_SMI_BIDCYR0			0x0E0
#define REG_SMI_BWSTRDR0		0x0E4
#define REG_SMI_BWSTWRR0		0x0E8
#define REG_SMI_BWSTOENR0		0x0EC
#define REG_SMI_BWSTWENR0		0x0F0
#define REG_SMI_BCR0			0X0F4
#define REG_SMI_BSR0			0x0F8
#define REG_SMI_BWSTBRDR0		0x0FC
#define REG_SMI_SR				0x200
#define REG_SMI_CR				0x204



#define REG_VALUE_SC_NOLOCK 0x1ACCE551
#define REG_VALUE_SC_LOCKED 0x00000001

#define REG_INTC_IRQSTATUS	0x000
#define REG_INTC_FIQSTATUS	0x004
#define REG_INTC_RAWSTATUS	0x008
#define REG_INTC_INTSELECT	0x00C
#define REG_INTC_INTENABLE	0x010
#define REG_INTC_INTENCLEAR	0x014
#define REG_INTC_SOFTINT	0x018
#define REG_INTC_SOFTINTCLEAR	0x01C
#define REG_INTC_PROTECTION	0x020

#define INTNR_IRQ_START	0
#define INTNR_IRQ_END	31

#define INTNR_WATCHDOG					0	/* Watchdog timer */
#define INTNR_SOFTINT					1	/* Software interrupt */
#define INTNR_COMMRx					2	/* Debug Comm Rx interrupt */
#define INTNR_COMMTx					3	/* Debug Comm Tx interrupt */
#define INTNR_TIMER_0_1					4	/* Timer 0 and 1 */
#define INTNR_TIMER_2_3                 5	/* Timer 2 and 3 */
#define INTNR_GPIO_0                    6	/* GPIO 0 */
#define INTNR_GPIO_1                    7	/* GPIO 1 */
#define INTNR_GPIO_2_7                  8	/* GPIO 2 */
#define INTNR_IR						9	/* GPIO 3 */
#define INTNR_RTC						10	/* Real Time Clock */
#define INTNR_SSP						11	/* Synchronous Serial Port */
#define INTNR_UART0						12
#define INTNR_UART1						13
#define INTNR_UART2						14
#define INTNR_ETH						15
#define INTNR_VOU						16
#define INTNR_DMAC						17
#define INTNR_SIO_1						18
#define INTNR_I2C						19
#define INTNR_USB11						20
#define INTNR_CIPHER					21
#define INTNR_EXTINT					22
#define INTNR_USBOTG					23
#define INTNR_MMC						24
#define INTNR_VIU						25
#define INTNR_DSU						26
#define INTNR_SIO_0						27
#define INTNR_VEDU						28
#define INTNR_RESERVED_0				29
#define INTNR_PCI						30
#define INTNR_TDE						31


#define REG_TIMER_RELOAD	0x000
#define REG_TIMER_VALUE		0x004
#define REG_TIMER_CONTROL	0x008
#define REG_TIMER_INTCLR	0x00C
#define REG_TIMER_RIS		0x010
#define REG_TIMER_MIS		0x014
#define REG_TIMER_BGLOAD	0x018

#define REG_TIMER1_RELOAD       0x020
#define REG_TIMER1_VALUE        0x024
#define REG_TIMER1_CONTROL      0x028
#define REG_TIMER1_INTCLR       0x02C
#define REG_TIMER1_RIS          0x030
#define REG_TIMER1_MIS          0x034
#define REG_TIMER1_BGLOAD       0x038

#define REG_DDRC_STATUS		0x000
#define REG_DDRC_CTRL		0x004
#define REG_DDRC_EMRS01		0x008
#define REG_DDRC_EMRS23		0x00C
#define REG_DDRC_CONFIG		0x010
#define REG_DDRC_TIMING0	0x020
#define REG_DDRC_TIMING1	0x024
#define REG_DDRC_TIMING2	0x028
#define REG_DDRC_TIMING3	0x02C
#define REG_DDRC_ODT_CONFIG	0x040
#define REG_DDRC_PHY_CONFIG	0x060
#define REG_DDRC_DLL_STATUS	0x078
#define REG_DDRC_DLL_CONFIG	0x07C
#define REG_DDRC_QOS_CONFIG	0x094
#define REG_DDRC_CH0_QOS	0x098
#define REG_DDRC_CH1_QOS	0x09C
#define REG_DDRC_CH2_QOS	0x0A0
#define REG_DDRC_CH3_QOS	0x0A4
#define REG_DDRC_CH4_QOS	0x0A8
#define REG_DDRC_CH5_QOS	0x0AC
#define REG_DDRC_CH6_QOS	0x0B0
#define REG_DDRC_CH7_QOS	0x0B4

#define DDRC_BUSWITH_32BITS     1
#define DDRC_BUSWITH_16BITS     0

#define DDRC_CHIPCAP_64Mb       0
#define DDRC_CHIPCAP_128Mb      1
#define DDRC_CHIPCAP_256Mb      2
#define DDRC_CHIPCAP_512Mb      3
#define DDRC_CHIPCAP_1Gb      	4
#define DDRC_CHIPCAP_2Gb      	5

#define DDRC_CHIP_8BITS         0
#define DDRC_CHIP_16BITS        1
#define DDRC_CHIP_32BITS        2

#define DDRC_CHIP_4BANK         0
#define DDRC_CHIP_8BANK         1

#define DDRC_READDELAY_2        0
#define DDRC_READDELAY_2_5      1
#define DDRC_READDELAY_3        2
#define DDRC_READDELAY_4        3
#define DDRC_READDELAY_5        4
#define DDRC_READDELAY_6        5


#define MEM_BASE_ITCM	0x00000000
#define MEM_SIZE_ITCM	0x00000800
#define MEM_CONF_ITCM_SIZE	3

#define MEM_BASE_DDR	0xE0000000
#define MEM_BASE_FLASH	0x34000000

#define PLLC_TO_M_N_OD(M, N, OD, pllc) do{ M = ((pllc)>>4)&0xFF; N = (pllc)&0xF; OD = ((pllc)>>12)&0x3; }while(0)
#define M_N_OD_TO_FREQ(M, N, OD, fxin) (((((fxin / 10) * (M))  / ( (N) * (1<<(OD)))) >> 1) * 10 )
#endif /*End of __HI_CHIP_REGS_H__ */

