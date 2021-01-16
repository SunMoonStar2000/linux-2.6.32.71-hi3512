/*
 *  linux/arch/arm/mach-hi3511-v100/pci.c
 *Copyright (c) 2006 Hisilicon Co., Ltd. 
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
 *
 * ARM hi3511 PCI driver.
 *
 * 12/2006 Initial version
 *
 */
#include <linux/autoconf.h>
#include <linux/kernel.h>
#include <linux/pci.h>
#include <linux/ptrace.h>
#include <linux/slab.h>
#include <linux/ioport.h>
#include <linux/interrupt.h>
#include <linux/spinlock.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/string.h>

#include <mach/hardware.h>
#include <mach/io.h>
#include <asm/irq.h>
#include <asm/system.h>
#include <asm/mach/pci.h>

#include <mach/early-debug.h>

#include <mach/pci.h>

static unsigned long hil_pci_clk_sel = 16;
static int __init hil_setup_pci_bus_clock(char *s)
{
	hil_pci_clk_sel = simple_strtoul(s, NULL, 0);
	return 1;
}
__setup("pciclksel=", hil_setup_pci_bus_clock);


/*
 * these spaces are mapped using the following base registers:
 *
 * Usage Local Bus Memory         Base/Map registers used
 *
 * Mem   B0000000 - B00003FF      BRIDGE REGISET
 * Mem   B0010000 - B001FFFF       IO
 * Mem   B0020000 - B7FFFFFF       LB_BASE0/LB_MAP0,  non prefetch
 * Mem   B8000000 - BFFFFFFF   	   LB_BASE1/LB_MAP1,  prefetch
 *
 */
#define PCIIRQ			0x1e 

static unsigned long pci_slot_ignore = 0;
unsigned long hi3511_pci_cfg_virt_base;
unsigned long hi3511_pci_virt_base;
unsigned long hi3511_sys_ctl_register_virt_base;
dma_addr_t hi3511_pci_dma_zone_base;
unsigned char *hi3511_pci_dma_zone_virt_base;


static int __init hi3511_pci_slot_ignore(char *str)
{
	int retval;
	int slot;

	while ((retval = get_option(&str,&slot))) {
		if ((slot < 0) || (slot > 31)) {
			printk("Illegal slot value: %d\n",slot);
		} else {
			pci_slot_ignore |= (1 << slot);
		}
	}
	return 1;
}

__setup("pci_slot_ignore=", hi3511_pci_slot_ignore);


static unsigned long __pci_addr(struct pci_bus *bus,
				unsigned int devfn, int offset)
{
	unsigned long addr,slot;
	unsigned int where;
	unsigned int busnr = bus->number;
	PCI_HAL_DEBUG(0, "ok, count its address %d", 1);
	/*
	 * Trap out illegal values
	 */
	if (offset > 255)
		BUG();
	if (busnr > 255)
		BUG();
	if (devfn > 255)
		BUG();
 	/* type 0*/
 	//there should has an address bit for the idsel signal of slave, the bit can be count out 
 	//according to the devfn.
	//printk("pcislot unmber %d\n",PCI_SLOT(devfn));
	where=((unsigned long)offset&(~0xffffff03U));
	if (busnr == 0) {
 		slot=PCI_SLOT(devfn);
 		if(slot>15) slot=15;
		addr=(( 0x01U<< (16+slot)) | (PCI_FUNC(devfn) << 8) | where);
		PCI_HAL_DEBUG(0,"pcibus unmber %d,pci address is 0x%lx\n",busnr,addr);
		return addr;
     // return (HI3511_PCI_CFG_BASE|( 0x01U<< (15+PCI_SLOT(devfn))) | (PCI_FUNC(devfn) << 8) | offset);
      /*type 1
	*return (HI3511_PCI_CFG_VIRT_BASE | (busnr << 16) |
	*	(PCI_SLOT(devfn) << 11) | (PCI_FUNC(devfn) << 8) | offset);
       */	
	} 
 	else {
		addr=((busnr << 16) |(PCI_SLOT(devfn) << 11) | (PCI_FUNC(devfn) << 8) | where|0x00000001U);	
		PCI_HAL_DEBUG(0,"pcibus unmber %d,pci address is 0x%lx\n",busnr,addr);
		return addr;	
	}	

}

static int pci_config_dma_read(unsigned int address,int size,unsigned int val)
{
	unsigned int control;
	int i;

/*write config address to dma pci address register */
	hi3511_bridge_ahb_writel(RDMA_PCI_ADDR, address);
/*write value address to dma ahb register*/
	hi3511_bridge_ahb_writel(RDMA_AHB_ADDR, val);
/*write dma control register*/
	control = (size<<8 |0|DMAR_CONF);
	hi3511_bridge_ahb_writel(RDMA_CONTROL, control);
	control |= DMAC_START;
	hi3511_bridge_ahb_writel(RDMA_CONTROL, control);

	for(i=0; i<10000 && (hi3511_bridge_ahb_readl(RDMA_CONTROL)&0x00000001); i++) {
		udelay(100);
	};
	if(hi3511_bridge_ahb_readl(RDMA_CONTROL)&0x00000001)
		printk(KERN_ERR "pci_config_dma_read timeout!\n");
	//for(i=0; i<100 ; i++) {
		//	udelay(100);
		//};

	return 0;
}

static int pci_config_dma_write(unsigned int address,int size,unsigned int val)
{
	unsigned int control;
	int i;

/*write config address to dma pci address register */
	hi3511_bridge_ahb_writel(WDMA_PCI_ADDR, address);
/*write value address to dma ahb register*/
	hi3511_bridge_ahb_writel(WDMA_AHB_ADDR, val);
/*write dma control register*/
	control = (size<<8 |0|DMAW_CONF);
	hi3511_bridge_ahb_writel(WDMA_CONTROL, control);
	control |= DMAC_START;
	hi3511_bridge_ahb_writel(WDMA_CONTROL, control);

	for(i=0; i<10000 && (hi3511_bridge_ahb_readl(WDMA_CONTROL)&0x00000001); i++) {
		udelay(100);
	};
	if(hi3511_bridge_ahb_readl(WDMA_CONTROL)&0x00000001)
		printk(KERN_ERR "pci_config_dma_write timeout!\n");
	//for(i=0; i<100 ; i++) {
	//		udelay(100);
	//	};

	return 0;
}

static int hi3511_read_config(struct pci_bus *bus, unsigned int devfn, int where,
				 int size, u32 *val)
{
	unsigned long shift;
	unsigned long addr = __pci_addr(bus, devfn, where);
	u32 v;
	int slot = PCI_SLOT(devfn);
	PCI_HAL_DEBUG(0, "ok, read it %d", 1);

	if (pci_slot_ignore & (1 << slot)) {
		/* Ignore this slot */
		switch (size) {
		case 1:
			v = 0xff;
			break;
		case 2:
			v = 0xffff;
			break;
		default:
			v = 0xffffffff;
		}
	} else {
	pci_config_dma_read(addr,4,hi3511_pci_dma_zone_base);
	PCI_HAL_DEBUG(0, "ok, read it %d", 2);
	/*	switch (size) {
		case 1:
			addr &= ~3;
			v = __raw_readb(addr);
			break;

		case 2:
			v = __raw_readl(addr & ~3);
			if (addr & 2) v >>= 16;
 			v &= 0xffff;
			break;

		default:
			addr &= ~3;
			v = __raw_readl(addr);
			break;
		}*/
	}
	v=*(volatile unsigned long *)hi3511_pci_dma_zone_virt_base;
	shift=(((unsigned long)where)&0x00000003UL)<<3;
	*val = (v>>shift);
	return PCIBIOS_SUCCESSFUL;
}

static int hi3511_write_config(struct pci_bus *bus, unsigned int devfn, int where,
				  int size, u32 val)
{
	unsigned long shift;
	unsigned long addr = __pci_addr(bus, devfn, where);
	int slot = PCI_SLOT(devfn);
	pci_config_dma_read(addr,4,hi3511_pci_dma_zone_base);
	switch(size){
			case 1:
			shift=(((unsigned long)where)&0x00000003UL)<<3;
			*(volatile unsigned int *)hi3511_pci_dma_zone_virt_base&=(~(0xff<<shift));
			*(volatile unsigned int *)hi3511_pci_dma_zone_virt_base|=(val<<shift);
				break;
		
			case 2:
			shift=(((unsigned long)where)&0x00000003UL)<<3;
			*(volatile unsigned int *)hi3511_pci_dma_zone_virt_base&=(~(0xffff<<shift));
			*(volatile unsigned int *)hi3511_pci_dma_zone_virt_base|=(val<<shift);
				break;
		
			case 4:
			shift=(((unsigned long)where)&0x00000003UL)<<3;
			*(volatile unsigned int *)hi3511_pci_dma_zone_virt_base&=(~(0xffffffff<<shift));
			*(volatile unsigned int *)hi3511_pci_dma_zone_virt_base|=(val<<shift);
				break;

	}
	
	PCI_HAL_DEBUG(0, "ok, write it %d", 1);
	if (pci_slot_ignore & (1 << slot)) {
		return PCIBIOS_SUCCESSFUL;
	}

	pci_config_dma_write(addr,4,hi3511_pci_dma_zone_base);
	/*switch (size) {
	case 1:
		__raw_writeb((u8)val, addr);
		break;

	case 2:
		__raw_writew((u16)val, addr);
		break;

	case 4:
		__raw_writel(val, addr);
		break;
	}*/

	return PCIBIOS_SUCCESSFUL;
}

static struct pci_ops pci_hi3511_ops = {
	.read	= hi3511_read_config,
	.write	= hi3511_write_config,
};


static struct resource io_mem = {
	.name	= "PCI I/O space",
	//.start  = HI3511_PCI_VIRT_BASE + (HI3511_PCI_MEM_BASE0 - HI3511_PCI_BASE), 
	.start	= HI3511_PCI_MEM_BASE0,
	.end	= HI3511_PCI_MEM_BASE0+HI3511_PCI_MEM_BASE0_SIZE-1,
	//.end	= HI3511_PCI_VIRT_BASE + (HI3511_PCI_MEM_BASE0 - HI3511_PCI_BASE) + HI3511_PCI_MEM_BASE0_SIZE-1,
	.flags	= IORESOURCE_IO,
};


static struct resource non_mem = {
	.name	= "PCI non-prefetchable",
	.start	= HI3511_PCI_MEM_BASE1,
	.end	= HI3511_PCI_MEM_BASE1+HI3511_PCI_MEM_BASE1_SIZE-1,
	.flags	= IORESOURCE_MEM,
};

static struct resource pre_mem = {
	.name	= "PCI prefetchable",
	.start	= HI3511_PCI_MEM_BASE2,
	.end	= HI3511_PCI_MEM_BASE2+HI3511_PCI_MEM_BASE2_SIZE-1,
	.flags	= IORESOURCE_MEM | IORESOURCE_PREFETCH,
};

static int __init pci_hi3511_setup_resources(struct resource **resource)
{
	int ret = 0;

	ret = request_resource(&iomem_resource, &io_mem);
	if (ret) {
		printk(KERN_ERR "PCI: unable to allocate I/O "
		       "memory region (%d)\n", ret);
		goto out;
	}

	ret = request_resource(&iomem_resource, &non_mem);
	if (ret) {
		printk(KERN_ERR "PCI: unable to allocate non-prefetchable "
		       "memory region (%d)\n", ret);
		goto release_io_mem;
	}
	ret = request_resource(&iomem_resource, &pre_mem);
	if (ret) {
		printk(KERN_ERR "PCI: unable to allocate prefetchable "
		       "memory region (%d)\n", ret);
		goto release_non_mem;
	}


	//sys->mem_offset = HI3511_PCI_MEM_OFFSET;
	//sys->io_offset  = HI3511_PCI_IO_OFFSET;


	/*
	 * bus->resource[0] is the IO resource for this bus
	 * bus->resource[1] is the mem resource for this bus
	 * bus->resource[2] is the prefetch mem resource for this bus
	 */
	resource[0] = &io_mem;
	resource[1] = &non_mem;
	resource[2] = &pre_mem;
	PCI_HAL_DEBUG(0, "ok, resources are start from %x", HI3511_PCI_MEM_BASE0);
	goto out;

 release_non_mem:
	release_resource(&non_mem);
 release_io_mem:
	release_resource(&pre_mem);
 out:
	return ret;
}

int __init pci_hi3511_setup(int nr, struct pci_sys_data *sys)
{
	
	int ret = 0;
	/*int i;
	int myslot = -1;
	unsigned long val;*/
	PCI_HAL_DEBUG(0, "ok, setup bus %d", 1);
	//if (nr == 0|nr==1) {
	if (nr == 0) {
		sys->mem_offset = 0;
		ret = pci_hi3511_setup_resources(sys->resource);
		PCI_HAL_DEBUG(0, "ok, resource ret %d", ret);
		if (ret < 0) {
			printk("pci_hi3511_setup: resources... oops?\n");
			goto out;
		}
	} else {
		printk("pci_hi3511_setup: resources... nr == 0??\n");
		goto out; 
	}
/*
	__raw_writel(HI3511_PCI_MEM_BASE0 >> 28,PCI_IMAP0);
	__raw_writel(HI3511_PCI_MEM_BASE1 >> 28,PCI_IMAP1);
	__raw_writel(HI3511_PCI_MEM_BASE2 >> 28,PCI_IMAP2);
*/
	//hi3511_bridge_ahb_writel(PCIAHB_ADDR_NP,HI3511_PCI_MEM_BASE0);
	//hi3511_bridge_ahb_writel(PCIAHB_ADDR_PF,HI3511_PCI_MEM_BASE1);
	//hi3511_bridge_ahb_writel(PCIAHB_SIZ_NP,HI3511_PCI_MEM_BASE0_SIZE);
	//hi3511_bridge_ahb_writel(PCIAHB_SIZ_PF,HI3511_PCI_MEM_BASE1_SIZE);


	
	


out:
	//return  ret;
	return  1;
}

static int
hi3511_pci_fault(unsigned long addr, unsigned int fsr, struct pt_regs *regs)
{
	return 0;
}

/*
 * map the specified device/slot/pin to an IRQ.   Different backplanes may need to modify this.
 */
static int __init hi3511_map_irq(struct pci_dev *dev, u8 slot, u8 pin)
{
	
	int irq;
	int devslot = PCI_SLOT(dev->devfn);
	irq = PCIIRQ;	/* only one irq number for pci */
	PCI_HAL_DEBUG(0, "ok, irq number registed in %d", irq);
	PCI_HAL_DEBUG(0,"map irq: slot %d, pin %d, devslot %d, irq: %d\n",slot,pin,devslot,irq);

	return irq;
}


struct pci_bus *pci_hi3511_scan_bus(int nr, struct pci_sys_data *sys)
{	
	PCI_HAL_DEBUG(0, "ok, find them %d", 1);
	return pci_scan_bus(sys->busnr, &pci_hi3511_ops, sys);
}

/*
 * V3_LB_BASE? - local bus address
 * V3_LB_MAP?  - pci bus address
 */
void __init pci_hi3511_preinit(void)
{
	int i=0;
	unsigned long reset_statu;
	/*unsigned long flags;
	unsigned int temp;
	int ret;*/
	PCI_HAL_DEBUG(0, "ok, bios is running %d", 1);
	//hi3511_pci_cfg_virt_base=HI3511_PCI_CFG_VIRT_BASE;
	//hi3511_pci_virt_base=HI3511_PCI_VIRT_BASE;
	hi3511_sys_ctl_register_virt_base=IO_ADDRESS(0x101e0000);
	
	//edb_putul(hi3511_pci_cfg_virt_base);
	//edb_putc('\n');
	//edb_putul(hi3511_pci_virt_base);
	//edb_putc('\n');
	//edb_putul(hi3511_sys_ctl_register_virt_base);
	//edb_putc('\n');
	//soft reset the bridge
	/*mw 0x101e003c 0x00120000  //config Sys_CTRL, set bridge to Host,and start the pci clock
	mw 0x101e001c 0x00000200   //soft reset PCI
	mw 0x101e001c 0x00000000   //clear soft reset*/
	reset_statu =__raw_readl( hi3511_sys_ctl_register_virt_base + REG_SC_PERCTRL4);
	__raw_writel(reset_statu | 0x00120000, hi3511_sys_ctl_register_virt_base + REG_SC_PERCTRL4);

	reset_statu=__raw_readl( hi3511_sys_ctl_register_virt_base+0x1c);
	__raw_writel(reset_statu|0x00000100, hi3511_sys_ctl_register_virt_base+0x1c);
	for(;i<100;i++);
	reset_statu=__raw_readl( hi3511_sys_ctl_register_virt_base+0x1c);
	reset_statu&=0xfffffeff;
	__raw_writel(reset_statu, hi3511_sys_ctl_register_virt_base+0x1c);
	for(i=0;i<100;i++);


//set pci req4 and gnt4
	reset_statu =__raw_readl( hi3511_sys_ctl_register_virt_base + 0x20);
	__raw_writel(reset_statu | 0x2000, hi3511_sys_ctl_register_virt_base + 0x20);
	
	/*
	 * Hook in our fault handler for PCI errors
	 */
	hook_fault_code(4, hi3511_pci_fault, SIGBUS, "external abort on linefetch");
	hook_fault_code(6, hi3511_pci_fault, SIGBUS, "external abort on linefetch");
	hook_fault_code(8, hi3511_pci_fault, SIGBUS, "external abort on non-linefetch");
	hook_fault_code(10, hi3511_pci_fault, SIGBUS, "external abort on non-linefetch");
//	edb_trace(10);
	//spin_lock_irqsave(&hi3511_lock, flags);
	//spin_unlock_irqrestore(&hi3511_lock, flags);

	hi3511_pci_dma_zone_virt_base = dma_alloc_coherent(NULL, SZ_4K, &hi3511_pci_dma_zone_base, GFP_DMA);

	if(hi3511_pci_dma_zone_virt_base == 0){

		PCI_HAL_DEBUG(0, "hi3511_pci_dma_zone_virt_base alloc error %d", 1);

		return ;
	}

	memset(hi3511_pci_dma_zone_virt_base ,0xa5, SZ_4K);
	//set host bridge address when it acts as a slave in a transaction
	/*	hi3511_bridge_ahb_writel(PCIAHB_ADDR_NP, 0xf1000001);
	hi3511_bridge_ahb_writel(PCIAHB_ADDR_PF, 0xf0000001);
	hi3511_bridge_ahb_writel(PCIAHB_SIZ_NP, 0xff000000);
	hi3511_bridge_ahb_writel(PCIAHB_SIZ_PF, 0xff000000);*/
	//hi3511_bridge_ahb_writel(PCIAHB_ADDR_PF, 0xf0000001);
	//hi3511_bridge_ahb_writel(PCIAHB_SIZ_PF, 0xf0000000);
	hi3511_bridge_ahb_writel(PCIAHB_ADDR_PF, PHYS_OFFSET+1);
	hi3511_bridge_ahb_writel(PCIAHB_SIZ_PF, 0xf0000000);

}

void __init pci_hi3511_postinit(void)
{
	//unsigned int control;
	//unsigned int pci_cmd;
	//edb_trace(10);
	//pci_cmd = PCI_COMMAND_MEMORY |PCI_COMMAND_MASTER | PCI_COMMAND_INVALIDATE;
	//set host status
	//	hi3511_bridge_ahb_writew(HI3511_PCI_CMD, pci_cmd);
	//clear host interrupt register,close interrupt
	hi3511_bridge_ahb_writel(CPU_ISTATUS, 0xffffffff);
	hi3511_bridge_ahb_writel(CPU_IMASK, 0x01000000);
	//hi3511_bridge_ahb_writel(CPU_ICMD, 0xffffffff);
	PCI_HAL_DEBUG(0,"istatus and imask %d \n",hi3511_bridge_ahb_readl(CPU_IMASK) );
	PCI_HAL_DEBUG(0,"istatus and imask %d \n",hi3511_bridge_ahb_readl(CPU_ISTATUS) );

/*hi3511_bridge_ahb_writel(WDMA_PCI_ADDR, 0x00010014);
hi3511_bridge_ahb_writel(WDMA_AHB_ADDR, 0xf3000000);
hi3511_bridge_ahb_writel(WDMA_CONTROL, 0x000004b1);

hi3511_bridge_ahb_writel(RDMA_PCI_ADDR, 0x00010014);*/

//hi3511_bridge_ahb_writel(RDMA_AHB_ADDR, 0xf3000000);
/*write dma control register*/
	//control = (4<<8 |DMAC_INT_EN|DMAR_CONF);
//hi3511_bridge_ahb_writel(RDMA_CONTROL, 0x000004a1);
	//control |= DMAC_START;
	//hi3511_bridge_ahb_writel(RDMA_CONTROL, control);

	//edb_trace(10);
}

static struct hw_pci hi3511_pci __initdata = {
	.swizzle		= NULL,
	.map_irq		= hi3511_map_irq,
	.nr_controllers		= 1,
	//.nr_controllers		= 1,
	.setup			= pci_hi3511_setup,
	.scan			= pci_hi3511_scan_bus,
	.preinit		= pci_hi3511_preinit,
	.postinit		= pci_hi3511_postinit,
};

static int __initdata hil_pcimod=0;

static int __init hil_setup_pcimod(char *s)
{
	if(strcmp(s, "host") ==0)
		hil_pcimod = 1;

	return 1;
}
__setup("pcimod=", hil_setup_pcimod);

static int __init hi3511_pci_init(void)
{
/*
0000：6分频。 0
0001：7分频。 1
0010：8分频。 2 
0011：9分频。 3
0100：10分频。4
0101：11分频。5
0110：12分频。6
0111：13分频。7
1000：14分频。8
1001：15分频。9
1010：16分频。A
1011：18分频。B
1100：20分频。C
1101：24分频。D
*/
	unsigned int reg = 0;
	static const unsigned char div[25] = {
	[6]=0x0,
	[7]=0x1,
	[8]=0x2,
	[9]=0x3,
	[10]=0x4,
	[11]=0x5,
	[12]=0x6,
	[13]=0x7,
	[14]=0x8,
	[15]=0x9,
	[16]=0xA,
	[18]=0xB,
	[20]=0xC,
	[24]=0xD
	};

	reg = readl(IO_ADDRESS(REG_BASE_SCTL + REG_SC_PERCTRL2));
	reg &= ~(0xf << 20);
	reg |= (div[hil_pci_clk_sel] << 20);
	writel(reg,IO_ADDRESS(REG_BASE_SCTL + REG_SC_PERCTRL2));

	if(hil_pcimod ==0) {
		printk("Hisilicon PCI work at slave mode.\n");
		return 0;
	}
	
	PCI_HAL_DEBUG(0, "begin %d", 1);
	//edb_trace(10);

	pci_common_init(&hi3511_pci);

	return 0;
}

subsys_initcall(hi3511_pci_init);

