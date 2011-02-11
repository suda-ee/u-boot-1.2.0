/*
 *
 * Copyright (C) 2004 Texas Instruments.
 *
 * ----------------------------------------------------------------------------
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
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 * ----------------------------------------------------------------------------
 Modifications:
 ver. 1.0: Mar 2007, Suresh Rajashekara (Based on the file davinci.c by
 *                                       Swaminathan S)
 *	 : Nov 2007, Kaustubh Sarwate (Updated with PCI based changes)
 */

#include <common.h>
#include <i2c.h>

#define PLL0_PLLM   *(volatile unsigned int *)0x01C40910
#define PLL1_PLLM   *(volatile unsigned int *)0x01C40D10
#define PLL1_DIV1   *(volatile unsigned char *)0x01C40D18
#define PBBPR       *(volatile unsigned int *)0x20000020
void davinci_hd_psc_enable(void);
void enable_tcm_cp15(void);

void raise()
{
        printf ("*** something is wrong... please reset ***\n");
}

void abort()
{
        printf ("*** something is wrong... please reset ***\n");
}


/*******************************************
 Routine: delay
 Description:  Delay function
*******************************************/
static inline void delay (unsigned long loops)
{
    __asm__ volatile ("1:\n"
		      "subs %0, %1, #1\n"
		      "bne 1b":"=r" (loops):"0" (loops));
}

/*******************************************
 Routine: board_init
 Description:  Board Initialization routine
*******************************************/
int board_init (void)
{
    DECLARE_GLOBAL_DATA_PTR;

	/* Arch Number. __Need to register__ */
    gd->bd->bi_arch_number = 1380;

      	/* Adress of boot parameters */
    gd->bd->bi_boot_params = LINUX_BOOT_PARAM_ADDR;

	/* Configure MUX settings ? */

	/* Power on required peripherals */
    davinci_hd_psc_enable();

    inittimer ();

    return 0;
}

#define PTCMD               *( volatile unsigned int* )( 0x01C41120 )
#define PTSTAT              *( volatile unsigned int* )( 0x01C41128 )
#define PDSTAT              *( volatile unsigned int* )( 0x01C41200 )
#define PDCTL               *( volatile unsigned int* )( 0x01C41300 )

/* PSC Registers */
#define PSC_ADDR            0x01C41000

#define PTCMD               ( PSC_ADDR + 0x120 ) /* Power domain transition
						  * commmand register */
#define PTSTAT              ( PSC_ADDR + 0x128 ) /* Power domain transition status
						  * register */

/**************************************
 Routine: davinci_hd_psc_enable
 Description:  Enable PSC domains
**************************************/
void davinci_hd_psc_enable ( void )
{
    unsigned int alwaysOnPdNum = 0, dspPdNum = 1;
    int waiting;
    unsigned int state;

    /* Note this function assumes that the Power Domains are already on */
    *(volatile unsigned int*) (PSC_ADDR+0xA00+4*14) = *(unsigned int*) (PSC_ADDR+0xA00+4*14) | 0x003; /* EMAC */
    *(volatile unsigned int*) (PSC_ADDR+0xA00+4*15) = *(unsigned int*) (PSC_ADDR+0xA00+4*15) | 0x003; /* VDCE */
    *(volatile unsigned int*) (PSC_ADDR+0xA00+4*16) = *(unsigned int*) (PSC_ADDR+0xA00+4*16) | 0x003; /* Video Port */
    *(volatile unsigned int*) (PSC_ADDR+0xA00+4*17) = *(unsigned int*) (PSC_ADDR+0xA00+4*17) | 0x003; /* Video Port */
    *(volatile unsigned int*) (PSC_ADDR+0xA00+4*20) = *(unsigned int*) (PSC_ADDR+0xA00+4*20) | 0x003; /* DDR2 */
    *(volatile unsigned int*) (PSC_ADDR+0xA00+4*21) = *(unsigned int*) (PSC_ADDR+0xA00+4*21) | 0x003; /* EMIFA */
    *(volatile unsigned int*) (PSC_ADDR+0xA00+4*26) = *(unsigned int*) (PSC_ADDR+0xA00+4*26) | 0x003; /* UART0 */
    *(volatile unsigned int*) (PSC_ADDR+0xA00+4*27) = *(unsigned int*) (PSC_ADDR+0xA00+4*27) | 0x003; /* UART1 */
    *(volatile unsigned int*) (PSC_ADDR+0xA00+4*28) = *(unsigned int*) (PSC_ADDR+0xA00+4*28) | 0x003; /* UART2 */
    *(volatile unsigned int*) (PSC_ADDR+0xA00+4*31) = *(unsigned int*) (PSC_ADDR+0xA00+4*31) | 0x003; /* I2C */
    *(volatile unsigned int*) (PSC_ADDR+0xA00+4*34) = *(unsigned int*) (PSC_ADDR+0xA00+4*34) | 0x003; /* TIMER0 */
    *(volatile unsigned int*) (PSC_ADDR+0xA00+4*35) = *(unsigned int*) (PSC_ADDR+0xA00+4*35) | 0x003; /* TIMER1 */
    
    
	/* Set PTCMD.GO to 0x1 to initiate the state transtion for Modules in
	 * the ALWAYSON Power Domain */
    *(volatile unsigned int*) PTCMD = (1<<alwaysOnPdNum);

	/* Wait for PTSTAT.GOSTAT0 to clear to 0x0 */
    while(! (((*(volatile unsigned int*) PTSTAT >> alwaysOnPdNum) & 0x00000001) == 0));

	/* Enable GIO3.3V cells used for EMAC (???) */
#define VDD3P3V_PWDN        0x01c40048
    //*(volatile unsigned int*) VDD3P3V_PWDN = 0x180000c0;
    *(volatile unsigned int*) VDD3P3V_PWDN = 0x80000c0;

#define PINMUX0     0x01C40000
#define PINMUX1     0x01C40004

	/* Select UART function on UART0 */
    *(volatile unsigned int *)PINMUX0 &= ~(0x0000003f << 18);
    *(volatile unsigned int *)PINMUX1 &= ~(0x00000003);

#ifndef CFG_PCI_BOOT
	/* Enable AEMIF pins */
    *(volatile unsigned int*) PINMUX0 &= ~(0x00000007);
#endif	/* CFG_PCI_BOOT */
    /* Enable USB */
    *(volatile unsigned int*) PINMUX0 &= ~(0x80000000);

    /* Set the Bus Priority Register to appropriate value */
    PBBPR = 0x20;
}

void enable_tcm_cp15(void)
{
        /* Set to SUPERVISOR  MODE */
        asm (" mrs R0, cpsr\n"
                " bic r0, r0, #0x1F\n"
                " orr r0, r0, #0x13\n"
                " msr cpsr, r0");

        /* Read ITCM */
        asm (" mrc p15, 0, R3, c9, c1, 1\n"
                " nop\n"
                " nop");

        /* Enable ITCM */
        asm(" mov R0, #0x1\n"
                " mcr p15, 0, R0, c9, c1, 1\n"
                " nop\n"
                " nop");

        /* Read Back the ITCM value to check the ITCM Enable function */
        asm(" mrc p15, 0, R4, c9, c1, 1\n"
                " nop\n"
                " nop");

        /* Read DTCM */
        asm(" mrc p15, 0, R3, c9, c1, 0\n"
                " nop\n"
                " nop");

        /* Create DTCM enable mask */
        asm(" ldr R0, =0x10\n"
                " mov R0, R0, lsl #12\n"
                " nop\n"
                " orr R0, R0, #0x1\n"
                " orr R0, R0, R3");

        /* Enable DTCM */
        asm(" mcr p15, 0, R0, c9, c1, 0\n"
                " nop\n"
                " nop");

        /* Read Back the DTCM value to check the DTCM Enable function */
        asm(" mrc p15, 0, R5, c9, c1, 0\n"
                " nop\n"
                " nop");
}

/******************************
 Routine: misc_init_r
 Description:  Misc. init
******************************/
int misc_init_r (void)
{
    char temp[20];
    char io_exp[1] = {0x00, 0x00};
    char emac_read_addr [10] = { 0x7f, 0 }, i= 0;
    int clk = 0;

    clk = ((PLL1_PLLM + 1) * 27) / (PLL1_DIV1 + 1);

    printf ("ARM Clock :- %dMHz\n", ((((PLL0_PLLM + 1) * 27 ) / 2)) );
    printf ("DDR Clock :- %dMHz\n", (clk/2));

    i2c_write (0x50, 0x00, 1, emac_read_addr, 2);
    i2c_read (0x50, 0x00, 1, emac_read_addr, 6);
    temp[0] = (emac_read_addr[0] & 0xF0) >> 4;
    temp[1] = (emac_read_addr[0] & 0x0F);
    temp[2] = ':';
    temp[3] = (emac_read_addr[1] & 0xF0) >> 4;
    temp[4] = (emac_read_addr[1] & 0x0F);
    temp[5] = ':';
    temp[6] = (emac_read_addr[2] & 0xF0) >> 4;
    temp[7] = (emac_read_addr[2] & 0x0F);
    temp[8] = ':';
    temp[9] = (emac_read_addr[3] & 0xF0) >> 4;
    temp[10]= (emac_read_addr[3] & 0x0F);
    temp[11]= ':';
    temp[12]= (emac_read_addr[4] & 0xF0) >> 4;
    temp[13]= (emac_read_addr[4] & 0x0F);
    temp[14]= ':';
    temp[15]= (emac_read_addr[5] & 0xF0) >> 4;
    temp[16]= (emac_read_addr[5] & 0x0F);
                                                                               
    for (i = 0; i < 17; i++)
    {
        if (temp[i] == ':')
	    continue;
        else if (temp[i] >= 0 && temp[i] <= 9)
	    temp[i] = temp[i] + 48;
        else
	    temp[i] = temp[i] + 87;
    }
                   
    temp [17] = 0;                                                            
    if ((emac_read_addr [0] != 0xFF) ||
	(emac_read_addr [1] != 0xFF) ||	
	(emac_read_addr [2] != 0xFF) ||	
	(emac_read_addr [3] != 0xFF) ||	
	(emac_read_addr [4] != 0xFF) ||	
	(emac_read_addr [5] != 0xFF))
    { 
	setenv ("ethaddr", temp);
    }

    /* enable the ITCM and DTCM */
    enable_tcm_cp15();

/*	setenv ("ethaddr", "00:0e:99:02:b0:0f");
	setenv ("serverip", "192.168.1.101");
	setenv ("bootfile", "manju");
	setenv ("bootcmd", "dhcp;bootm");
	setenv ("bootargs", "console=ttyS0,115200n8 noinitrd rw ip=dhcp root=/dev/nfs nfsroot=192.168.1.101:/opt/montavista/pro/devkit/arm/v5t_le/target, nolock mem=120M");*/
#ifdef CFG_PCI_BOOT
	// Use the following command to run a boot script at run time
	setenv ("bootcmd", "autoscr 0x82080000");
#endif	/* CFG_PCI_BOOT */
    return (0);
}

/******************************
 Routine: dram_init
 Description:  Memory Info
******************************/
int dram_init (void)
{
    DECLARE_GLOBAL_DATA_PTR;

    gd->bd->bi_dram[0].start = PHYS_SDRAM_1;
    gd->bd->bi_dram[0].size = PHYS_SDRAM_1_SIZE;

    return 0;
}

#ifdef CFG_PCI_BOOT
char * env_name_spec = "nowhere";
int saveenv (void)
{
	return 0;
}
#endif	/* CFG_PCI_BOOT */

