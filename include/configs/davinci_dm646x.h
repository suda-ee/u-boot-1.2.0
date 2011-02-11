/*
 * (C) Copyright 2003
 * Texas Instruments.
 * Swaminathan S <swami.iyer@ti.com>
 * Configuation settings for the TI DaVinci EVM board.
 *
 * See file CREDITS for list of people who contributed to this
 * project.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	 See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */

#ifndef __CONFIG_H
#define __CONFIG_H

/* Chip Configurations */
/*============================================================================*/
#define CFG_DAVINCI_HD
#define CFG_DM6467_HD
#define CONFIG_ARM926EJS	/* This is an arm926ejs CPU core  	  */
#define CONFIG_SYS_CLK_FREQ	297000000	/* Arm Clock frequency    */
#define CFG_TIMERBASE		0x01C21400	/* use timer 0 		  */
#define CFG_HZ			148500000	/* Timer Input clock freq */	
/* #define CFG_PCI_BOOT	*/			/* Build for PCI Booting  */
/*============================================================================*/

#ifdef CFG_PCI_BOOT
/* PCI Boot Info */
/*============================================================================*/

#define CONFIG_INITRD_TAG  	(1)
#define CFG_ENV_IS_NOWHERE	(1)		/* PCI booting */
#define CFG_ENV_ADDR		(0x80000000)
#define CFG_ENV_SIZE		(64 * 2048)

#else
/* Flash Boot info */
/*============================================================================*/
/*#define CFG_ENV_IS_IN_FLASH 	1*/		/* U-Boot env in NOR Flash   */

#ifndef CFG_ENV_IS_IN_FLASH
#define CONFIG_INITRD_TAG  	1
#define CFG_ENV_IS_IN_NAND 	1               /* U-Boot env in NAND Flash  */
#define CFG_ENV_SECT_SIZE	2048		/* Env sector Size */
#define CFG_ENV_SIZE		(64 * 2048)
#else
#define CONFIG_INITRD_TAG  	1
#define CFG_ENV_SECT_SIZE	CFG_FLASH_SECT_SZ	/* Env sector Size */
#define CFG_ENV_SIZE		CFG_FLASH_SECT_SZ
#define CFG_ENV_ADDR		(CFG_FLASH_BASE + 0x20000)
#endif

#endif /* CFG_PCI_BOOT */

/*
 * NOR Flash Info 
 */
/*============================================================================*/
#define CONFIG_CS0_BOOT				/* Boot from Flash 	     */
#define CFG_MAX_FLASH_BANKS	1		/* max number of flash banks */
#define CFG_FLASH_SECT_SZ	0x20000		/* 128KB sect size Intel Flash */

#ifdef CONFIG_CS0_BOOT
#define PHYS_FLASH_1		0x02000000	/* CS0 Base address 	 */
#endif
#ifdef CONFIG_CS3_BOOT
#define PHYS_FLASH_1		0x00000000	/* Need to update CHECK  */
#endif
#define CFG_FLASH_BASE		PHYS_FLASH_1 	/* Flash Base for U-Boot */
#define CONFIG_ENV_OVERWRITE			/* allow env overwrie 	 */
#define PHYS_FLASH_SIZE		0x1000000	/* Flash size 16MB 	 */
#define CFG_MAX_FLASH_SECT	256		/* max sectors on flash  */
						/* Intel 28F128P30T has  */
						/* 131 sectors, 256      */
						/* is used for backwards */
						/* compatibility with    */
						/* AMD AMLV256U on early */
						/* boards.               */
#if(0)
#define CFG_MAX_FLASH_SECT	(PHYS_FLASH_SIZE/CFG_FLASH_SECT_SZ)
#endif
#define CFG_FLASH_ERASE_TOUT	(20*CFG_HZ)	/* Timeout for Flash Erase */
#define CFG_FLASH_WRITE_TOUT	(20*CFG_HZ)	/* Timeout for Flash Write */
/*============================================================================*/

/*
 * Memory Info 
 */
/*============================================================================*/
#define CFG_MALLOC_LEN		(0x20000 + 128*1024)  /* malloc () len */
#define CFG_GBL_DATA_SIZE	128		/* reserved for initial data */
#define CFG_MEMTEST_START	0x82000000	/* memtest start address  */
#define CFG_MEMTEST_END		0x90000000	/* 16MB RAM test   	  */
#define CONFIG_NR_DRAM_BANKS	1		/* we have 1 bank of DRAM */
#define PHYS_SDRAM_1		0x80000000	/* DDR Start 		  */
#define PHYS_SDRAM_1_SIZE	0x10000000	/* DDR size 256MB 	  */
#define CONFIG_STACKSIZE	(256*1024)	/* regular stack	  */
/*============================================================================*/

/*
 * Serial Driver info
 */
/*============================================================================*/
#define CFG_NS16550			/* Include NS16550 as serial driver */
#define CFG_NS16550_SERIAL
#define CFG_NS16550_REG_SIZE 	4		/* NS16550 register size */
#define CFG_NS16550_COM1 	0X01C20000	/* Base address of UART0  */
#define CFG_NS16550_CLK 	24000000	/* Input clock to NS16550 */
#define CONFIG_CONS_INDEX	1		/* use UART0 for console  */
#define CONFIG_BAUDRATE		115200		/* Default baud rate      */
#define CFG_BAUDRATE_TABLE	{ 9600, 19200, 38400, 57600, 115200 }
/*============================================================================*/

/* U-Boot Configurations */
/*============================================================================*/
/*
 * If we are developing, we might want to start armboot from ram
 * so we MUST NOT initialize critical regs like mem-timing ...
 */
/*#undef CONFIG_INIT_CRITICAL             undef for developing */

#undef 	CONFIG_USE_IRQ				/* we don't need IRQ/FIQ */
#define CONFIG_MISC_INIT_R
#define CONFIG_BOOTDELAY	  4     	/* Boot delay before OS boot*/
#define CONFIG_BOOTFILE		"uImage"	/* file to load */
#define CFG_LONGHELP				/* undef to save memory     */
#define CFG_PROMPT	"DM6467 HD EVM # "	/* Monitor Command Prompt   */
#define CFG_CBSIZE	1024			/* Console I/O Buffer Size  */
#define CFG_PBSIZE	(CFG_CBSIZE+sizeof(CFG_PROMPT)+16) /* Print buffer sz */
#define CFG_MAXARGS	16		/* max number of command args   */
#define CFG_BARGSIZE	CFG_CBSIZE	/* Boot Argument Buffer Size    */
#undef	CFG_CLKS_IN_HZ			/* Clock info are in HZ */
#define CFG_LOAD_ADDR	0x80700000	/* default load address of Linux */

/*
 *  I2C Configuration 
 */
#define CONFIG_HARD_I2C
#define CFG_I2C_SPEED 100000
#define CFG_I2C_SLAVE 10
#define CONFIG_DRIVER_DAVINCI_I2C

/* macro to read the 32 bit timer Timer 2 */
#define READ_TIMER (0xFFFFFFFF - (*(volatile ulong *)(CFG_TIMERBASE + 0x14)))

/* Linux Information */

#define LINUX_BOOT_PARAM_ADDR	0x80000100	/* Set the Boot location at the
						 * end of DDR
						 */
#define CONFIG_CMDLINE_TAG	  1	/* enable passing of ATAGs  */
#define CONFIG_SETUP_MEMORY_TAGS  1
#define CONFIG_BOOTARGS		"console=ttyS0,115200n8 noinitrd rw root=/dev/nfs nfsroot=192.168.1.101:/opt/montavista/pro/devkit/arm/v5t_le/target,nolock mem=120M"

#define CONFIG_BOOTCOMMAND	"dhcp;setenv addip setenv bootargs \$(bootargs) ip=\$(ipaddr):\$(serverip):\$(gatewayip):\$(netmask):\$(hostname)::off eth=\$(ethaddr);run addip;bootm 0x80700000"

/*============================================================================*/

/*
 * Network & Ethernet Configuration
 */
/*============================================================================*/
#define CONFIG_DRIVER_TI_EMAC

#define CONFIG_BOOTP_MASK	(CONFIG_BOOTP_DEFAULT | CONFIG_BOOTP_DNS | CONFIG_BOOTP_DNS2 | CONFIG_BOOTP_SEND_HOSTNAME)
#define CONFIG_NET_RETRY_COUNT  10
/*============================================================================*/

/*============================================================================*/

/* NAND Flash stuff */
/*============================================================================*/
#ifdef CFG_ENV_IS_IN_NAND
#define CONFIG_COMMANDS		(CONFIG_CMD_DFL | CFG_CMD_PING | CFG_CMD_DHCP | CFG_CMD_NAND | CFG_CMD_NFS)
#define CONFIG_SKIP_LOWLEVEL_INIT       /* needed for booting from NAND as UBL
					 * bootloads u-boot.  The low level init
					 * is configured by the UBL.
					 */
#define CFG_NAND_BASE           0x42000000

#define CFG_MAX_NAND_DEVICE     1	/* Max number of NAND devices */
#define SECTORSIZE              2048

#define ADDR_COLUMN             1
#define ADDR_PAGE               2
#define ADDR_COLUMN_PAGE        3

#define NAND_MAX_FLOORS         1
#define NAND_MAX_CHIPS          1
#define CFG_ENV_OFFSET	        0x80000 /* environment starts here  */
#else
#define CONFIG_COMMANDS		(CONFIG_CMD_DFL | CFG_CMD_PING | CFG_CMD_DHCP)
#endif

/* this must be included AFTER the definition of CONFIG_COMMANDS (if any) */
#include <cmd_confdefs.h>

/* KGDB support */
/*============================================================================*/
#if (CONFIG_COMMANDS & CFG_CMD_KGDB)
#define CONFIG_KGDB_BAUDRATE	115200	/* speed to run kgdb serial port */
#define CONFIG_KGDB_SER_INDEX	1	/* which serial port to use */
#endif
#endif /* __CONFIG_H */
