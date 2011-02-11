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
 ver. 1.0: Oct 2005, Swaminathan S
 *
 */
#include <common.h>
#include <i2c.h>
#include <asm/io.h>

#define inw(a) __raw_readw(a)
#define outw(a,v) __raw_writew(a,v)


#define PLL1_PLLM    *(volatile unsigned int *)0x01c40910
#define PLL2_PLLM    *(volatile unsigned int *)0x01c40D10
#define PLL2_DIV2    *(volatile unsigned char *)0x01c40D1C
#define PLL2_PREDIV  *(volatile unsigned int *)0x01C40D14
#define PLL1_PLLDIV3 *(volatile unsigned int *)0x01C40920
#define PLL1_POSTDIV *(volatile unsigned int *)0x01C40928
#define PLL1_PLLDIV4 *(volatile unsigned int *)0x01C40960
#define SYSTEM_MISC  *(volatile unsigned int *)0x01C40038
#define MACH_DM350_EVM	1381

void davinci_psc_all_enable(void);
short MSP430_getReg( short reg, unsigned short *regval );
unsigned int UARTSendInt(unsigned int value);

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

        /* arch number of DaVinci DVDP-Board */
        gd->bd->bi_arch_number = MACH_DM350_EVM;

      	/* adress of boot parameters */
       	gd->bd->bi_boot_params = LINUX_BOOT_PARAM_ADDR;
        /* Configure MUX settings */

      /* Power on required peripherals 
    	davinci_psc_all_enable(); */
#if 0
      	/* this speeds up your boot a quite a bit.  However to make it
 	 *  work, you need make sure your kernel startup flush bug is fixed.
	 *  ... rkw ...
	 */
	icache_enable ();

#endif
      	inittimer ();

      	return 0;
}

/* PSC Domains */

#define LPSC_VPSSMSTR       0       // VPSS Master LPSC
#define LPSC_VPSSSLV        1       // VPSS Slave LPSC
#define LPSC_TPCC           2       // TPCC LPSC
#define LPSC_TPTC0          3       // TPTC0 LPSC
#define LPSC_TPTC1          4       // TPTC1 LPSC
#define PAL_SYS_CLK_MODULE_SPI1 6 /**<SPI1 LPSC Module No*/
#define PAL_SYS_CLK_MODULE_MMCSD1 7 /**<MMCSD1 LPSC Module No*/
#define LPSC_USB            9       // USB LPSC
#define PAL_SYS_CLK_MODULE_PWM3 10  /**<PWM3 LPSC Module No*/
#define PAL_SYS_CLK_MODULE_SPI2 11  /**<SPI2 LPSC Module No*/
#define PAL_SYS_CLK_MODULE_RTO 12   /**<TIMER2 LPSC Module No*/
#define LPSC_DDR_EMIF       13      // DDR_EMIF LPSC
#define LPSC_AEMIF          14      // AEMIF LPSC
#define LPSC_MMC_SD         15      // MMC_SD LPSC
#define LPSC_MEMSTICK       16      // MEMSTICK LPSC
#define PAL_SYS_CLK_MODULE_ASP 17   /**<AEMIF LPSC Module No*/
#define LPSC_I2C            18      // I2C LPSC
#define LPSC_UART0          19      // UART0 LPSC
#define LPSC_UART1          20      // UART1 LPSC
#define LPSC_UART2          21      // UART2 LPSC
#define LPSC_SPI            22      // SPI LPSC
#define LPSC_PWM0           23      // PWM0 LPSC
#define LPSC_PWM1           24      // PWM1 LPSC
#define LPSC_PWM2           25      // PWM2 LPSC
#define LPSC_GPIO           26      // GPIO LPSC
#define LPSC_TIMER0         27      // TIMER0 LPSC
#define LPSC_TIMER1         28      // TIMER1 LPSC
#define LPSC_TIMER2         29      // TIMER2 LPSC
#define LPSC_SYSTEM_SUBSYS  30      // SYSTEM SUBSYSTEM LPSC
#define LPSC_ARM            31      // ARM LPSC
#define PAL_SYS_CLK_MODULE_VPSS_DAC 40  /**<VPSS DAC LPSC Module No*/

#define EPCPR               *( unsigned int* )( 0x01C41070 )
#define PTCMD               *( unsigned int* )( 0x01C41120 )
#define PTSTAT              *( unsigned int* )( 0x01C41128 )
#define PDSTAT              *( unsigned int* )( 0x01C41200 )
#define PDSTAT1             *( unsigned int* )( 0x01C41204 )
#define PDCTL               *( unsigned int* )( 0x01C41300 )
#define PDCTL1              *( unsigned int* )( 0x01C41304 )
#define VBPR                *( unsigned int* )( 0x20000020 )

/**************************************
 Routine: board_setup_psc_on
 Description:  Enable a PSC domain
**************************************/
void board_setup_psc_on( unsigned int domain, unsigned int id )
{
        volatile unsigned int* mdstat = ( unsigned int* )( 0x01C41800 + 4 * id );
	      volatile unsigned int* mdctl  = ( unsigned int* )( 0x01C41A00 + 4 * id );

	      *mdctl |= 0x00000003; // Set PowerDomain to turn on

    	  if ( ( PDSTAT & 0x00000001 ) == 0 )
    	  {
                PDCTL1 |= 0x1;
        	      PTCMD = ( 1 << domain );
        	      while ( ( ( ( EPCPR >> domain ) & 1 ) == 0 ) );

        	      PDCTL1 |= 0x100 ;
        	      while( ! ( ( ( PTSTAT >> domain ) & 1 ) == 0 ) );
    	  }
    	  else
    	  {
        	      PTCMD = ( 1<<domain );
        	      while( ! ( ( ( PTSTAT >> domain ) & 1 ) == 0 ) );
    	  }

    	  while( ! ( ( *mdstat & 0x0000001F ) == 0x3 ) );
}

/**************************************
 Routine: davinci_psc_all_enable
 Description:  Enable all PSC domains
**************************************/
void davinci_psc_all_enable(void)
{
#define PSC_ADDR            0x01C41000
#define PTCMD               (PSC_ADDR+0x120)
#define PTSTAT              (PSC_ADDR+0x128)

        unsigned int alwaysOnPdNum = 0, dspPdNum = 1, i;

    	  /* This function turns on all clocks in the ALWAYSON and DSP Power
	       * Domains.  Note this function assumes that the Power Domains are
	       * already on.
	       */
#if 0
    	  /* Write ENABLE (0x3) to all 41 MDCTL[i].NEXT bit fields. */
    	  for( i = 0; i < 41; i++){
                *(volatile unsigned int*) (PSC_ADDR+0xA00+4*i) =
				        *(volatile unsigned int*) (PSC_ADDR+0xA00+4*i) | 0x3;
    	  }

    	  /* For special workaround:  Set MDCTL[i].EMURSTIE to 0x1 for all of the
	       * following Modules.  VPSSSLV, EMAC, EMACCTRL, MDIO, USB, ATA, VLYNQ,
         * HPI, DDREMIF, AEMIF, MMCSD, MEMSTICK, ASP, GPIO, IMCOP.
         */
        /**(unsigned int*) (PSC_ADDR+0xA00+4*1) = *(unsigned int*) (PSC_ADDR+0xA00+4*1) | 0x203;*/
        *(unsigned int*) (PSC_ADDR+0xA00+4*5) = *(unsigned int*) (PSC_ADDR+0xA00+4*5) | 0x203;
        *(unsigned int*) (PSC_ADDR+0xA00+4*6) = *(unsigned int*) (PSC_ADDR+0xA00+4*6) | 0x203;
        *(unsigned int*) (PSC_ADDR+0xA00+4*7) = *(unsigned int*) (PSC_ADDR+0xA00+4*7) | 0x203;
        /**(unsigned int*) (PSC_ADDR+0xA00+4*9) = *(unsigned int*) (PSC_ADDR+0xA00+4*9) | 0x203;
        *(unsigned int*) (PSC_ADDR+0xA00+4*10) = *(unsigned int*) (PSC_ADDR+0xA00+4*10) | 0x203;
        *(unsigned int*) (PSC_ADDR+0xA00+4*11) = *(unsigned int*) (PSC_ADDR+0xA00+4*11) | 0x203;
        *(unsigned int*) (PSC_ADDR+0xA00+4*12) = *(unsigned int*) (PSC_ADDR+0xA00+4*12) | 0x203;*/
        *(unsigned int*) (PSC_ADDR+0xA00+4*13) = *(unsigned int*) (PSC_ADDR+0xA00+4*13) | 0x203;
        *(unsigned int*) (PSC_ADDR+0xA00+4*14) = *(unsigned int*) (PSC_ADDR+0xA00+4*14) | 0x203;
        /**(unsigned int*) (PSC_ADDR+0xA00+4*15) = *(unsigned int*) (PSC_ADDR+0xA00+4*15) | 0x203;
        *(unsigned int*) (PSC_ADDR+0xA00+4*16) = *(unsigned int*) (PSC_ADDR+0xA00+4*16) | 0x203;
        *(unsigned int*) (PSC_ADDR+0xA00+4*17) = *(unsigned int*) (PSC_ADDR+0xA00+4*17) | 0x203;*/
        *(unsigned int*) (PSC_ADDR+0xA00+4*19) = *(unsigned int*) (PSC_ADDR+0xA00+4*19) | 0x203;
        /**(unsigned int*) (PSC_ADDR+0xA00+4*26) = *(unsigned int*) (PSC_ADDR+0xA00+4*26) | 0x203;
        *(unsigned int*) (PSC_ADDR+0xA00+4*40) = *(unsigned int*) (PSC_ADDR+0xA00+4*40) | 0x203;*/
#endif

        /* For special workaround:  Clear MDCTL[i].EMURSTIE to 0x0 for all of the following Modules.
         *  VPSSSLV, EMAC, EMACCTRL, MDIO, USB, ATA, VLYNQ,
         *  HPI, DDREMIF, AEMIF, MMCSD, MEMSTICK, ASP, GPIO, IMCOP.
         */
        /**(unsigned int*) (PSC_ADDR+0xA00+4*1) = *(unsigned int*) (PSC_ADDR+0xA00+4*1) & 0x003;*/
        *(unsigned int*) (PSC_ADDR+0xA00+4*5) = *(unsigned int*) (PSC_ADDR+0xA00+4*5) | 0x003;
        *(unsigned int*) (PSC_ADDR+0xA00+4*6) = *(unsigned int*) (PSC_ADDR+0xA00+4*6) | 0x003;
        *(unsigned int*) (PSC_ADDR+0xA00+4*7) = *(unsigned int*) (PSC_ADDR+0xA00+4*7) | 0x003;
        /**(unsigned int*) (PSC_ADDR+0xA00+4*9) = *(unsigned int*) (PSC_ADDR+0xA00+4*9) & 0x003;
        *(unsigned int*) (PSC_ADDR+0xA00+4*10) = *(unsigned int*) (PSC_ADDR+0xA00+4*10) & 0x003;
        *(unsigned int*) (PSC_ADDR+0xA00+4*11) = *(unsigned int*) (PSC_ADDR+0xA00+4*11) & 0x003;
        *(unsigned int*) (PSC_ADDR+0xA00+4*12) = *(unsigned int*) (PSC_ADDR+0xA00+4*12) & 0x003;*/
        *(unsigned int*) (PSC_ADDR+0xA00+4*13) = *(unsigned int*) (PSC_ADDR+0xA00+4*13) | 0x003;
        *(unsigned int*) (PSC_ADDR+0xA00+4*14) = *(unsigned int*) (PSC_ADDR+0xA00+4*14) | 0x003;
        /**(unsigned int*) (PSC_ADDR+0xA00+4*15) = *(unsigned int*) (PSC_ADDR+0xA00+4*15) & 0x003;
        *(unsigned int*) (PSC_ADDR+0xA00+4*16) = *(unsigned int*) (PSC_ADDR+0xA00+4*16) & 0x003;
        *(unsigned int*) (PSC_ADDR+0xA00+4*17) = *(unsigned int*) (PSC_ADDR+0xA00+4*17) & 0x003;*/
        *(unsigned int*) (PSC_ADDR+0xA00+4*19) = ((*(unsigned int*) (PSC_ADDR+0xA00+4*19))&0xFFFFFFF8) | 0x003;
        *(unsigned int*) (PSC_ADDR+0xA00+4*20) = ((*(unsigned int*) (PSC_ADDR+0xA00+4*20))&0xFFFFFFF8) | 0x003;
        *(unsigned int*) (PSC_ADDR+0xA00+4*21) = ((*(unsigned int*) (PSC_ADDR+0xA00+4*21))&0xFFFFFFF8) | 0x003;
        *(unsigned int*) (PSC_ADDR+0xA00+4*18) = *(unsigned int*) (PSC_ADDR+0xA00+4*18) | 0x003;
        *(unsigned int*) (PSC_ADDR+0xA00+4*28) = *(unsigned int*) (PSC_ADDR+0xA00+4*28) | 0x003;
        /**(unsigned int*) (PSC_ADDR+0xA00+4*26) = *(unsigned int*) (PSC_ADDR+0xA00+4*26) & 0x003;
        *(unsigned int*) (PSC_ADDR+0xA00+4*40) = *(unsigned int*) (PSC_ADDR+0xA00+4*40) & 0x003;*/

    	  /* Set PTCMD.GO0 to 0x1 to initiate the state transtion for Modules in
	       * the ALWAYSON Power Domain
	       */
    	  *(volatile unsigned int*) PTCMD = (1<<alwaysOnPdNum);


    	  /* Wait for PTSTAT.GOSTAT0 to clear to 0x0 */
    	  while(! (((*(volatile unsigned int*) PTSTAT >> alwaysOnPdNum) & 0x00000001) == 0));

	/* Wait for PTSTAT.GOSTAT0 to clear to 0x0 */
          while(! ((*(unsigned int*) (PSC_ADDR+0x800+4*19)& 0x0000001F ) == 0x3));
	/* Wait for PTSTAT.GOSTAT0 to clear to 0x0 */
          while(! ((*(unsigned int*) (PSC_ADDR+0x800+4*20)& 0x0000001F ) == 0x3));
	/* Wait for PTSTAT.GOSTAT0 to clear to 0x0 */
          while(! ((*(unsigned int*) (PSC_ADDR+0x800+4*21)& 0x0000001F ) == 0x3));
        /* Bringup UART out of reset here since NS16650 code that we are using from uBoot
         * will not do it
         */

#define UART0PWREMU_MGMT     0x01c20030
        *(volatile unsigned int*) UART0PWREMU_MGMT |= 0x00008001;


#define UART1PWREMU_MGMT     0x01c20430
        *(volatile unsigned int*) UART1PWREMU_MGMT |= 0x00008001;

#define UART2PWREMU_MGMT     0x01e06030
        *(volatile unsigned int*) UART2PWREMU_MGMT |= 0x00008001;

#define PINMUX3     0x01C4000C
        /* Enable UART1 MUX Lines */
        *(volatile unsigned int *)PINMUX3 |= 0x00600000;
	
				/* Enable UART2 MUX Lines */
        *(volatile unsigned int *)PINMUX3 |= 0x0000AA00;

	/* Set the Bus Priority Register to appropriate value */
	VBPR = 0x20;
}

/******************************
 Routine: misc_init_r
 Description:  Misc. init
******************************/
int misc_init_r (void)
{
	char temp[20], *env=0;
	char rtcdata[10] = { 4, 1, 0, 0, 0, 0, 0, 0, 0, 0};
    int clk = 0;
    unsigned short regval=0 ;

    clk = ((PLL2_PLLM + 1) * 24) / ((PLL2_PREDIV & 0x1F) + 1); 

    printf ("ARM Clock :- %dMHz\n", ( ( ((PLL1_PLLM+1)*24 )/(2*(7+1)*((SYSTEM_MISC & 0x2)?2:1 )))) );
    printf ("DDR Clock :- %dMHz\n", (clk/2));

    if ( !(env=getenv("videostd")) || !strcmp(env,"ntsc") || !strcmp(env, "pal") )
    {
        MSP430_getReg( 0x04, &regval);
        //printf("regval is %x\n",regval);
        setenv ("videostd", ((regval  & 0x10)?"ntsc":"pal"));
    }

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


typedef int     Bool;
#define TRUE        ((Bool) 1)
#define FALSE       ((Bool) 0)


typedef int             Int;
typedef unsigned int    Uns;    /* deprecated type */
typedef char            Char;
typedef char *          String;
typedef void *          Ptr;

/* unsigned quantities */
typedef unsigned int    Uint32;
typedef unsigned short  Uint16;
typedef unsigned char   Uint8;

/* signed quantities */
typedef int             Int32;
typedef short           Int16;
typedef char            Int8;

/* volatile unsigned quantities */
typedef volatile unsigned int       VUint32;
typedef volatile unsigned short     VUint16;
typedef volatile unsigned char      VUint8;

/* volatile signed quantities */
typedef volatile int                VInt32;
typedef volatile short              VInt16;
typedef volatile char               VInt8;

typedef struct _uart_regs
{
    VUint32 RBR;
    VUint32 IER;
    VUint32 IIR;
    VUint32 LCR;
    VUint32 MCR;
    VUint32 LSR;
    VUint32 MSR;
    VUint32 SCR;
    VUint8 DLL;
    VUint8 RSVDO[3];
    VUint8 DLH;
    VUint8 RSVD1[3];
    VUint32 PID1;
    VUint32 PID2;
    VUint32 PWREMU_MGNT;
} uartRegs;

#define THR RBR
#define FCR IIR

#define UART0 ((uartRegs*) 0x01C20000)

#define MAXSTRLEN 256
#define E_PASS    0x00000000u
#define E_FAIL    0x00000001u
#define E_TIMEOUT 0x00000002u



// Send specified number of bytes

Int32 GetStringLen(Uint8* seq)
{
    Int32 i = 0;
    while ((seq[i] != 0) && (i<MAXSTRLEN)){ i++;}
    if (i == MAXSTRLEN)
        return -1;
    else
        return i;
}

Uint32 UARTSendData(Uint8* seq, Bool includeNull)
{
    Uint32 status = 0;
    Int32 i,numBytes;
    Uint32 timerStatus = 0x1000000;

    numBytes = includeNull?(GetStringLen(seq)+1):(GetStringLen(seq));

    for(i=0;i<numBytes;i++) {
        /* Enable Timer one time */
        //TIMER0Start();
        do{
            status = (UART0->LSR)&(0x60);
            //timerStatus = TIMER0Status();
            timerStatus--;
        } while (!status && timerStatus);

        if(timerStatus == 0)
            return E_TIMEOUT;

        // Send byte
        (UART0->THR) = seq[i];
    }
    return E_PASS;
}

Uint32 UARTSendInt(Uint32 value)
{
    char seq[9];
    Uint32 i,shift,temp;

    for( i = 0; i < 8; i++)
    {
        shift = ((7-i)*4);
        temp = ((value>>shift) & (0x0000000F));
        if (temp > 9)
        {
            temp = temp + 7;
        }
        seq[i] = temp + 48;
        seq[i] = temp + 48;
    }
    seq[8] = 0;
    return UARTSendData(seq, FALSE);
}

#define I2C_BASE                0x01C21000
#define	I2C_OA			(I2C_BASE + 0x00)
#define I2C_IE                  (I2C_BASE + 0x04)
#define I2C_STAT                (I2C_BASE + 0x08)
#define I2C_SCLL                (I2C_BASE + 0x0c)
#define I2C_SCLH                (I2C_BASE + 0x10)
#define I2C_CNT                 (I2C_BASE + 0x14)
#define I2C_DRR                 (I2C_BASE + 0x18)
#define I2C_SA                  (I2C_BASE + 0x1c)
#define I2C_DXR                 (I2C_BASE + 0x20)
#define I2C_CON                 (I2C_BASE + 0x24)
#define I2C_IV                  (I2C_BASE + 0x28)
#define I2C_PSC                 (I2C_BASE + 0x30)

#define I2C_CON_EN      (1 << 5)       /* I2C module enable */
#define I2C_CON_STB     (1 << 4)       /* Start byte mode (master mode only) */
#define I2C_CON_MST     (1 << 10)      /* Master/slave mode */
#define I2C_CON_TRX     (1 << 9)       /* Transmitter/receiver mode (master mode only) */
#define I2C_CON_XA      (1 << 8)       /* Expand address */
#define I2C_CON_STP     (1 << 11)      /* Stop condition (master mode only) */
#define I2C_CON_STT     (1 << 13)      /* Start condition (master mode only) */

#define I2C_STAT_BB     (1 << 12)       /* Bus busy */
#define I2C_STAT_ROVR   (1 << 11)       /* Receive overrun */
#define I2C_STAT_XUDF   (1 << 10)       /* Transmit underflow */
#define I2C_STAT_AAS    (1 << 9)        /* Address as slave */
#define I2C_STAT_SCD    (1 << 5)        /* Stop condition detect */
#define I2C_STAT_XRDY   (1 << 4)        /* Transmit data ready */
#define I2C_STAT_RRDY   (1 << 3)        /* Receive data ready */
#define I2C_STAT_ARDY   (1 << 2)        /* Register access ready */
#define I2C_STAT_NACK   (1 << 1)        /* No acknowledgment interrupt enable */
#define I2C_STAT_AL     (1 << 0)        /* Arbitration lost interrupt enable */

static Int16 I2C_init(void );
static Int16 I2C_close(void );
static Int16 I2C_reset( void);
static Int16 I2C_write( Uint16 i2c_addr, Uint8* data, Uint16 len );
static Int16 I2C_read( Uint16 i2c_addr, Uint8* data, Uint16 len );
Int32 i2c_timeout = 0x10000;

Int16 MSP430_getReg( Int16 reg, Uint16 *regval )
{
    volatile Int16 retcode;
    Uint8 msg[2];

    I2C_reset();
    udelay(10000);
    /* Send Msg */
    msg[0] = (Uint8)(reg & 0xff);
    if ( retcode = I2C_write( 0x25, msg, 1) )
    {
        return retcode;
    }

    if ( retcode = I2C_read(  0x25, msg, 1 ) )
    {
        return retcode;
    }

    *regval = msg[0];

    /* Wait 1 msec */
    udelay( 1000 );

    return 0;
}

static Int16 I2C_init( )
{
    outw(0, I2C_CON);                // Reset I2C
    outw(26,I2C_PSC);               // Config prescaler for 27MHz
    outw(20,I2C_SCLL);              // Config clk LOW for 20kHz
    outw(20,I2C_SCLH);               // Config clk HIGH for 20kHz
    outw(inw(I2C_CON) | I2C_CON_EN,I2C_CON);        // Release I2C from reset
    return 0;
}

/* ------------------------------------------------------------------------ *
 *                                                                          *
 *  _I2C_close( )                                                           *
 *                                                                          *
 * ------------------------------------------------------------------------ */
static Int16 I2C_close( )
{
        outw(0,I2C_CON);                      // Reset I2C
        return 0;
}

/* ------------------------------------------------------------------------ *
 *                                                                          *
 *  _I2C_reset( )                                                           *
 *                                                                          *
 * ------------------------------------------------------------------------ */
static Int16 I2C_reset( )
{
    I2C_close( );
    I2C_init( );
    return 0;
}

static Int16 I2C_write( Uint16 i2c_addr, Uint8* data, Uint16 len )
{
    Int32 timeout, i, status;

        outw(len, I2C_CNT);                    // Set length
        outw(i2c_addr, I2C_SA);               // Set I2C slave address
        outw(0x2000               // Set for Master Write
             | 0x0200
             | 0x0400
             | I2C_CON_EN
             | 0x4000, I2C_CON );

        udelay( 10 );                        // Short delay

        for ( i = 0 ; i < len ; i++ )
        {
            outw( data[i],I2C_DXR);;            // Write

            timeout = i2c_timeout;
            do
            {
                if ( timeout-- < 0  )
                {
                    I2C_reset( );
                    return -1;
                }
            } while ( ( inw(I2C_STAT) & I2C_STAT_XRDY ) == 0 );// Wait for Tx Ready
        }

        outw( inw(I2C_CON) | 0x0800, I2C_CON);             // Generate STOP

        return 0;

}
static Int16 I2C_read( Uint16 i2c_addr, Uint8* data, Uint16 len )
{
    Int32 timeout, i, status;

    outw( len, I2C_CNT);                    // Set length
    outw( i2c_addr, I2C_SA);               // Set I2C slave address
    outw( 0x2000               // Set for Master Read
          | 0x0400
          | I2C_CON_EN
          | 0x4000,I2C_CON);

    udelay( 10 );                        // Short delay

    for ( i = 0 ; i < len ; i++ )
    {
        timeout = i2c_timeout;

        /* Wait for Rx Ready */
        do
        {
            if ( timeout-- < 0 )
            {
                I2C_reset( );
                return -1;
            }
        } while ( ( inw(I2C_STAT) & I2C_STAT_RRDY ) == 0 );// Wait for Rx Ready

        data[i] = inw(I2C_DRR);            // Read
    }

      //I2C_ICMDR |= ICMDR_STP;             // Generate STOP
    return 0;
}

