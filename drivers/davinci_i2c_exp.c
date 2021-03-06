/*
 * Basic I2C functions
 *
 * Copyright (c) 2004 Texas Instruments
 *
 * This package is free software;  you can redistribute it and/or
 * modify it under the terms of the license found in the file
 * named COPYING that should have accompanied this file.
 *
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 * Author: Jian Zhang jzhang@ti.com, Texas Instruments
 *
 * Copyright (c) 2003 Wolfgang Denk, wd@denx.de
 * Rewritten to fit into the current U-Boot framework
 *
 * Adapted for DaVinci I2C, swami.iyer@ti.com
 *
 */

#include <common.h>

#ifdef CONFIG_DRIVER_DAVINCI_I2C

#include "davinci_i2c.h"
#include <i2c.h>
#include <asm/io.h>

#define inw(a) __raw_readw(a)
#define outw(a,v) __raw_writew(a,v)

static void wait_for_bb (void);
static u16 wait_for_pin (void);
static u16 wait_for_read (void);
void flush_fifo(void);

void i2c_init (int speed, int slaveadd)
{
	u16 scl;

	if (inw (I2C_CON) & I2C_CON_EN) {
		outw (0, I2C_CON);
		udelay (50000);
	}

#ifdef CFG_DAVINCI_HD
	outw (12, I2C_PSC);
	outw (53, I2C_SCLL);
	outw (53, I2C_SCLH);
#elif defined (CFG_DAVINCI)
	outw (26, I2C_PSC);
	outw (20, I2C_SCLL);
	outw (20, I2C_SCLH);
#endif
	/* own address */
	outw (slaveadd, I2C_OA);
	outw (0, I2C_CNT);
	/* have to enable intrrupts or DaVinci i2c module doesn't work */
	outw (I2C_IE_SCD_IE | I2C_IE_XRDY_IE | I2C_IE_RRDY_IE | /*I2C_IE_ARDY_IE |*/
	      I2C_IE_NACK_IE | I2C_IE_AL_IE, I2C_IE);
	outw (I2C_CON_EN, I2C_CON);
	udelay (1000);

}

static int i2c_read_byte (u8 devaddr, u8 regoffset, u8 * value)
{
	int i2c_error = 0;
	u16 status;

	/* wait until bus not busy */
	wait_for_bb ();

	/* one byte only */
	outw (1, I2C_CNT);
	/* set slave address */
	outw (devaddr, I2C_SA);
	/* no stop bit needed here */
	outw (I2C_CON_EN | I2C_CON_MST | I2C_CON_STT | I2C_CON_TRX, I2C_CON);

	status = wait_for_pin ();


	if (!i2c_error) {
		/* free bus, otherwise we can't use a combined transction */
		outw (0, I2C_CON);

		wait_for_bb ();
		/* set slave address */
		outw (devaddr, I2C_SA);
		/* read one byte from slave */
		outw (1, I2C_CNT);
		/* need stop bit here */
		outw (I2C_CON_EN | I2C_CON_MST | I2C_CON_STT | I2C_CON_STP,
		      I2C_CON);

		status = wait_for_pin ();
		status = wait_for_read ();
		if (status & I2C_STAT_RRDY) {
			*value = inw (I2C_DRR);
			udelay (50000);
		} else {
			i2c_error = 1;
		}

		if (!i2c_error) {
			outw (I2C_CON_EN, I2C_CON);
		}
	}
	flush_fifo();
	outw (0xFFFF, I2C_STAT);
	outw (0, I2C_CNT);
	return i2c_error;
}

static int i2c_write_byte (u8 devaddr, u8 regoffset, u8 value)
{
	int i2c_error = 0;
	u16 status, stat;
	u16 temp;

	/* wait until bus not busy */
	wait_for_bb ();

	/* two bytes */
	outw (2, I2C_CNT);
	/* set slave address */
	outw (devaddr, I2C_SA);
	/* stop bit needed here */
	outw (I2C_CON_EN | I2C_CON_MST | I2C_CON_STT | I2C_CON_TRX |
	      I2C_CON_STP, I2C_CON);

	/* wait until state change */
	status = wait_for_pin ();

	if (status & I2C_STAT_XRDY) {
		/* send out two bytes */
		outw (value, I2C_DXR);
		/* must have enough delay to allow BB bit to go low */
		udelay (50000);
		if (inw (I2C_STAT) & I2C_STAT_NACK) {
			i2c_error = 1;
		}
	} else {
		i2c_error = 1;
	}

	if (!i2c_error) {
		outw (I2C_CON_EN, I2C_CON);
		do {
			temp = inw(I2C_STAT) && I2C_STAT_SCD;
		} while (!temp);	
	}
	flush_fifo();
	outw (0xFFFF, I2C_STAT);
	outw (0, I2C_CNT);
	return i2c_error;
}

void flush_fifo(void)
{	u16 stat;

	/* note: if you try and read data when its not there or ready
	 * you get a bus error
	 */
	while(1){
		stat = inw(I2C_STAT);
		if(stat == I2C_STAT_RRDY){
			inw(I2C_DRR);
			outw(I2C_STAT_RRDY,I2C_STAT);
			udelay(1000);
		}else
			break;
	}
}

int i2c_probe (uchar chip)
{
	int res = 1; /* default = fail */

	if (chip == inw (I2C_OA)) {
		return res;
	}

	/* wait until bus not busy */
	wait_for_bb ();

	/* try to read one byte */
	outw (1, I2C_CNT);
	/* set slave address */
	outw (chip, I2C_SA);
	/* stop bit needed here */
	outw (I2C_CON_EN | I2C_CON_MST | I2C_CON_STT | I2C_CON_STP, I2C_CON);
	/* enough delay for the NACK bit set */
	udelay (50000);

	if (!(inw (I2C_STAT) & I2C_STAT_NACK)) {
		res = 0;      /* success case */
		flush_fifo();
		outw(0xFFFF, I2C_STAT);
	} else {
		outw(0xFFFF, I2C_STAT);	 /* failue, clear sources*/
		outw (inw (I2C_CON) | I2C_CON_STP, I2C_CON); /* finish up xfer */
		udelay(20000);
		wait_for_bb ();
	}
	flush_fifo();
	outw (0, I2C_CNT); /* don't allow any more data in...we don't want it.*/
	outw(0xFFFF, I2C_STAT);
	return res;
}

int i2c_read (uchar chip, uint addr, int alen, uchar * buffer, int len)
{
	int i;

	if (alen > 1) {
		printf ("I2C read: addr len %d not supported\n", alen);
		return 1;
	}

	if (addr + len > 256) {
		printf ("I2C read: address out of range\n");
		return 1;
	}

	for (i = 0; i < len; i++) {
		if (i2c_read_byte (chip, addr + i, &buffer[i])) {
			printf ("I2C read: I/O error\n");
			i2c_init (CFG_I2C_SPEED, CFG_I2C_SLAVE);
			return 1;
		}
	}

	return 0;
}

int i2c_write (uchar chip, uint addr, int alen, uchar * buffer, int len)
{
	int i;

	if (alen > 1) {
		printf ("I2C read: addr len %d not supported\n", alen);
		return 1;
	}

	if (addr + len > 256) {
		printf ("I2C read: address out of range\n");
		return 1;
	}

	for (i = 0; i < len; i++) {
		if (i2c_write_byte (chip, addr + i, buffer[i])) {
			printf ("I2C write: I/O error\n");
			i2c_init (CFG_I2C_SPEED, CFG_I2C_SLAVE);
			return 1;
		}
	}

	return 0;
}

static void wait_for_bb (void)
{
	int timeout = 0;
	u16 stat;

	outw(0xFFFF, I2C_STAT);	 /* clear current interrupts...*/
	while ((stat = inw (I2C_STAT) & I2C_STAT_BB) && timeout--) {
		outw (stat, I2C_STAT);
		udelay (50000);
	}

	if (timeout <= 0) {
		/*printf ("timed out in wait_for_bb: I2C_STAT=%x\n",
			inw (I2C_STAT));*/
	}
	outw(0xFFFF, I2C_STAT);	 /* clear delayed stuff*/
}

static u16 wait_for_pin (void)
{
	u16 status;
	int timeout = 10;

	do {
		udelay (1000);
		//udelay (50000);
		status = inw (I2C_STAT);
	} while (  !(status &
		   (I2C_STAT_ROVR | I2C_STAT_XUDF | I2C_STAT_XRDY |
		    I2C_STAT_RRDY | I2C_STAT_ARDY | I2C_STAT_NACK |
		    I2C_STAT_AL   | I2C_STAT_SCD)) && timeout--);

	if (timeout <= 0) {
		printf ("timed out in wait_for_pin: I2C_STAT=%x\n",
			inw (I2C_STAT));
			outw(0xFFFF, I2C_STAT);
	}
	return status;
}

static u16 wait_for_read (void)
{
	u16 status;
	int timeout = 10;

	do {
		udelay (1000);
		status = inw (I2C_STAT);
	} while (  !(status & I2C_STAT_RRDY) && timeout--);

	if (timeout <= 0) {
		printf ("timed out in wait_for_read: I2C_STAT=%x\n",
			inw (I2C_STAT));
			outw(0xFFFF, I2C_STAT);
	}
	return status;
}
#endif /* CONFIG_DRIVER_DAVINCI_I2C */
