/*
 * NAND driver for TI DaVinci based boards.
 *
 * Copyright (C) 2007 Sergey Kubushyn <ksi@koi8.net>
 *
 * Based on Linux DaVinci NAND driver by TI. Original copyright follows:
 */

/*
 *
 * linux/drivers/mtd/nand/nand_dm355.c
 *
 * NAND Flash Driver
 *
 * Copyright (C) 2006 Texas Instruments.
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
 *
 *  Overview:
 *   This is a device driver for the NAND flash device found on the
 *   DaVinci board which utilizes the Samsung k9k2g08 part.
 *
 Modifications:
 ver. 1.0: Feb 2005, Vinod/Sudhakar
 -
 *
 */

#include <common.h>

#if (CONFIG_COMMANDS & CFG_CMD_NAND)
#if !defined(CFG_NAND_LEGACY)

#include <asm/arch/types.h>
#include <nand.h>
#include <asm/arch/nand_defs.h>
#include <asm/arch/emif_defs.h>
#include <asm/io.h>

#define NAND_Ecc_P1e            (1 << 0)
#define NAND_Ecc_P2e            (1 << 1)
#define NAND_Ecc_P4e            (1 << 2)
#define NAND_Ecc_P8e            (1 << 3)
#define NAND_Ecc_P16e           (1 << 4)
#define NAND_Ecc_P32e           (1 << 5)
#define NAND_Ecc_P64e           (1 << 6)
#define NAND_Ecc_P128e          (1 << 7)
#define NAND_Ecc_P256e          (1 << 8)
#define NAND_Ecc_P512e          (1 << 9)
#define NAND_Ecc_P1024e         (1 << 10)
#define NAND_Ecc_P2048e         (1 << 11)

#define NAND_Ecc_P1o            (1 << 16)
#define NAND_Ecc_P2o            (1 << 17)
#define NAND_Ecc_P4o            (1 << 18)
#define NAND_Ecc_P8o            (1 << 19)
#define NAND_Ecc_P16o           (1 << 20)
#define NAND_Ecc_P32o           (1 << 21)
#define NAND_Ecc_P64o           (1 << 22)
#define NAND_Ecc_P128o          (1 << 23)
#define NAND_Ecc_P256o          (1 << 24)
#define NAND_Ecc_P512o          (1 << 25)
#define NAND_Ecc_P1024o         (1 << 26)
#define NAND_Ecc_P2048o         (1 << 27)

#define TF(value)       (value ? 1 : 0)

#define P2048e(a)       (TF(a & NAND_Ecc_P2048e)        << 0 )
#define P2048o(a)       (TF(a & NAND_Ecc_P2048o)        << 1 )
#define P1e(a)          (TF(a & NAND_Ecc_P1e)           << 2 )
#define P1o(a)          (TF(a & NAND_Ecc_P1o)           << 3 )
#define P2e(a)          (TF(a & NAND_Ecc_P2e)           << 4 )
#define P2o(a)          (TF(a & NAND_Ecc_P2o)           << 5 )
#define P4e(a)          (TF(a & NAND_Ecc_P4e)           << 6 )
#define P4o(a)          (TF(a & NAND_Ecc_P4o)           << 7 )

#define P8e(a)          (TF(a & NAND_Ecc_P8e)           << 0 )
#define P8o(a)          (TF(a & NAND_Ecc_P8o)           << 1 )
#define P16e(a)         (TF(a & NAND_Ecc_P16e)          << 2 )
#define P16o(a)         (TF(a & NAND_Ecc_P16o)          << 3 )
#define P32e(a)         (TF(a & NAND_Ecc_P32e)          << 4 )
#define P32o(a)         (TF(a & NAND_Ecc_P32o)          << 5 )
#define P64e(a)         (TF(a & NAND_Ecc_P64e)          << 6 )
#define P64o(a)         (TF(a & NAND_Ecc_P64o)          << 7 )

#define P128e(a)        (TF(a & NAND_Ecc_P128e)         << 0 )
#define P128o(a)        (TF(a & NAND_Ecc_P128o)         << 1 )
#define P256e(a)        (TF(a & NAND_Ecc_P256e)         << 2 )
#define P256o(a)        (TF(a & NAND_Ecc_P256o)         << 3 )
#define P512e(a)        (TF(a & NAND_Ecc_P512e)         << 4 )
#define P512o(a)        (TF(a & NAND_Ecc_P512o)         << 5 )
#define P1024e(a)       (TF(a & NAND_Ecc_P1024e)        << 6 )
#define P1024o(a)       (TF(a & NAND_Ecc_P1024o)        << 7 )

#define P8e_s(a)        (TF(a & NAND_Ecc_P8e)           << 0 )
#define P8o_s(a)        (TF(a & NAND_Ecc_P8o)           << 1 )
#define P16e_s(a)       (TF(a & NAND_Ecc_P16e)          << 2 )
#define P16o_s(a)       (TF(a & NAND_Ecc_P16o)          << 3 )
#define P1e_s(a)        (TF(a & NAND_Ecc_P1e)           << 4 )
#define P1o_s(a)        (TF(a & NAND_Ecc_P1o)           << 5 )
#define P2e_s(a)        (TF(a & NAND_Ecc_P2e)           << 6 )
#define P2o_s(a)        (TF(a & NAND_Ecc_P2o)           << 7 )

#define P4e_s(a)        (TF(a & NAND_Ecc_P4e)           << 0 )
#define P4o_s(a)        (TF(a & NAND_Ecc_P4o)           << 1 )

#define CSL_EMIF_1_REGS    0x01E10000

#define NAND4BITECCLOAD		(0x01E10000 +0xBC)
#define NAND4BITECC1        (0x01E10000 +0xC0)
#define NAND4BITECC2        (0x01E10000 +0xC4) 
#define NAND4BITECC3        (0x01E10000 +0xC8)
#define NAND4BITECC4        (0x01E10000 +0xCC)

#define NANDERRADD1		    (0x01E10000 +0xD0)
#define NANDERRADD2		    (0x01E10000 +0xD4)
#define NANDERRVAL1		    (0x01E10000 +0xD8)
#define NANDERRVAL2		    (0x01E10000 +0xDC)

/* Definitions for 4-bit hardware ECC */
#define NAND_4BITECC_MASK		0x03FF03FF
#define EMIF_NANDFSR_ECC_STATE_MASK  	0x00000F00
#define ECC_STATE_NO_ERR		0x0
#define ECC_STATE_TOO_MANY_ERRS		0x1
#define ECC_STATE_ERR_CORR_COMP_P	0x2
#define ECC_STATE_ERR_CORR_COMP_N	0x3
#define ECC_MAX_CORRECTABLE_ERRORS	0x4
extern struct nand_chip nand_dev_desc[CFG_MAX_NAND_DEVICE];

/* Function Prototypes */
static u32 nand_dm355_4bit_readecc(struct mtd_info *mtd, unsigned int ecc[4]);

/* This buffer holds the ecc values calculated on every 512 bytes of data during the read */
static u_char davinci_ecc_buf[NAND_MAX_OOBSIZE];

static void nand_dm350evm_hwcontrol(struct mtd_info *mtd, int cmd)
{
	struct		nand_chip *this = mtd->priv;
	u_int32_t	IO_ADDR_W = (u_int32_t)this->IO_ADDR_W;

	IO_ADDR_W &= ~(MASK_ALE|MASK_CLE);

	switch (cmd) {
		case NAND_CTL_SETCLE:
			IO_ADDR_W |= MASK_CLE;
			break;
		case NAND_CTL_SETALE:
			IO_ADDR_W |= MASK_ALE;
			break;
	}

	this->IO_ADDR_W = (void *)IO_ADDR_W;
}

static int nand_dm350evm_dev_ready(struct mtd_info *mtd)
{
	emifregs	emif_addr = (emifregs)CSL_EMIF_1_REGS;

	return(emif_addr->NANDFSR) /*& 0x1)*/;
}

static int nand_dm350evm_waitfunc(struct mtd_info *mtd, struct nand_chip *this, int state)
{
	while(!nand_dm350evm_dev_ready(mtd)) {;}
	*NAND_CE0CLE = NAND_STATUS;
	return(*NAND_CE0DATA);
}

void nand_debug_dump(uint8_t *buf, size_t len)
{
	int i;
	printk("nand_debug_dump: len = %d\n", len);
	for (i = 0; i < len; i++)
	{
		if (i % 8 == 0)
			printk("\n%p:", &buf[i]);
		printk(" %02x", buf[i]);
	}
	printk("\n");
}

/**
 * nand_dm355_write_buf - [DEFAULT] write buffer to chip
 * @mtd:	MTD device structure
 * @buf:	data buffer
 * @len:	number of bytes to write
 *
 * NOTE:  This function has been modified for TI DaVinci devices.  If the
 *        device it a DaVinci device then the ecc+oob data is placed in a
 *	  temporary buffer such that the layout is data+oob.
 */
static void nand_dm355_write_buf(struct mtd_info *mtd, const u_char *buf, int len, int layout_idx, int index)
{
	int i;
	int offset = 0;
	struct nand_chip *this = mtd->priv;
	int flush = 0;

#ifdef CFG_DM355_NAND_STD_LAYOUT
	if (this->eccmode == NAND_ECC_SOFT)
		for (i=0; i<len; i++)
			writeb(buf[i], this->IO_ADDR_W);
	else {
		switch (this->layout[layout_idx].type) {
			case ITEM_TYPE_DATA:
				/* Write 512 bytes data chunk */
				for (i = 0; i < len; i++)
					writeb(buf[i], this->IO_ADDR_W);
				break;
			case ITEM_TYPE_ECC:
				/* Save the ECC in the temporary buffer */
				offset = (16 * index) + 6;
				memcpy(&davinci_ecc_buf[offset], buf, len);
				if (index == this->eccsteps - 1)
					flush = 1;
				break;
			case ITEM_TYPE_OOB:
				/* Save the OOB in the temporary buffer */
				offset = (16 * index);
				memcpy(&davinci_ecc_buf[offset], buf, len);
				break;
		}
		if (!flush)
			return;

		/* Write the sparebytes into the page, once all eccsteps have been covered */
		for (i = 0; i < mtd->oobsize; i++)
			writeb(davinci_ecc_buf[i], this->IO_ADDR_W);
	}
#else
	for (i=0; i<len; i++)
		writeb(buf[i], this->IO_ADDR_W);
#endif
}

/**
 * nand_dm355_read_buf - [DEFAULT] read chip data into buffer 
 * @mtd:	MTD device structure
 * @buf:	buffer to store date
 * @len:	number of bytes to read
 *
 * Default read function for 8bit buswith
 */
static void nand_dm355_read_buf(struct mtd_info *mtd, u_char *buf, int len, int layout_idx, int index, int page)
{
	int i, j;
	int offset = 0;
	struct nand_chip *this = mtd->priv;

#ifdef CFG_DM355_NAND_STD_LAYOUT
	if ((this->eccmode == NAND_ECC_SOFT)) {
		for (i=0; i<len; i++)
			buf[i] = readb(this->IO_ADDR_R);
	}
	else {
		switch (this->layout[layout_idx].type) {
			case ITEM_TYPE_DATA:
				if (index == 0) {
					/* Seek to the OOB region using the cmdfunc, read OOB region into
					 * a temporary buffer and then seek back to the data region */
			        	this->cmdfunc (mtd, NAND_CMD_READOOB, 0x0, page & this->pagemask);
					for (j = 0; j < mtd->oobsize; j++)
						davinci_ecc_buf[j] = readb(this->IO_ADDR_R);
					this->cmdfunc (mtd, NAND_CMD_READ0, 0x0, page & this->pagemask);
					/* Enable HW ECC just before you read the data bytes */
					this->enable_hwecc(mtd, NAND_ECC_READ);
				}
				/* Read the 512 bytes data chunk */
				for (i = 0; i < len; i++)
					buf[i] = readb(this->IO_ADDR_R);
				break;
			case ITEM_TYPE_ECC:
				offset = (16 * index) + 6;
				memcpy(buf, &davinci_ecc_buf[offset], len);
				break;
			case ITEM_TYPE_OOB:
				offset = (16 * index);
				memcpy(buf, &davinci_ecc_buf[offset], len);
				break;
		}
	}
#else
	for (i=0; i<len; i++)
		buf[i] = readb(this->IO_ADDR_R);
#endif
}

static void nand_dm355evm_enable_hwecc(struct mtd_info *mtd, int mode)
{
        emifregs    emif_addr;

	    emif_addr = (emifregs)CSL_EMIF_1_REGS;
        emif_addr->NANDFCR |= (1 << 8);
}

static u32 nand_dm355evm_readecc(struct mtd_info *mtd, u32  Reg)
{
        u32      l = 0;
        emifregs    emif_addr;
	    emif_addr = (emifregs)CSL_EMIF_1_REGS;

        if (Reg == 1)
                l = emif_addr->NANDF1ECC;
        else if (Reg == 2)
                l = emif_addr->NANDF2ECC;
        else if (Reg == 3)
                l = emif_addr->NANDF3ECC;
        else if (Reg == 4)
                l = emif_addr->NANDF4ECC;

        return l;
}

static int nand_dm355evm_calculate_ecc(struct mtd_info *mtd, const u_char *dat,
				      u_char *ecc_code)
{
        unsigned int      l;
        int               reg;
        int               n;
        struct nand_chip *this = mtd->priv;

        if (this->eccmode == NAND_ECC_HW12_2048)
                n = 4;
        else
                n = 1;

        reg = 1;
        while (n--) {
                l = nand_dm355evm_readecc(mtd, reg);
                *ecc_code++ = l;          // P128e, ..., P1e
                *ecc_code++ = l >> 16;    // P128o, ..., P1o
                // P2048o, P1024o, P512o, P256o, P2048e, P1024e, P512e, P256e
                *ecc_code++ = ((l >> 8) & 0x0f) | ((l >> 20) & 0xf0);
                reg++;
        }
        return 0;
}

static void nand_dm355evm_gen_true_ecc(u8 *ecc_buf)
{
        u32 tmp = ecc_buf[0] | (ecc_buf[1] << 16) | ((ecc_buf[2] & 0xF0) << 20) | ((ecc_buf[2] & 0x0F) << 8);

        ecc_buf[0] = ~(P64o(tmp) | P64e(tmp) | P32o(tmp) | P32e(tmp) | P16o(tmp) | P16e(tmp) | P8o(tmp) | P8e(tmp) );
        ecc_buf[1] = ~(P1024o(tmp) | P1024e(tmp) | P512o(tmp) | P512e(tmp) | P256o(tmp) | P256e(tmp) | P128o(tmp) | P128e(tmp));
        ecc_buf[2] = ~( P4o(tmp) | P4e(tmp) | P2o(tmp) | P2e(tmp) | P1o(tmp) | P1e(tmp) | P2048o(tmp) | P2048e(tmp));
}

static int nand_dm355evm_compare_ecc(u8 *     ecc_data1,   /* read from NAND memory */
                                    u8 *     ecc_data2,   /* read from register */
                                    u8 *     page_data)
{
        u32    i;
        u8     tmp0_bit[8], tmp1_bit[8], tmp2_bit[8];
        u8     comp0_bit[8], comp1_bit[8], comp2_bit[8];
        u8     ecc_bit[24];
        u8     ecc_sum = 0;
        u8     find_bit = 0;
        u32    find_byte = 0;
        int    isEccFF;

        isEccFF = ((*(u32 *)ecc_data1 & 0xFFFFFF) == 0xFFFFFF);

        nand_dm355evm_gen_true_ecc(ecc_data1);
        nand_dm355evm_gen_true_ecc(ecc_data2);

        for (i = 0; i <= 2; i++) {
                *(ecc_data1 + i) = ~(*(ecc_data1 + i));
                *(ecc_data2 + i) = ~(*(ecc_data2 + i));
        }

        for (i = 0; i < 8; i++) {
                tmp0_bit[i]      = *ecc_data1 % 2;
                *ecc_data1       = *ecc_data1 / 2;
        }

        for (i = 0; i < 8; i++) {
                tmp1_bit[i]      = *(ecc_data1 + 1) % 2;
                *(ecc_data1 + 1) = *(ecc_data1 + 1) / 2;
        }

        for (i = 0; i < 8; i++) {
                tmp2_bit[i]      = *(ecc_data1 + 2) % 2;
                *(ecc_data1 + 2) = *(ecc_data1 + 2) / 2;
        }

        for (i = 0; i < 8; i++) {
                comp0_bit[i]     = *ecc_data2 % 2;
                *ecc_data2       = *ecc_data2 / 2;
        }

        for (i = 0; i < 8; i++) {
                comp1_bit[i]     = *(ecc_data2 + 1) % 2;
                *(ecc_data2 + 1) = *(ecc_data2 + 1) / 2;
        }

        for (i = 0; i < 8; i++) {
                comp2_bit[i]     = *(ecc_data2 + 2) % 2;
                *(ecc_data2 + 2) = *(ecc_data2 + 2) / 2;
        }

        for (i = 0; i< 6; i++ )
                ecc_bit[i] = tmp2_bit[i + 2] ^ comp2_bit[i + 2];

        for (i = 0; i < 8; i++)
                ecc_bit[i + 6] = tmp0_bit[i] ^ comp0_bit[i];

        for (i = 0; i < 8; i++)
                ecc_bit[i + 14] = tmp1_bit[i] ^ comp1_bit[i];

        ecc_bit[22] = tmp2_bit[0] ^ comp2_bit[0];
        ecc_bit[23] = tmp2_bit[1] ^ comp2_bit[1];

        for (i = 0; i < 24; i++)
                ecc_sum += ecc_bit[i];

        switch (ecc_sum) {
        case 0:
                /* Not reached because this function is not called if
                   ECC values are equal */
                return 0;

        case 1:
                /* Uncorrectable error */
                DEBUG (MTD_DEBUG_LEVEL0, "ECC UNCORRECTED_ERROR 1\n");
                return -1;

        case 12:
                /* Correctable error */
                find_byte = (ecc_bit[23] << 8) +
                            (ecc_bit[21] << 7) +
                            (ecc_bit[19] << 6) +
                            (ecc_bit[17] << 5) +
                            (ecc_bit[15] << 4) +
                            (ecc_bit[13] << 3) +
                            (ecc_bit[11] << 2) +
                            (ecc_bit[9]  << 1) +
                            ecc_bit[7];

                find_bit = (ecc_bit[5] << 2) + (ecc_bit[3] << 1) + ecc_bit[1];

                DEBUG (MTD_DEBUG_LEVEL0, "Correcting single bit ECC error at offset: %d, bit: %d\n", find_byte, find_bit);

                page_data[find_byte] ^= (1 << find_bit);

                return 0;

        default:
                if (isEccFF) {
                        if (ecc_data2[0] == 0 && ecc_data2[1] == 0 && ecc_data2[2] == 0)
                                return 0;
                }
                DEBUG (MTD_DEBUG_LEVEL0, "UNCORRECTED_ERROR default\n");
                return -1;
        }
}

static int nand_dm355evm_correct_data(struct mtd_info *mtd, u_char *dat,
				     u_char *read_ecc, u_char *calc_ecc)
{
	int r = 0;
#if 0
	if (memcmp(read_ecc, calc_ecc, 3) != 0) {
		u_char read_ecc_copy[3], calc_ecc_copy[3];
		int i;

		for (i = 0; i < 3; i++) {
			read_ecc_copy[i] = read_ecc[i];
			calc_ecc_copy[i] = calc_ecc[i];
		}
		r = nand_dm355_1bit_compare_ecc(read_ecc_copy, calc_ecc_copy,
						  dat);
	}
#endif
	return r;
}

/*
 * 4-bit ECC routines
 */

/*
 * Instead of placing the spare data at the end of the page, the 4-bit ECC
 * hardware generator requires that the page be subdivided into 4 subpages,
 * each with its own spare data area.  This structure defines the format of
 * each of these subpages.
 */
static struct page_layout_item nand_dm355_hw10_512_layout[] = {
	{.type = ITEM_TYPE_DATA,.length = 512},
	{.type = ITEM_TYPE_OOB,.length = 6,},
	{.type = ITEM_TYPE_ECC,.length = 10,},
	{.type = 0,.length = 0,},
};

static struct nand_oobinfo nand_dm355_hw10_512_oobinfo = {
	.useecc = MTD_NANDECC_AUTOPLACE,
	/*
	 * We actually have 40 bytes of ECC per page, but the nand_oobinfo
	 * structure definition limits us to a maximum of 32 bytes.  This
	 * doesn't matter, because out page_layout_item structure definition
	 * determines where our ECC actually goes in the flash page.
	 */
	.eccbytes = 32,
	.eccpos = {6, 7, 8, 9, 10, 11, 12, 13, 14, 15,
		   22, 23, 24, 25, 26, 27, 28, 29, 30, 31,
		   38, 39, 40, 41, 42, 43, 44, 45, 46, 47,
		   54, 55,
		   },
	.oobfree = {{0, 6}, {16, 6}, {32, 6}, {48, 6}},
};

/*
 * We should always have a flash-based bad block table.  However, if one isn't
 * found then all blocks will be scanned to look for factory-marked bad blocks.
 * We supply a null pattern so that no blocks will be detected as bad.
 */
static struct nand_bbt_descr nand_dm355_hw10_512_badblock_pattern = {
	.options = 0,
	.offs = 0,
	.len = 0,
	.pattern = NULL,
};

/*
 * When using 4-bit ECC with a 2048-byte data + 64-byte spare page size, the
 * oob is scattered throughout the page in 4 16-byte chunks instead of being
 * grouped together at the end of the page.  This means that the factory
 * bad-block markers at offsets 2048 and 2049 will be overwritten when data
 * is written to the flash.  Thus, we cannot use the factory method to mark
 * or detect bad blocks and must rely on a flash-based bad block table instead.
 *
 */
static int nand_dm355_hw10_512_block_bad(struct mtd_info *mtd, loff_t ofs,
					   int getchip)
{
	return 0;
}

static int nand_dm355_hw10_512_block_markbad(struct mtd_info *mtd, loff_t ofs)
{
	struct nand_chip *this = mtd->priv;
	int block;

	/* Get block number */
	block = ((int)ofs) >> this->bbt_erase_shift;
	if (this->bbt)
		this->bbt[block >> 2] |= 0x01 << ((block & 0x03) << 1);

	/* Do we have a flash based bad block table ? */
	if (this->options & NAND_USE_FLASH_BBT)
		return nand_update_bbt(mtd, ofs);

	return 0;
}

static void nand_dm355_4bit_enable_hwecc(struct mtd_info *mtd, int mode)
{
    emifregs    emif_addr = (emifregs)CSL_EMIF_1_REGS;
	u32 val;

	switch (mode) {
	case NAND_ECC_WRITE:
	case NAND_ECC_READ:
		/*
		 * Start a new ECC calculation for reading or writing 512 bytes
		 *  of data.
		 */
		val = (emif_addr->NANDFCR & ~(3 << 4))
		    | (1 << 12);
        emif_addr->NANDFCR = val;
		break;
	case NAND_ECC_WRITEOOB:
	case NAND_ECC_READOOB:
		/*
		 * Terminate ECC calculation by performing a dummy read of an
		 * ECC register.  Our hardware ECC generator supports including
		 * the OOB in the ECC calculation, but the NAND core code
		 * doesn't really support that.  We will only calculate the ECC
		 * on the data; errors in the non-ECC bytes in the OOB will not
		 * be detected or corrected.
		 */
		val = emif_addr->NANDF1ECC;
		break;
	case NAND_ECC_WRITESYN:
	case NAND_ECC_READSYN:
		/*
		 * Our ECC calculation has already been terminated, so no need
		 * to do anything here.
		 */
		break;
	default:
		break;
	}
}

static u32 nand_dm355_4bit_readecc(struct mtd_info *mtd, unsigned int ecc[4])
{
	ecc[0] = (*(dv_reg_p) NAND4BITECC1) & NAND_4BITECC_MASK;
	ecc[1] = (*(dv_reg_p) NAND4BITECC2) & NAND_4BITECC_MASK;
	ecc[2] = (*(dv_reg_p) NAND4BITECC3) & NAND_4BITECC_MASK;
	ecc[3] = (*(dv_reg_p) NAND4BITECC4) & NAND_4BITECC_MASK;

	return 0;
}

static int nand_dm355_4bit_calculate_ecc(struct mtd_info *mtd,
					   const u_char * dat,
					   u_char * ecc_code)
{
	unsigned int hw_4ecc[4] = { 0, 0, 0, 0 };
	unsigned int const1 = 0, const2 = 0;
	unsigned char count1 = 0;

	/*
	 * Since the NAND_HWECC_SYNDROME option is enabled, this routine is
	 * only called just after the data and oob have been written.  The
	 * ECC value calculated by the hardware ECC generator is available
	 * for us to read.
	 */
	nand_dm355_4bit_readecc(mtd, hw_4ecc);

	/*Convert 10 bit ecc value to 8 bit */
	for (count1 = 0; count1 < 2; count1++) {
		const2 = count1 * 5;
		const1 = count1 * 2;

		/* Take first 8 bits from val1 (count1=0) or val5 (count1=1) */
		ecc_code[const2] = hw_4ecc[const1] & 0xFF;

		/*
		 * Take 2 bits as LSB bits from val1 (count1=0) or val5
		 * (count1=1) and 6 bits from val2 (count1=0) or val5 (count1=1)
		 */
		ecc_code[const2 + 1] =
		    ((hw_4ecc[const1] >> 8) & 0x3) | ((hw_4ecc[const1] >> 14) &
						      0xFC);

		/*
		 * Take 4 bits from val2 (count1=0) or val5 (count1=1) and
		 * 4 bits from val3 (count1=0) or val6 (count1=1)
		 */
		ecc_code[const2 + 2] =
		    ((hw_4ecc[const1] >> 22) & 0xF) |
		    ((hw_4ecc[const1 + 1] << 4) & 0xF0);

		/*
		 * Take 6 bits from val3(count1=0) or val6 (count1=1) and
		 * 2 bits from val4 (count1=0) or  val7 (count1=1)
		 */
		ecc_code[const2 + 3] =
		    ((hw_4ecc[const1 + 1] >> 4) & 0x3F) |
		    ((hw_4ecc[const1 + 1] >> 10) & 0xC0);

		/* Take 8 bits from val4 (count1=0) or val7 (count1=1) */
		ecc_code[const2 + 4] = (hw_4ecc[const1 + 1] >> 18) & 0xFF;
	}

	return 0;
}

static int nand_dm355_4bit_compare_ecc(struct mtd_info *mtd, u8 * read_ecc,	/* read from NAND */
					 u8 * page_data)
{
	struct nand_chip *this = mtd->priv;
	unsigned short ecc_10bit[8] = { 0, 0, 0, 0, 0, 0, 0, 0 };
	int i;
	unsigned int hw_4ecc[4] = { 0, 0, 0, 0 }, iserror = 0;
	unsigned short *pspare = NULL, *pspare1 = NULL;
	unsigned int numErrors, errorAddress, errorValue;
	emifregs    emif_addr = (emifregs)CSL_EMIF_1_REGS;
	u32 val;

	/*
	 * Check for an ECC where all bytes are 0xFF.  If this is the case, we
	 * will assume we are looking at an erased page and we should ignore the
	 * ECC.
	 */
	for (i = 0; i < 10; i++) {
		if (read_ecc[i] != 0xFF)
			break;
	}
	if (i == 10)
		return 0;

	/* Convert 8 bit in to 10 bit */
	pspare = (unsigned short *)&read_ecc[2];
	pspare1 = (unsigned short *)&read_ecc[0];
	/* Take 10 bits from 0th and 1st bytes */
	ecc_10bit[0] = (*pspare1) & 0x3FF;	/* 10 */
	/* Take 6 bits from 1st byte and 4 bits from 2nd byte */
	ecc_10bit[1] = (((*pspare1) >> 10) & 0x3F)
	    | (((pspare[0]) << 6) & 0x3C0);	/* 6 + 4 */
	/* Take 4 bits form 2nd bytes and 6 bits from 3rd bytes */
	ecc_10bit[2] = ((pspare[0]) >> 4) & 0x3FF;	/* 10 */
	/*Take 2 bits from 3rd byte and 8 bits from 4th byte */
	ecc_10bit[3] = (((pspare[0]) >> 14) & 0x3)
	    | ((((pspare[1])) << 2) & 0x3FC);	/* 2 + 8 */
	/* Take 8 bits from 5th byte and 2 bits from 6th byte */
	ecc_10bit[4] = ((pspare[1]) >> 8)
	    | ((((pspare[2])) << 8) & 0x300);	/* 8 + 2 */
	/* Take 6 bits from 6th byte and 4 bits from 7th byte */
	ecc_10bit[5] = (pspare[2] >> 2) & 0x3FF;	/* 10 */
	/* Take 4 bits from 7th byte and 6 bits from 8th byte */
	ecc_10bit[6] = (((pspare[2]) >> 12) & 0xF)
	    | ((((pspare[3])) << 4) & 0x3F0);	/* 4 + 6 */
	/*Take 2 bits from 8th byte and 8 bits from 9th byte */
	ecc_10bit[7] = ((pspare[3]) >> 6) & 0x3FF;	/* 10 */

	/*
	 * Write the parity values in the NAND Flash 4-bit ECC Load register.
	 * Write each parity value one at a time starting from 4bit_ecc_val8
	 * to 4bit_ecc_val1.
	 */
	for (i = 7; i >= 0; i--)
	{
		*(dv_reg_p)NAND4BITECCLOAD = ecc_10bit[i];  
	}

	/*
	 * Perform a dummy read to the EMIF Revision Code and Status register.
	 * This is required to ensure time for syndrome calculation after
	 * writing the ECC values in previous step.
	 */
	val = emif_addr->ERCSR;

	/*
	 * Read the syndrome from the NAND Flash 4-Bit ECC 1-4 registers.
	 * A syndrome value of 0 means no bit errors. If the syndrome is
	 * non-zero then go further otherwise return.
	 */
	nand_dm355_4bit_readecc(mtd, hw_4ecc);

	if (hw_4ecc[0] == ECC_STATE_NO_ERR && hw_4ecc[1] == ECC_STATE_NO_ERR &&
	    hw_4ecc[2] == ECC_STATE_NO_ERR && hw_4ecc[3] == ECC_STATE_NO_ERR)
		return 0;

	/*
	 * Clear any previous address calculation by doing a dummy read of an
	 * error address register.
	 */
	val = *(dv_reg_p)NANDERRADD1;

	/*
	 * Set the addr_calc_st bit(bit no 13) in the NAND Flash Control
	 * register to 1.
	 */
    
    emif_addr->NANDFCR |= (1 << 13);

	/*
	 * Wait for the corr_state field (bits 8 to 11)in the
	 * NAND Flash Status register to be equal to 0x0, 0x1, 0x2, or 0x3.
	 */
    do {
        iserror = emif_addr->NANDFSR & 0xC00;
    } while (iserror);       

	iserror = emif_addr->NANDFSR;
	iserror &= EMIF_NANDFSR_ECC_STATE_MASK;
	iserror = iserror >> 8;

	/*
	 * ECC_STATE_TOO_MANY_ERRS (0x1) means errors cannot be
	 * corrected (five or more errors).  The number of errors
	 * calculated (err_num field) differs from the number of errors
	 * searched.  ECC_STATE_ERR_CORR_COMP_P (0x2) means error
	 * correction complete (errors on bit 8 or 9).
	 * ECC_STATE_ERR_CORR_COMP_N (0x3) means error correction
	 * complete (error exists).
	 */
	udelay (this->chip_delay);
	if (iserror == ECC_STATE_NO_ERR)
		return 0;
	else if (iserror == ECC_STATE_TOO_MANY_ERRS)
	{
		printf("too many errors to be corrected!\n");
		return -1;
	}

	numErrors = ((emif_addr->NANDFSR >> 16) & 0x3) + 1;

	/* Read the error address, error value and correct */
	for (i = 0; i < numErrors; i++) {
		if (i > 1) {
			errorAddress =
			    ((*(dv_reg_p)(NANDERRADD2) >>
			      (16 * (i & 1))) & 0x3FF);
			errorAddress = ((512 + 7) - errorAddress);
			errorValue =
			    ((*(dv_reg_p)(NANDERRVAL2) >>
			      (16 * (i & 1))) & 0xFF);
		} else {
			errorAddress =
			    ((*(dv_reg_p)(NANDERRADD1) >>
			      (16 * (i & 1))) & 0x3FF);
			errorAddress = ((512 + 7) - errorAddress);
			errorValue =
			    ((*(dv_reg_p)(NANDERRVAL1) >>
			      (16 * (i & 1))) & 0xFF);
		}
		/* xor the corrupt data with error value */
		if (errorAddress < 512)
			page_data[errorAddress] ^= errorValue;
	}

	return numErrors;
}

static int nand_dm355_4bit_correct_data(struct mtd_info *mtd, u_char * dat,
					  u_char * read_ecc, u_char * calc_ecc)
{
	int r = 0;

	/*
	 * dat points to 512 bytes of data.  read_ecc points to the start of the
	 * oob area for this subpage, so the ecc values start at offset 6.
	 * The calc_ecc pointer is not needed since our caclulated ECC is
	 * already latched in the hardware ECC generator.
	 */
	r = nand_dm355_4bit_compare_ecc(mtd, read_ecc + 6, dat);

	return r;
}

int board_nand_init(struct nand_chip *nand)
{
	nand->chip_delay  = 0;
	nand->eccmode     = NAND_ECC_HW10_512;
	nand->options     = NAND_USE_FLASH_BBT | NAND_BBT_LASTBLOCK | NAND_HWECC_SYNDROME;
	nand->autooob = &nand_dm355_hw10_512_oobinfo;
	nand->layout = nand_dm355_hw10_512_layout;
	nand->calculate_ecc = nand_dm355_4bit_calculate_ecc;
	nand->correct_data = nand_dm355_4bit_correct_data;
	nand->enable_hwecc = nand_dm355_4bit_enable_hwecc;
	nand->read_buf = nand_dm355_read_buf;
	nand->write_buf = nand_dm355_write_buf;
	nand->block_markbad = nand_dm355_hw10_512_block_markbad;
	/* Set address of hardware control function */
	nand->hwcontrol = nand_dm350evm_hwcontrol;
	return 0;
}

#else
#error "U-Boot legacy NAND support not available for DaVinci chips"
#endif
#endif	/* CFG_USE_NAND */
