/*
 * SPI ethernet driver for SPI to 100Mbps ethernet chip ch390.
 *
 * Copyright (C) 2024 Nanjing Qinheng Microelectronics Co., Ltd.
 * Web:      http://wch.cn
 * Author:   WCH <tech@wch.cn>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 * 
 * System required:
 * Kernel version beyond 4.0.x
 * Update Log:
 * V1.0 - initial version
 * V1.1 - use spi core APIs instead of regmap
 */

#include <linux/etherdevice.h>
#include <linux/ethtool.h>
#include <linux/interrupt.h>
#include <linux/iopoll.h>
#include <linux/irq.h>
#include <linux/gpio.h>
#include <linux/mii.h>
#include <linux/module.h>
#include <linux/netdevice.h>
#include <linux/phy.h>
#include <linux/skbuff.h>
#include <linux/spinlock.h>
#include <linux/spi/spi.h>
#include <linux/types.h>
#include <linux/crc32.h>
#include <linux/version.h>

#include "ch390.h"

#define DRVNAME_CH390 "ch390"
#define DRIVER_AUTHOR "WCH"
#define DRIVER_DESC   "SPI ethernet driver for ch390, etc."
#define VERSION_DESC  "V1.1 On 2024.09"

/*
 * struct rx_ctl_mach - rx activities record
 * @status_err_counter: rx status error counter
 * @large_err_counter: rx get large packet length error counter
 * @rx_err_counter: receive packet error counter
 * @tx_err_counter: transmit packet error counter
 * @fifo_rst_counter: reset operation counter
 *
 * To keep track for the driver operation statistics
 */
struct rx_ctl_mach {
	u16 status_err_counter;
	u16 large_err_counter;
	u16 rx_err_counter;
	u16 tx_err_counter;
	u16 fifo_rst_counter;
};

/*
 * struct ch390_rxctrl - ch390 driver rx control
 * @hash_table: Multicast hash-table data
 * @rcr_all: KS_RXCR1 register setting
 *
 * The settings needs to control the receive filtering
 * such as the multicast hash-filter and the receive register settings
 */
struct ch390_rxctrl {
	u16 hash_table[4];
	u8 rcr_all;
};

/*
 * struct ch390_rxhdr - rx packet data header
 * @headbyte: lead byte equal to 0x01 notifies a valid packet
 * @status: status bits for the received packet
 * @rxlen: packet length
 *
 * The Rx packed, entered into the FIFO memory, start with these
 * four bytes which is the Rx header, followed by the ethernet
 * packet data and ends with an appended 4-byte CRC data.
 * Both Rx packet and CRC data are for check purpose and finally
 * are dropped by this driver
 */
struct ch390_rxhdr {
	u8 headbyte;
	u8 status;
	__le16 rxlen;
};

enum val_type {
	TYPE_U8,
	TYPE_U16
};

/*
 * struct board_info - maintain the saved data
 * @spidev: spi device structure
 * @ndev: net device structure
 * @mdiobus: mii bus structure
 * @phydev: phy device structure
 * @txq: tx queue structure
 * @rxctrl_work: Work queue for updating RX mode and multicast lists
 * @tx_work: Work queue for tx packets
 * @pause: ethtool pause parameter structure
 * @spi_lockm: between threads lock structure
 * @reg_mutex: reg write/read lock structure
 * @bc: rx control statistics structure
 * @rxhdr: rx header structure
 * @rctl: rx control setting structure
 * @msg_enable: message level value
 * @imr_all: to store operating imr value for register ch390_IMR
 * @lcr_all: to store operating rcr value for register ch390_LMCR
 * @reg_stride: The register address stride. Valid register addresses are a
 * 				multiple of this value. If set to 0, a value of 1 will be
 *              used.
 * The saved data variables, keep up to date for retrieval back to use
 */
struct board_info {
	u32 msg_enable;
	struct spi_device *spidev;
	struct net_device *ndev;
	struct mii_bus *mdiobus;
	struct phy_device *phydev;
	struct sk_buff_head txq;
	struct work_struct rxctrl_work;
	struct work_struct tx_work;
	struct ethtool_pauseparam pause;
	struct mutex spi_lockm;
	struct mutex reg_mutex;
	struct rx_ctl_mach bc;
	struct ch390_rxhdr rxhdr;
	struct ch390_rxctrl rctl;
	u8 imr_all;
	u8 lcr_all;
	int reg_stride;
};

static int ch390_set_reg(struct board_info *db, unsigned int reg, unsigned int val)
{
	int ret;
	u8 cmd_buf[2];

	cmd_buf[0] = (OPC_REG_W | reg);
	cmd_buf[1] = val;

	ret = spi_write(db->spidev, cmd_buf, sizeof(cmd_buf));
	if (ret < 0)
		netif_err(db, drv, db->ndev, "%s: error %d set reg %02x\n", __func__, ret, reg);

	return ret;
}

static int ch390_get_reg(struct board_info *db, unsigned int reg, void *val)
{
	int ret;
	u8 cmd = (OPC_REG_R | reg);

	ret = spi_write_then_read(db->spidev, &cmd, sizeof(cmd), val, 1);
	if (ret)
		netif_err(db, drv, db->ndev, "%s: error %d get reg %02x\n", __func__, ret, reg);

	return ret;
}

static int ch390_update_bits(struct board_info *db, unsigned int reg, unsigned int mask, unsigned int val)
{
	int ret;
	u8 current_val;

	mutex_lock(&db->reg_mutex);

	ret = ch390_get_reg(db, reg, &current_val);
	if (ret < 0) {
		netif_err(db, drv, db->ndev, "%s: error %d reading reg %02x\n", __func__, ret, reg);
		mutex_unlock(&db->reg_mutex);
		return ret;
	}

	current_val = (current_val & ~mask) | (val & mask);

	ret = ch390_set_reg(db, reg, current_val);
	if (ret < 0) {
		netif_err(db, drv, db->ndev, "%s: error %d writing reg %02x\n", __func__, ret, reg);
	}

	mutex_unlock(&db->reg_mutex);

	return ret;
}

/*
 * skb buffer exhausted, just discard the received data
 */
static int ch390_dumpblk(struct board_info *db, u8 reg, size_t count)
{
	struct net_device *ndev = db->ndev;
	u8 cmd_buf[2];
	u8 rb;
	int ret;

	cmd_buf[0] = OPC_MEM_READ;
	cmd_buf[1] = 0x00;

	do {
		ret = spi_write_then_read(db->spidev, cmd_buf, sizeof(cmd_buf), &rb, sizeof(rb));
		if (ret < 0) {
			netif_err(db, drv, ndev, "%s: error %d dumping read reg %02x\n", __func__, ret, reg);
			break;
		}
	} while (--count);

	return ret;
}

static int ch390_set_regs(struct board_info *db, unsigned int reg, const void *val, size_t val_count,
			  enum val_type type)
{
	int ret = 0;
	int i;

	if (!IS_ALIGNED(reg, db->reg_stride))
		return -EINVAL;

	mutex_lock(&db->reg_mutex);

	if (type == TYPE_U16) {
		const u16 *val_words = (const u16 *)val;
		for (i = 0; i < val_count / 2; i++) {
			ret = ch390_set_reg(db, reg + i * 2, val_words[i] & 0xFF);
			if (ret < 0) {
				netif_err(db, drv, db->ndev, "%s: error %d writing reg %02x\n", __func__, ret,
					  reg + i * 2);
				break;
			}

			ret = ch390_set_reg(db, reg + i * 2 + 1, (val_words[i] >> 8) & 0xFF);
			if (ret < 0) {
				netif_err(db, drv, db->ndev, "%s: error %d writing reg %02x\n", __func__, ret,
					  reg + i * 2 + 1);
				break;
			}
		}
	} else {
		const u8 *val_bytes = (const u8 *)val;
		for (i = 0; i < val_count; i++) {
			ret = ch390_set_reg(db, reg + i, val_bytes[i]);
			if (ret < 0) {
				netif_err(db, drv, db->ndev, "%s: error %d writing reg %02x\n", __func__, ret, reg + i);
				break;
			}
		}
	}

	mutex_unlock(&db->reg_mutex);

	return ret;
}

static int ch390_get_regs(struct board_info *db, unsigned int reg, void *val, size_t val_count, enum val_type type)
{
	int ret = 0;
	int i;

	if (!IS_ALIGNED(reg, db->reg_stride))
		return -EINVAL;

	mutex_lock(&db->reg_mutex);

	if (type == TYPE_U16) {
		u16 *val_words = (u16 *)val;
		for (i = 0; i < val_count / 2; i++) {
			u8 low, high;

			ret = ch390_get_reg(db, reg + i * 2, &low);
			if (ret < 0) {
				netif_err(db, drv, db->ndev, "%s: error %d reading reg %02x\n", __func__, ret,
					  reg + i * 2);
				break;
			}

			ret = ch390_get_reg(db, reg + i * 2 + 1, &high);
			if (ret < 0) {
				netif_err(db, drv, db->ndev, "%s: error %d reading reg %02x\n", __func__, ret,
					  reg + i * 2 + 1);
				break;
			}

			val_words[i] = (high << 8) | low;
		}
	} else {
		u8 *val_bytes = (u8 *)val;
		for (i = 0; i < val_count; i++) {
			ret = ch390_get_reg(db, reg + i, &val_bytes[i]);
			if (ret < 0) {
				netif_err(db, drv, db->ndev, "%s: error %d reading reg %02x\n", __func__, ret, reg + i);
				break;
			}
		}
	}

	mutex_unlock(&db->reg_mutex);

	return ret;
}

/*
 * Note: The declaration of 'struct spi_transfer xfer' is intentionally placed after
 * the initialization of 'tx_buf' to avoid compiler warnings.  This is a workaround
 * for systems that experience issues with bufferless transfers of length 1.
 */

static int ch390_write_mem(struct board_info *db, unsigned int reg, const void *buff, size_t len)
{
	int ret;
	struct spi_message msg;
	u8 *tx_buf;

	tx_buf = kzalloc(len + 1, GFP_KERNEL);
	if (!tx_buf)
		return -ENOMEM;

	tx_buf[0] = reg;

	memcpy(tx_buf + 1, buff, len);

	struct spi_transfer xfer = {
		.tx_buf = tx_buf,
		.rx_buf = NULL,
		.len = len + 1,
	};

	spi_message_init(&msg);
	spi_message_add_tail(&xfer, &msg);

	ret = spi_sync(db->spidev, &msg);
	if (ret < 0)
		netif_err(db, drv, db->ndev, "%s: error %d writing regs %02x\n", __func__, ret, reg);

	kfree(tx_buf);

	return ret;
}

static int ch390_read_mem(struct board_info *db, unsigned int reg, void *buff, size_t len)
{
	int ret;
	struct spi_message msg;
	u8 cmd_buf = reg;

	struct spi_transfer xfer1 = {
		.tx_buf = &cmd_buf,
		.rx_buf = NULL,
		.len = sizeof(cmd_buf),
	};
	struct spi_transfer xfer2 = {
		.tx_buf = NULL,
		.rx_buf = buff,
		.len = len,
	};
	spi_message_init(&msg);

	spi_message_add_tail(&xfer1, &msg);
	spi_message_add_tail(&xfer2, &msg);

	ret = spi_sync(db->spidev, &msg);
	if (ret < 0)
		netif_err(db, drv, db->ndev, "%s: error %d reading regs %02x\n", __func__, ret, reg);

	return ret;
}

static int ch390_wait_for_condition(struct board_info *db, unsigned int reg, u8 mask, unsigned int timeout_us,
				    unsigned int delay_us)
{
	unsigned int elapsed_us = 0;
	u8 mval;
	int ret;

	while (elapsed_us < timeout_us) {
		ret = ch390_get_reg(db, reg, &mval);
		if (ret < 0)
			return ret;

		if (!(mval & mask))
			return 0;

		usleep_range(delay_us / 2, delay_us);
		elapsed_us += delay_us;
	}

	return -ETIMEDOUT;
}

static int ch390_epcr_poll(struct board_info *db)
{
	int ret;

	ret = ch390_wait_for_condition(db, CH390_EPCR, EPCR_ERRE, 10000, 100);
	if (ret == -ETIMEDOUT)
		netdev_err(db->ndev, "eeprom/phy in processing get timeout\n");

	return ret;
}

static int ch390_irq_flag(struct board_info *db)
{
	struct spi_device *spi = db->spidev;
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 11, 0))
	int irq_type = irq_get_trigger_type(spi->irq);
#else
	struct irq_data *d = irq_get_irq_data(spi->irq);
	int irq_type = d ? irqd_get_trigger_type(d) : 0;
#endif

	if (irq_type)
		return irq_type;

	return IRQF_TRIGGER_LOW;
}

static unsigned int ch390_intcr_value(struct board_info *db)
{
	return (ch390_irq_flag(db) == IRQF_TRIGGER_LOW) ? INCR_POL_L : INCR_POL_H;
}

static int ch390_set_fcr(struct board_info *db)
{
	u8 fcr = 0;

	if (db->pause.rx_pause)
		fcr |= FCR_BKPM | FCR_FLCE;
	if (db->pause.tx_pause)
		fcr |= FCR_TXPEN;

	return ch390_set_reg(db, CH390_FCR, fcr);
}

static int ch390_set_recv(struct board_info *db)
{
	int ret;

	ret = ch390_set_regs(db, CH390_MAR, db->rctl.hash_table, sizeof(db->rctl.hash_table), TYPE_U16);
	if (ret)
		return ret;

	return ch390_set_reg(db, CH390_RCR, db->rctl.rcr_all); /* enable rx */
}

static int ch390_core_reset(struct board_info *db)
{
	int ret;

	db->bc.fifo_rst_counter++;

	ret = ch390_set_reg(db, CH390_NCR, NCR_RST); /* NCR reset */
	if (ret)
		return ret;

	ret = ch390_set_reg(db, CH390_MLEDCR, db->lcr_all); /* LEDMode1 */
	if (ret)
		return ret;

	return ch390_set_reg(db, CH390_INTCR, ch390_intcr_value(db));
}

static int ch390_update_fcr(struct board_info *db)
{
	u8 fcr = 0;

	if (db->pause.rx_pause)
		fcr |= FCR_BKPM | FCR_FLCE;
	if (db->pause.tx_pause)
		fcr |= FCR_TXPEN;

	return ch390_update_bits(db, CH390_FCR, FCR_RXTX_BITS, fcr);
}

static int ch390_disable_interrupt(struct board_info *db)
{
	return ch390_set_reg(db, CH390_IMR, IMR_PAR);
}

static int ch390_enable_interrupt(struct board_info *db)
{
	return ch390_set_reg(db, CH390_IMR, IMR_ALL);
}

static int ch390_clear_interrupt(struct board_info *db)
{
	return ch390_update_bits(db, CH390_ISR, ISR_CLR_INT, ISR_CLR_INT);
}

static int ch390_eeprom_read(struct board_info *db, int offset, u8 *to)
{
	int ret;

	ret = ch390_set_reg(db, CH390_EPAR, offset);
	if (ret)
		return ret;

	ret = ch390_set_reg(db, CH390_EPCR, EPCR_ERPRR);
	if (ret)
		return ret;

	ret = ch390_epcr_poll(db);
	if (ret)
		return ret;

	ret = ch390_set_reg(db, CH390_EPCR, 0);
	if (ret)
		return ret;
	return ch390_get_regs(db, CH390_EPDRL, to, 2, TYPE_U8);
}

static int ch390_eeprom_write(struct board_info *db, int offset, u8 *data)
{
	int ret;

	ret = ch390_set_reg(db, CH390_EPAR, offset);
	if (ret)
		return ret;

	ret = ch390_set_regs(db, CH390_EPDRL, data, 2, TYPE_U8);
	if (ret < 0)
		return ret;

	ret = ch390_set_reg(db, CH390_EPCR, EPCR_WEP | EPCR_ERPRW);
	if (ret)
		return ret;

	ret = ch390_epcr_poll(db);
	if (ret)
		return ret;

	return ch390_set_reg(db, CH390_EPCR, 0);
}

static int ch390_phyread(void *context, unsigned int reg, unsigned int *val)
{
	struct board_info *db = context;
	int ret;

	ret = ch390_set_reg(db, CH390_EPAR, CH390_PHY | reg);
	if (ret)
		return ret;

	ret = ch390_set_reg(db, CH390_EPCR, EPCR_ERPRR | EPCR_EPOS);
	if (ret)
		return ret;

	ret = ch390_epcr_poll(db);
	if (ret)
		return ret;

	ret = ch390_set_reg(db, CH390_EPCR, 0);
	if (ret)
		return ret;

	*val = 0;
	return ch390_get_regs(db, CH390_EPDRL, val, 2, TYPE_U16);
}

static int ch390_phywrite(void *context, unsigned int reg, u16 val)
{
	struct board_info *db = context;
	int ret;

	ret = ch390_set_reg(db, CH390_EPAR, CH390_PHY | reg);
	if (ret)
		return ret;

	ret = ch390_set_regs(db, CH390_EPDRL, &val, 2, TYPE_U16);
	if (ret < 0)
		return ret;

	ret = ch390_set_reg(db, CH390_EPCR, EPCR_EPOS | EPCR_ERPRW);
	if (ret)
		return ret;

	ret = ch390_epcr_poll(db);
	if (ret)
		return ret;

	return ch390_set_reg(db, CH390_EPCR, 0);
}

static int ch390_mdio_read(struct mii_bus *bus, int addr, int regnum)
{
	struct board_info *db = bus->priv;
	unsigned int val = 0xffff;
	int ret;

	if (addr == CH390_PHY_ADDR) {
		ret = ch390_phyread(db, regnum, &val);
		if (ret)
			return ret;
	}

	return val;
}

static int ch390_mdio_write(struct mii_bus *bus, int addr, int regnum, u16 val)
{
	struct board_info *db = bus->priv;

	if (addr == CH390_PHY_ADDR)
		return ch390_phywrite(db, regnum, val);

	return -ENODEV;
}

static int ch390_map_chipid(struct board_info *db)
{
	struct device *dev = &db->spidev->dev;
	unsigned short wid;
	u8 buff[6];
	int ret;

	ret = ch390_get_regs(db, CH390_VIDL, buff, sizeof(buff), TYPE_U8);
	if (ret < 0)
		return ret;

	wid = get_unaligned_le16(buff + 2);
	if (wid != CH390_ID) {
		dev_err(dev, "chipid error as %04x !\n", wid);
		return -ENODEV;
	}

	dev_info(dev, "chip %04x found\n", wid);
	return 0;
}

/* 
 * Read ch390_PAR registers which is the mac address loaded from EEPROM while power-on
 */
static int ch390_map_etherdev_par(struct net_device *ndev, struct board_info *db)
{
	u8 addr[ETH_ALEN];
	int ret;

	ret = ch390_get_regs(db, CH390_PAR, addr, sizeof(addr), TYPE_U8);
	if (ret < 0)
		return ret;

	if (!is_valid_ether_addr(addr)) {
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 4, 0))
		eth_hw_addr_random(ndev);
#else
		random_ether_addr(ndev->dev_addr);
#endif

		ret = ch390_set_regs(db, CH390_PAR, ndev->dev_addr, sizeof(ndev->dev_addr), TYPE_U8);
		if (ret < 0)
			return ret;

		dev_dbg(&db->spidev->dev, "Use random MAC address\n");
		return 0;
	}

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 16, 0))
	eth_hw_addr_set(ndev, addr);
#else
	ether_addr_copy(ndev->dev_addr, addr);
#endif

	return 0;
}

/*
 * ethtool-ops
 */
static void ch390_get_drvinfo(struct net_device *ndev, struct ethtool_drvinfo *info)
{
	strlcpy(info->driver, DRVNAME_CH390, sizeof(info->driver));
}

#if (LINUX_VERSION_CODE <= KERNEL_VERSION(4, 5, 0))
static int ch390_get_settings(struct net_device *ndev, struct ethtool_cmd *cmd)
{
	if (!ndev->phydev)
		return -ENODEV;

	return phy_ethtool_gset(ndev->phydev, cmd);
}

static int ch390_set_settings(struct net_device *ndev, struct ethtool_cmd *cmd)
{
	if (!ndev->phydev)
		return -ENODEV;

	return phy_ethtool_sset(ndev->phydev, cmd);
}
#endif

static void ch390_set_msglevel(struct net_device *ndev, u32 value)
{
	struct board_info *db = to_ch390_board(ndev);

	db->msg_enable = value;
}

static u32 ch390_get_msglevel(struct net_device *ndev)
{
	struct board_info *db = to_ch390_board(ndev);

	return db->msg_enable;
}

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(4, 10, 0))
#else
static int ch390_nway_reset(struct net_device *ndev)
{
	if (!ndev->phydev)
		return -ENODEV;

	return phy_start_aneg(ndev->phydev);
}
#endif

static int ch390_get_eeprom_len(struct net_device *ndev)
{
	return 128;
}

static int ch390_get_eeprom(struct net_device *ndev, struct ethtool_eeprom *ee, u8 *data)
{
	struct board_info *db = to_ch390_board(ndev);
	int offset = ee->offset;
	int len = ee->len;
	int i, ret;

	if ((len | offset) & 1)
		return -EINVAL;

	ee->magic = CH390_EEPROM_MAGIC;

	for (i = 0; i < len; i += 2) {
		ret = ch390_eeprom_read(db, (offset + i) / 2, data + i);
		if (ret)
			break;
	}
	return ret;
}

static int ch390_set_eeprom(struct net_device *ndev, struct ethtool_eeprom *ee, u8 *data)
{
	struct board_info *db = to_ch390_board(ndev);
	int offset = ee->offset;
	int len = ee->len;
	int i, ret;

	if ((len | offset) & 1)
		return -EINVAL;

	if (ee->magic != CH390_EEPROM_MAGIC)
		return -EINVAL;

	for (i = 0; i < len; i += 2) {
		ret = ch390_eeprom_write(db, (offset + i) / 2, data + i);
		if (ret)
			break;
	}
	return ret;
}

static void ch390_get_pauseparam(struct net_device *ndev, struct ethtool_pauseparam *pause)
{
	struct board_info *db = to_ch390_board(ndev);

	*pause = db->pause;
}

static int ch390_set_pauseparam(struct net_device *ndev, struct ethtool_pauseparam *pause)
{
	struct board_info *db = to_ch390_board(ndev);
	int advertise = 0;

	db->pause = *pause;

	if (pause->autoneg == AUTONEG_DISABLE)
		return ch390_update_fcr(db);

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(4, 20, 0))
	(void)advertise;
	phy_set_sym_pause(db->phydev, pause->rx_pause, pause->tx_pause, pause->autoneg);
#else
	if (pause->rx_pause && pause->tx_pause)
		advertise |= ADVERTISE_PAUSE_CAP;
	else
		advertise &= ~ADVERTISE_PAUSE_CAP;

	if (pause->rx_pause && !pause->tx_pause)
		advertise |= ADVERTISE_PAUSE_ASYM;
	else
		advertise &= ~ADVERTISE_PAUSE_ASYM;

	ch390_mdio_write(db->mdiobus, CH390_PHY_ADDR, MII_ADVERTISE, advertise);
#endif
	phy_start_aneg(db->phydev);
	return 0;
}

static const struct ethtool_ops ch390_ethtool_ops = {
	.get_drvinfo = ch390_get_drvinfo,
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(4, 6, 0))
	.get_link_ksettings = phy_ethtool_get_link_ksettings,
	.set_link_ksettings = phy_ethtool_set_link_ksettings,
#else
	.get_settings = ch390_get_settings,
	.set_settings = ch390_set_settings,
#endif
	.get_msglevel = ch390_get_msglevel,
	.set_msglevel = ch390_set_msglevel,
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(4, 10, 0))
	.nway_reset = phy_ethtool_nway_reset,
#else
	.nway_reset = ch390_nway_reset,
#endif
	.get_link = ethtool_op_get_link,
	.get_eeprom_len = ch390_get_eeprom_len,
	.get_eeprom = ch390_get_eeprom,
	.set_eeprom = ch390_set_eeprom,
	.get_pauseparam = ch390_get_pauseparam,
	.set_pauseparam = ch390_set_pauseparam,
};

static int ch390_all_start(struct board_info *db)
{
	int ret;

	ret = ch390_core_reset(db);
	if (ret)
		return ret;

	/* 
	 * After ch390_core_reset phy must be reopen
	 */
	ret = ch390_set_reg(db, CH390_GPR, 0);
	if (ret)
		return ret;

	msleep(1);

	return ch390_enable_interrupt(db);
}

static int ch390_all_stop(struct board_info *db)
{
	int ret;

	/*
	 * GPR power off of the internal phy,
	 * the internal phy still could be accessed after this GPR power off control
	 */
	ret = ch390_set_reg(db, CH390_GPR, GPR_PHYPD);
	if (ret)
		return ret;

	return ch390_set_reg(db, CH390_RCR, RCR_DIS_CRC);
}

/*
 * fifo reset while rx error found
 */
static int ch390_all_restart(struct board_info *db)
{
	struct net_device *ndev = db->ndev;
	int ret;

	ret = ch390_core_reset(db);
	if (ret)
		return ret;

	/* 
	 * After ch390_core_reset phy must be reopen
	 */
	ret = ch390_set_reg(db, CH390_GPR, 0);
	if (ret)
		return ret;

	msleep(1);

	ret = ch390_enable_interrupt(db);
	if (ret)
		return ret;

	netdev_dbg(ndev, " rxstatus_Er & rxlen_Er %d, RST_c %d\n", db->bc.status_err_counter + db->bc.large_err_counter,
		   db->bc.fifo_rst_counter);

	ret = ch390_set_recv(db);
	if (ret)
		return ret;

	return ch390_set_fcr(db);
}

/*
 * read packets from the fifo memory
 * return value:
 *  > 0 - read packet number, caller can repeat the rx operation
 *    0 - no error, caller need stop further rx operation
 *  -EBUSY - read data error, caller escape from rx operation
 */
static int ch390_loop_rx(struct board_info *db)
{
	struct net_device *ndev = db->ndev;
	u8 rxbyte;
	int ret, rxlen;
	struct sk_buff *skb;
	u8 *rdptr;
	int scanrr = 0;

	do {
		ret = ch390_get_reg(db, OPC_MEM_DMY_R, &rxbyte);
		if (ret)
			return ret;

		ret = ch390_get_reg(db, OPC_MEM_DMY_R, &rxbyte);
		if (ret)
			return ret;

		if (rxbyte & CH390_PKT_ERR) {
			ret = ch390_set_reg(db, CH390_RCR, 0);
			if (ret)
				return ret;

			ret = ch390_set_reg(db, CH390_MPTRCR, 0x01);
			if (ret)
				return ret;

			ret = ch390_set_reg(db, CH390_MRRH, 0x0c);
			if (ret)
				return ret;

			msleep(1);
			ret = ch390_set_reg(db, CH390_RCR, RCR_RXEN);
			if (ret)
				return ret;

			break; /* packet-erro */
		}

		if (rxbyte != CH390_PKT_RDY)
			break; /* exhaust-empty */

		ret = ch390_read_mem(db, OPC_MEM_READ, &db->rxhdr, CH390_RXHDR_SIZE);
		if (ret)
			return ret;

		rxlen = le16_to_cpu(db->rxhdr.rxlen);
		if (db->rxhdr.status & RSR_ERR_BITS || rxlen > CH390_PKT_MAX) {
			netdev_dbg(ndev, "rxhdr-byte (%02x)\n", db->rxhdr.headbyte);

			if (db->rxhdr.status & RSR_ERR_BITS) {
				db->bc.status_err_counter++;
				netdev_dbg(ndev, "check rxstatus-error (%02x)\n", db->rxhdr.status);
			} else {
				db->bc.large_err_counter++;
				netdev_dbg(ndev, "check rxlen large-error (%d > %d)\n", rxlen, CH390_PKT_MAX);
			}

			return ch390_all_restart(db);
		}

		skb = dev_alloc_skb(rxlen);
		if (!skb) {
			ret = ch390_dumpblk(db, OPC_MEM_READ, rxlen);
			if (ret)
				return ret;

			return scanrr;
		}

		rdptr = skb_put(skb, rxlen - 4);
		ret = ch390_read_mem(db, OPC_MEM_READ, rdptr, rxlen);
		if (ret) {
			db->bc.rx_err_counter++;
			dev_kfree_skb(skb);
			return ret;
		}

		skb->protocol = eth_type_trans(skb, db->ndev);
		if (db->ndev->features & NETIF_F_RXCSUM)
			skb_checksum_none_assert(skb);
		netif_rx(skb);
		db->ndev->stats.rx_bytes += rxlen;
		db->ndev->stats.rx_packets++;
		scanrr++;
	} while (!ret);

	return scanrr;
}

/* 
 * transmit a packet
 * return value:
 *   0 - succeed
 *  -ETIMEDOUT - timeout error
 */
static int ch390_single_tx(struct board_info *db, u8 *buff, unsigned int len)
{
	int ret;
	unsigned int temp_low = len & 0xff;
	unsigned int temp_high = (len >> 8) & 0xff;
	u8 val, temp;

	ret = ch390_write_mem(db, OPC_MEM_WRITE, buff, len);
	if (ret)
		return ret;

	do {
		ret = ch390_get_reg(db, CH390_TCR, &temp);
		if (ret)
			return ret;
	} while (temp & TCR_TXREQ);

	ret = ch390_set_reg(db, CH390_TXPLL, temp_low);
	if (ret < 0)
		return ret;

	ret = ch390_set_reg(db, CH390_TXPLH, temp_high);
	if (ret < 0)
		return ret;

	ret = ch390_get_reg(db, CH390_TCR, &val);
	if (ret < 0)
		return ret;

	return ch390_set_reg(db, CH390_TCR, val | TCR_TXREQ);
}

static int ch390_loop_tx(struct board_info *db)
{
	struct net_device *ndev = db->ndev;
	int ntx = 0;
	int ret;

	while (!skb_queue_empty(&db->txq)) {
		struct sk_buff *skb;
		unsigned int len;
		skb = skb_dequeue(&db->txq);
		if (skb) {
			ntx++;
			ret = ch390_single_tx(db, skb->data, skb->len);
			len = skb->len;
			dev_kfree_skb(skb);
			if (ret < 0) {
				db->bc.tx_err_counter++;
				return 0;
			}
			ndev->stats.tx_bytes += len;
			ndev->stats.tx_packets++;
		}

		if (netif_queue_stopped(ndev) && (skb_queue_len(&db->txq) < CH390_TX_QUE_LO_WATER))
			netif_wake_queue(ndev);
	}

	return ntx;
}

static irqreturn_t ch390_rx_threaded_irq(int irq, void *pw)
{
	struct board_info *db = pw;
	int result, result_tx;

	mutex_lock(&db->spi_lockm);

	result = ch390_disable_interrupt(db);
	if (result)
		goto out_unlock;

	result = ch390_clear_interrupt(db);
	if (result)
		goto out_unlock;

	do {
		result = ch390_loop_rx(db); /* threaded irq rx */
		if (result < 0)
			goto out_unlock;

		result_tx = ch390_loop_tx(db); /* more tx better performance */
		if (result_tx < 0)
			goto out_unlock;
	} while (result > 0);

	ch390_enable_interrupt(db);

	/*
	 * To exit and has mutex unlock while rx or tx error
	 */
out_unlock:
	mutex_unlock(&db->spi_lockm);

	return IRQ_HANDLED;
}

static void ch390_tx_delay(struct work_struct *work)
{
	struct board_info *db = container_of(work, struct board_info, tx_work);
	int result;

	mutex_lock(&db->spi_lockm);

	result = ch390_loop_tx(db);
	if (result < 0)
		netdev_err(db->ndev, "transmit packet error\n");

	mutex_unlock(&db->spi_lockm);
}

static void ch390_rxctl_delay(struct work_struct *work)
{
	struct board_info *db = container_of(work, struct board_info, rxctrl_work);
	struct net_device *ndev = db->ndev;
	int result;

	mutex_lock(&db->spi_lockm);

	result = ch390_set_regs(db, CH390_PAR, ndev->dev_addr, sizeof(ndev->dev_addr), TYPE_U8);
	if (result < 0)
		goto out_unlock;

	ch390_set_recv(db);

out_unlock:
	mutex_unlock(&db->spi_lockm);
}

/* 
 * Open network device
 * Called when the network device is marked active, such as a user executing
 * 'ifconfig up' on the device
 */
static int ch390_open(struct net_device *ndev)
{
	struct board_info *db = to_ch390_board(ndev);
	struct spi_device *spi = db->spidev;
	int ret;
	unsigned long flags = IRQ_TYPE_LEVEL_HIGH;

	db->imr_all = IMR_PAR | IMR_PRI;
	db->lcr_all = MLEDCR_LED_MOD1;
	db->rctl.rcr_all = RCR_DIS_CRC | RCR_RXEN;
	memset(db->rctl.hash_table, 0, sizeof(db->rctl.hash_table));

	/* if your platform supports acquire irq number from dts */
#ifdef USE_IRQ_FROM_DTS
	ndev->irq = spi->irq; /* by dts */
#else
	ret = devm_gpio_request(&spi->dev, GPIO_NUMBER, "gpioint");
	if (ret) {
		dev_err(&spi->dev, "gpio_request\n");
		goto out;
	}
	ret = gpio_direction_input(GPIO_NUMBER);
	if (ret) {
		dev_err(&spi->dev, "gpio_direction_input\n");
		goto out;
	}
	irq_set_irq_type(gpio_to_irq(GPIO_NUMBER), flags);

	spi->irq = gpio_to_irq(GPIO_NUMBER);
	ndev->irq = spi->irq;
#endif

	ret = request_threaded_irq(spi->irq, NULL, ch390_rx_threaded_irq, ch390_irq_flag(db) | IRQF_ONESHOT, ndev->name,
				   db);
	if (ret < 0) {
		netdev_err(ndev, "failed to get irq\n");
		return ret;
	}

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(4, 20, 0))
	phy_support_sym_pause(db->phydev);
#else
	db->phydev->supported &= ~SUPPORTED_Asym_Pause;
	db->phydev->advertising &= ~SUPPORTED_Asym_Pause;
	db->phydev->supported |= SUPPORTED_Pause;
	db->phydev->advertising |= SUPPORTED_Pause;
#endif
	phy_start(db->phydev);

	/* flow control parameters init */
	db->pause.rx_pause = true;
	db->pause.tx_pause = true;
	db->pause.autoneg = AUTONEG_DISABLE;

	if (db->phydev->autoneg)
		db->pause.autoneg = AUTONEG_ENABLE;

	ret = ch390_all_start(db);
	if (ret) {
		phy_stop(db->phydev);
		free_irq(spi->irq, db);
		return ret;
	}

	netif_wake_queue(ndev);

out:
	return 0;
}

/*
 * Close network device
 * Called to close down a network device which has been active. Cancel any
 * work, shutdown the RX and TX process and then place the chip into a low
 * power state while it is not being used
 */
static int ch390_stop(struct net_device *ndev)
{
	struct board_info *db = to_ch390_board(ndev);
	int ret;

	ret = ch390_all_stop(db);
	if (ret)
		return ret;

	flush_work(&db->tx_work);
	flush_work(&db->rxctrl_work);

	phy_stop(db->phydev);

	free_irq(db->spidev->irq, db);

	netif_stop_queue(ndev);

	skb_queue_purge(&db->txq);

	return 0;
}

/*
 * event: play a schedule starter in condition
 */
static netdev_tx_t ch390_start_xmit(struct sk_buff *skb, struct net_device *ndev)
{
	struct board_info *db = to_ch390_board(ndev);

	skb_queue_tail(&db->txq, skb);
	if (skb_queue_len(&db->txq) > CH390_TX_QUE_HI_WATER)
		netif_stop_queue(ndev); /* enforce limit queue size */

	schedule_work(&db->tx_work);

	return NETDEV_TX_OK;
}

/*
 * event: play with a schedule starter
 */
static void ch390_set_rx_mode(struct net_device *ndev)
{
	struct board_info *db = to_ch390_board(ndev);
	struct ch390_rxctrl rxctrl;
	struct netdev_hw_addr *ha;
	u8 rcr = RCR_DIS_CRC | RCR_RXEN;
	u32 hash_val;

	memset(&rxctrl, 0, sizeof(rxctrl));

	/* rx control */
	if (ndev->flags & IFF_PROMISC) {
		rcr |= RCR_PRMSC;
		netdev_dbg(ndev, "set_multicast rcr |= RCR_PRMSC, rcr= %02x\n", rcr);
	}

	if (ndev->flags & IFF_ALLMULTI) {
		rcr |= RCR_ALL;
		netdev_dbg(ndev, "set_multicast rcr |= RCR_ALLMULTI, rcr= %02x\n", rcr);
	}

	rxctrl.rcr_all = rcr;

	/* broadcast address */
	rxctrl.hash_table[0] = 0;
	rxctrl.hash_table[1] = 0;
	rxctrl.hash_table[2] = 0;
	rxctrl.hash_table[3] = 0x8000;

	/* the multicast address in Hash Table : 64 bits */
	netdev_for_each_mc_addr(ha, ndev) {
		hash_val = crc32_le(~0, ha->addr, ETH_ALEN) & GENMASK(5, 0);
		rxctrl.hash_table[hash_val / 16] |= BIT(0) << (hash_val % 16);
	}

	/* schedule work to do the actual set of the data if needed */

	if (memcmp(&db->rctl, &rxctrl, sizeof(rxctrl))) {
		memcpy(&db->rctl, &rxctrl, sizeof(rxctrl));
		schedule_work(&db->rxctrl_work);
	}
}

/*
 * event: write into the mac registers and eeprom directly
 */
static int ch390_set_mac_address(struct net_device *ndev, void *p)
{
	struct board_info *db = to_ch390_board(ndev);
	struct sockaddr *addr = p;

	if (!(ndev->priv_flags & IFF_LIVE_ADDR_CHANGE) && netif_running(ndev))
		return -EBUSY;
	if (!is_valid_ether_addr(addr->sa_data))
		return -EADDRNOTAVAIL;

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 9, 0))
	eth_commit_mac_addr_change(ndev, p);
#else
	memcpy(ndev->dev_addr, addr->sa_data, ndev->addr_len);
#endif
	return ch390_set_regs(db, CH390_PAR, ndev->dev_addr, sizeof(ndev->dev_addr), TYPE_U8);
}

static const struct net_device_ops ch390_netdev_ops = {
	.ndo_open = ch390_open,
	.ndo_stop = ch390_stop,
	.ndo_start_xmit = ch390_start_xmit,
	.ndo_set_rx_mode = ch390_set_rx_mode,
	.ndo_validate_addr = eth_validate_addr,
	.ndo_set_mac_address = ch390_set_mac_address,
};

static void ch390_operation_clear(struct board_info *db)
{
	db->bc.status_err_counter = 0;
	db->bc.large_err_counter = 0;
	db->bc.rx_err_counter = 0;
	db->bc.tx_err_counter = 0;
	db->bc.fifo_rst_counter = 0;
}

static int ch390_mdio_register(struct board_info *db)
{
	struct spi_device *spi = db->spidev;
	int ret;

	db->mdiobus = mdiobus_alloc();
	if (!db->mdiobus)
		return -ENOMEM;

	db->mdiobus->priv = db;
	db->mdiobus->read = ch390_mdio_read;
	db->mdiobus->write = ch390_mdio_write;
	db->mdiobus->name = "ch390-mdiobus";
	db->mdiobus->phy_mask = (u32)~BIT(1);
	db->mdiobus->parent = &spi->dev;
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(6, 8, 0))
	snprintf(db->mdiobus->id, MII_BUS_ID_SIZE, "ch390-%s.%u", dev_name(&spi->dev), spi_get_chipselect(spi, 0));
#else
	snprintf(db->mdiobus->id, MII_BUS_ID_SIZE, "ch390-%s.%u", dev_name(&spi->dev), spi->chip_select);
#endif

	ret = mdiobus_register(db->mdiobus);
	if (ret) {
		netdev_err(db->ndev, "can't register MDIO bus\n");
		goto out;
	}

	return 0;
out:
	mdiobus_free(db->mdiobus);
	return ret;
}

static void ch390_mdio_unregister(struct board_info *db)
{
	mdiobus_unregister(db->mdiobus);
	mdiobus_free(db->mdiobus);
}

static void ch390_handle_link_change(struct net_device *ndev)
{
	struct board_info *db = to_ch390_board(ndev);

	/*
	 * only write pause settings to mac. since mac and phy are integrated
	 * together, such as link state, speed and duplex are sync already
	 */
	if (db->phydev->link) {
		if (db->phydev->pause) {
			db->pause.rx_pause = true;
			db->pause.tx_pause = true;
		}
		ch390_update_fcr(db);
	}
}

/*
 * phy connect as poll mode
 */
static int ch390_phy_connect(struct board_info *db)
{
	char phy_id[MII_BUS_ID_SIZE + 3];

	snprintf(phy_id, sizeof(phy_id), PHY_ID_FMT, db->mdiobus->id, CH390_PHY_ADDR);

	db->phydev = phy_connect(db->ndev, phy_id, ch390_handle_link_change, PHY_INTERFACE_MODE_MII);
	if (IS_ERR(db->phydev))
		return PTR_ERR(db->phydev);
	return 0;
}

static int ch390_probe(struct spi_device *spi)
{
	struct device *dev = &spi->dev;
	struct net_device *ndev;
	struct board_info *db;
	int ret = 0;

	ndev = alloc_etherdev(sizeof(*db));
	if (!ndev)
		goto out;

	SET_NETDEV_DEV(ndev, dev);
	dev_set_drvdata(dev, ndev);

	db = netdev_priv(ndev);

	db->msg_enable = 0;
	db->reg_stride = 1;
	db->spidev = spi;
	db->ndev = ndev;

	ndev->netdev_ops = &ch390_netdev_ops;
	ndev->ethtool_ops = &ch390_ethtool_ops;

	mutex_init(&db->spi_lockm);
	mutex_init(&db->reg_mutex);

	INIT_WORK(&db->rxctrl_work, ch390_rxctl_delay);
	INIT_WORK(&db->tx_work, ch390_tx_delay);

	ret = ch390_map_chipid(db);
	if (ret)
		goto out1;

	ret = ch390_map_etherdev_par(ndev, db);
	if (ret < 0)
		goto out1;

	ret = ch390_mdio_register(db);
	if (ret)
		goto out1;

	ret = ch390_phy_connect(db);
	if (ret)
		goto out2;

	ch390_operation_clear(db);
	skb_queue_head_init(&db->txq);

	ret = register_netdev(ndev);
	if (ret) {
		phy_disconnect(db->phydev);
		dev_err(dev, "device register failed: %d\n", ret);
		goto out2;
	}

	return 0;

out2:
	ch390_mdio_unregister(db);
out1:
	free_netdev(ndev);
out:
	return ret;
}

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 18, 0))
static void ch390_drv_remove(struct spi_device *spi)
{
	struct device *dev = &spi->dev;
	struct net_device *ndev = dev_get_drvdata(dev);
	struct board_info *db = to_ch390_board(ndev);

	phy_disconnect(db->phydev);
	unregister_netdev(ndev);
	ch390_mdio_unregister(db);
	free_netdev(ndev);
}
#else
static int ch390_drv_remove(struct spi_device *spi)
{
	struct device *dev = &spi->dev;
	struct net_device *ndev = dev_get_drvdata(dev);
	struct board_info *db = to_ch390_board(ndev);

	phy_disconnect(db->phydev);
	unregister_netdev(ndev);
	ch390_mdio_unregister(db);
	free_netdev(ndev);

	return 0;
}
#endif

static const struct of_device_id ch390_match_table[] = { { .compatible = "ch390_ethernet" }, {} };

static struct spi_driver ch390_driver = {
	.driver = {
		.name = DRVNAME_CH390,
		.of_match_table = ch390_match_table,
	},
	.probe = ch390_probe,
	.remove = ch390_drv_remove,
};
module_spi_driver(ch390_driver);

MODULE_AUTHOR(DRIVER_AUTHOR);
MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_VERSION(VERSION_DESC);
MODULE_LICENSE("GPL");
