/*
 * SPI ethernet driver for SPI to 100Mbps ethernet chip ch390.
 *
 * Copyright (C) 2026 Nanjing Qinheng Microelectronics Co., Ltd.
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
 * V1.2 - add sysfs and fixup rx_irq
 * V1.3 - add sysfs for PHY control and protect PHY ops with mutex
 * V1.3.1 - skip BMCR soft reset write in mdio_write to
 *			prevent unintended PHY reset.
 * V1.4 - add link-up false-link monitor, recovery and stop handling.
 */

// #define DEBUG
// #define VERBOSE

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
#include <linux/slab.h>
#include <linux/skbuff.h>
#include <linux/spi/spi.h>
#include <linux/types.h>
#include <linux/bitops.h>
#include <linux/bitrev.h>
#include <linux/crc32.h>
#include <linux/version.h>

#include "ch390.h"

#define DRVNAME_CH390 "ch390"
#define DRIVER_AUTHOR "WCH"
#define DRIVER_DESC "SPI ethernet driver for ch390, etc."
#define VERSION_DESC "V1.4 On 2026.05"
#define CH390_TX_TIMEOUT_US 750000
#define CH390_TX_POLL_DELAY_US 20
#define CH390_SPI_READ_BUF_SIZE (CH390_PKT_MAX + 1)
#define CH390_RX_READY_BYTES 2
#define CH390_RX_READY_VALID_OFFSET 1
#define CH390_MDIO_C22_REG_MAX 0x1f

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

enum val_type { TYPE_U8, TYPE_U16 };

struct reg_label {
	char *name;
	unsigned int reg;
};

static const char ch390_gstrings[][ETH_GSTRING_LEN] = {
	"set_phypn_triggers", "linkup_restarts", "rx_status_errors",
	"rx_length_errors",   "rx_errors",	 "tx_errors",
	"fifo_resets",
};

static struct reg_label reg_labels[] = {
	REG_LABEL(CH390_NCR),	REG_LABEL(CH390_NSR),	 REG_LABEL(CH390_TCR),
	REG_LABEL(CH390_TSRA),	REG_LABEL(CH390_TSRB),	 REG_LABEL(CH390_RCR),
	REG_LABEL(CH390_RSR),	REG_LABEL(CH390_ROCR),	 REG_LABEL(CH390_BPTR),
	REG_LABEL(CH390_FCTR),	REG_LABEL(CH390_FCR),	 REG_LABEL(CH390_GPR),
	REG_LABEL(CH390_ATCR),	REG_LABEL(CH390_RCSCSR), REG_LABEL(CH390_INTCR),
	REG_LABEL(CH390_ALNCR), REG_LABEL(CH390_ISR),	 REG_LABEL(CH390_IMR),
};

struct ch39x_regs {
	u8 check_phypn_reg;
	u16 check_phypn_mask;
	u8 set_phypn_reg;
	u16 set_phypn_mask;
	u8 check_link_page;
	u8 check_link_reg;
	u16 check_link_mask;
};

static const struct ch39x_regs ch390_pulse_phypn_regs = {
	.check_phypn_reg = CH390_PHYPN_PULSE_STATUS0,
	.check_phypn_mask = CH390_PHYPN_PULSE_STATE,
	.set_phypn_reg = CH390_PHYPN_PULSE_CTL,
	.set_phypn_mask = CH390_PHYPN_PULSE_FORCE_OK,
	.check_link_page = CH390_PHY_PAGE99,
	.check_link_reg = CH390_PHY_LINK_PULSE_STATUS,
	.check_link_mask = CH390_LINK_PULSE_UP,
};

static const struct ch39x_regs ch390_latch_phypn_regs = {
	.check_phypn_reg = CH390_PHY_STATUS0,
	.check_phypn_mask = CH390_POLARITY_STATE,
	.set_phypn_reg = CH390_PHYPN_LATCH_CTL,
	.set_phypn_mask = CH390_PHYPN_LATCH_FORCE_OK,
	.check_link_page = CH390_PHY_PAGE0,
	.check_link_reg = CH390_PHY_STATUS0,
	.check_link_mask = CH390_LINK_LATCH_UP,
};

struct ch39x_ops {
	void (*set_phypn)(struct ch390_priv *dev);
};

enum ch390_flags {
	CH390_DEV_STOPPING,
	CH390_LINK_CHECK_ENABLED,
	CH390_LINKDN_CHECK_PENDING,
	CH390_LINKDN_CHECK_AUTONEG,
	CH390_LINKUP_CHECK_ACTIVE,
};

/*
 * struct ch390_priv - maintain the saved data
 * @spidev: spi device structure
 * @ndev: net device structure
 * @mdiobus: mii bus structure
 * @phydev: phy device structure
 * @txq: tx queue structure
 * @rx_mode_work: Work queue for applying RX mode and multicast lists
 * @tx_work: Work queue for tx packets
 * @tx_timeout_work: Work queue for TX timeout recovery
 * @pause: ethtool pause parameter structure
 * @spi_lockm: serialize MAC data-path and state-change SPI sequences
 * @phy_mutex: serialize CH390 EPCR-backed PHY/EEPROM transactions
 * @reg_mutex: serialize low-level CH390 register/memory SPI transfers
 * @bc: rx control statistics structure
 * @rxhdr: rx header structure
 * @msg_enable: message level value
 * @imr_all: to store operating imr value for register ch390_IMR
 * @lcr_all: to store operating rcr value for register ch390_LMCR
 * @spi_tx_buf: command/dummy buffer for combined SPI reads
 * @spi_rx_buf: response buffer for combined SPI reads
 * @reg_stride: The register address stride. Valid register addresses are a
 * 				multiple of this value. If set to 0, a value of 1 will be
 *              used.
 * @link_name: name of the sysfs symbolic link to be created
 * @flags: driver state bits
 * The saved data variables, keep up to date for retrieval back to use
 */
struct ch390_priv {
	u32 msg_enable;
	struct spi_device *spidev;
	struct net_device *ndev;
	struct mii_bus *mdiobus;
	struct phy_device *phydev;
	struct sk_buff_head txq;
	struct workqueue_struct *wq;
	struct delayed_work linkdn_work;
	struct delayed_work linkup_work;
	struct work_struct rx_mode_work;
	struct work_struct tx_work;
	struct work_struct tx_timeout_work;
	struct ethtool_pauseparam pause;
	struct mutex spi_lockm;
	struct mutex phy_mutex;
	struct mutex reg_mutex;
	struct rx_ctl_mach bc;
	struct ch390_rxhdr rxhdr;
	const struct ch39x_regs *regs;
	const struct ch39x_ops *dev_ops;
	u8 imr_all;
	u8 lcr_all;
	u8 *spi_tx_buf;
	u8 *spi_rx_buf;
	int reg_stride;
	char link_name[32];
	unsigned long flags;
	u64 set_phypn_triggers;
	u64 linkup_restarts;
};

/*
 * Hardware lock order:
 *
 *   spi_lockm -> mdiobus->mdio_lock -> phy_mutex -> reg_mutex
 *
 * Reset and PHY power transitions can disturb the EPCR indirect-access engine,
 * so they wait for both phylib/user MDIO page sequences and direct EPCR users.
 */
static void ch390_lock_epcr_users(struct ch390_priv *dev)
{
	mutex_lock(&dev->mdiobus->mdio_lock);
	mutex_lock(&dev->phy_mutex);
}

static void ch390_unlock_epcr_users(struct ch390_priv *dev)
{
	mutex_unlock(&dev->phy_mutex);
	mutex_unlock(&dev->mdiobus->mdio_lock);
}

static void ch390_set_phypn(struct ch390_priv *dev);
static void ch390_latch_phypn(struct ch390_priv *dev);

static const struct ch39x_ops ch390_pulse_phypn_ops = {
	.set_phypn = ch390_set_phypn,
};

static const struct ch39x_ops ch390_latch_phypn_ops = {
	.set_phypn = ch390_latch_phypn,
};

static int ch390_set_reg_unlocked(struct ch390_priv *dev, u8 reg, u8 val)
{
	struct spi_transfer xfer = {};
	struct spi_message msg;
	int ret;
	u8 cmd_buf[2];

	cmd_buf[0] = (OPC_REG_W | reg);
	cmd_buf[1] = val;

	xfer.tx_buf = cmd_buf;
	xfer.len = sizeof(cmd_buf);

	spi_message_init(&msg);
	spi_message_add_tail(&xfer, &msg);

	ret = spi_sync(dev->spidev, &msg);
	if (ret < 0)
		netif_err(dev, drv, dev->ndev, "%s: error %d set reg %02x\n",
			  __func__, ret, reg);

	return ret;
}

static int ch390_set_reg(struct ch390_priv *dev, u8 reg, u8 val)
{
	int ret;

	mutex_lock(&dev->reg_mutex);
	ret = ch390_set_reg_unlocked(dev, reg, val);
	mutex_unlock(&dev->reg_mutex);

	return ret;
}

static int ch390_get_reg_unlocked(struct ch390_priv *dev, u8 reg, void *val)
{
	struct spi_transfer xfer = {};
	struct spi_message msg;
	int ret;
	u8 tx_buf[2] = { (OPC_REG_R | reg), 0x00 };
	u8 rx_buf[2] = {};

	xfer.tx_buf = tx_buf;
	xfer.rx_buf = rx_buf;
	xfer.len = sizeof(tx_buf);

	spi_message_init(&msg);
	spi_message_add_tail(&xfer, &msg);

	ret = spi_sync(dev->spidev, &msg);
	if (ret < 0)
		netif_err(dev, drv, dev->ndev, "%s: error %d get reg %02x\n",
			  __func__, ret, reg);
	else
		*(u8 *)val = rx_buf[1];

	return ret;
}

static int ch390_get_reg(struct ch390_priv *dev, u8 reg, void *val)
{
	int ret;

	mutex_lock(&dev->reg_mutex);
	ret = ch390_get_reg_unlocked(dev, reg, val);
	mutex_unlock(&dev->reg_mutex);

	return ret;
}

static int ch390_update_bits(struct ch390_priv *dev, u8 reg, u8 mask, u8 val)
{
	int ret;
	u8 current_val;

	mutex_lock(&dev->reg_mutex);

	ret = ch390_get_reg_unlocked(dev, reg, &current_val);
	if (ret < 0) {
		netif_err(dev, drv, dev->ndev,
			  "%s: error %d reading reg %02x\n", __func__, ret,
			  reg);
		goto out_unlock;
	}

	current_val = (current_val & ~mask) | (val & mask);

	ret = ch390_set_reg_unlocked(dev, reg, current_val);
	if (ret < 0) {
		netif_err(dev, drv, dev->ndev,
			  "%s: error %d writing reg %02x\n", __func__, ret,
			  reg);
	}

out_unlock:
	mutex_unlock(&dev->reg_mutex);
	return ret;
}

static int ch390_set_regs(struct ch390_priv *dev, u8 reg, const void *val,
			  size_t val_count, enum val_type type)
{
	int ret = 0;
	int i;

	if (!IS_ALIGNED(reg, dev->reg_stride))
		return -EINVAL;

	mutex_lock(&dev->reg_mutex);

	if (type == TYPE_U16) {
		const u16 *val_words = (const u16 *)val;
		for (i = 0; i < val_count / 2; i++) {
			ret = ch390_set_reg_unlocked(dev, reg + i * 2,
						     val_words[i] & 0xFF);
			if (ret < 0) {
				netif_err(dev, drv, dev->ndev,
					  "%s: error %d writing reg %02x\n",
					  __func__, ret, reg + i * 2);
				break;
			}

			ret = ch390_set_reg_unlocked(dev, reg + i * 2 + 1,
						     (val_words[i] >> 8) &
							     0xFF);
			if (ret < 0) {
				netif_err(dev, drv, dev->ndev,
					  "%s: error %d writing reg %02x\n",
					  __func__, ret, reg + i * 2 + 1);
				break;
			}
		}
	} else {
		const u8 *val_bytes = (const u8 *)val;
		for (i = 0; i < val_count; i++) {
			ret = ch390_set_reg_unlocked(dev, reg + i,
						     val_bytes[i]);
			if (ret < 0) {
				netif_err(dev, drv, dev->ndev,
					  "%s: error %d writing reg %02x\n",
					  __func__, ret, reg + i);
				break;
			}
		}
	}

	mutex_unlock(&dev->reg_mutex);

	return ret;
}

static int ch390_get_regs(struct ch390_priv *dev, u8 reg, void *val,
			  size_t val_count, enum val_type type)
{
	int ret = 0;
	int i;

	if (!IS_ALIGNED(reg, dev->reg_stride))
		return -EINVAL;

	mutex_lock(&dev->reg_mutex);

	if (type == TYPE_U16) {
		u16 *val_words = (u16 *)val;
		for (i = 0; i < val_count / 2; i++) {
			u8 low, high;

			ret = ch390_get_reg_unlocked(dev, reg + i * 2, &low);
			if (ret < 0) {
				netif_err(dev, drv, dev->ndev,
					  "%s: error %d reading reg %02x\n",
					  __func__, ret, reg + i * 2);
				break;
			}

			ret = ch390_get_reg_unlocked(dev, reg + i * 2 + 1,
						     &high);
			if (ret < 0) {
				netif_err(dev, drv, dev->ndev,
					  "%s: error %d reading reg %02x\n",
					  __func__, ret, reg + i * 2 + 1);
				break;
			}

			val_words[i] = (high << 8) | low;
		}
	} else {
		u8 *val_bytes = (u8 *)val;
		for (i = 0; i < val_count; i++) {
			ret = ch390_get_reg_unlocked(dev, reg + i,
						     &val_bytes[i]);
			if (ret < 0) {
				netif_err(dev, drv, dev->ndev,
					  "%s: error %d reading reg %02x\n",
					  __func__, ret, reg + i);
				break;
			}
		}
	}

	mutex_unlock(&dev->reg_mutex);

	return ret;
}

static int ch390_write_mem(struct ch390_priv *dev, const void *buff, size_t len)
{
	struct spi_transfer xfer = {};
	struct spi_message msg;
	int ret;

	xfer.tx_buf = buff;
	xfer.len = len + CH390_TX_OVERHEAD;

	mutex_lock(&dev->reg_mutex);

	spi_message_init(&msg);
	spi_message_add_tail(&xfer, &msg);

	ret = spi_sync(dev->spidev, &msg);
	if (ret < 0)
		netif_err(dev, drv, dev->ndev, "%s: error %d writing!\n",
			  __func__, ret);

	mutex_unlock(&dev->reg_mutex);

	return ret;
}

static int ch390_read_mem(struct ch390_priv *dev, u8 reg, void *buff,
			  size_t len)
{
	struct spi_transfer xfer = {};
	struct spi_message msg;
	int ret;

	mutex_lock(&dev->reg_mutex);

	if (!len) {
		ret = 0;
		goto out_unlock;
	}

	if (!dev->spi_tx_buf || !dev->spi_rx_buf) {
		ret = -ENOMEM;
		goto out_log;
	}

	if (len > CH390_SPI_READ_BUF_SIZE - 1) {
		ret = -EMSGSIZE;
		goto out_log;
	}

	dev->spi_tx_buf[0] = reg;
	memset(dev->spi_tx_buf + 1, 0, len);

	xfer.tx_buf = dev->spi_tx_buf;
	xfer.rx_buf = dev->spi_rx_buf;
	xfer.len = len + 1;

	spi_message_init(&msg);
	spi_message_add_tail(&xfer, &msg);

	ret = spi_sync(dev->spidev, &msg);
	if (!ret && buff) {
		/* The first received byte belongs to the command phase. */
		memcpy(buff, dev->spi_rx_buf + 1, len);
	}

out_log:
	if (ret < 0)
		netif_err(dev, drv, dev->ndev, "%s: error %d get reg %02x\n",
			  __func__, ret, reg);

out_unlock:
	mutex_unlock(&dev->reg_mutex);

	return ret;
}

static int ch390_wait_for_condition(struct ch390_priv *dev, u8 reg, u8 mask,
				    unsigned int timeout_us,
				    unsigned int delay_us)
{
	unsigned int elapsed_us = 0;
	u8 mval;
	int ret;

	while (elapsed_us < timeout_us) {
		ret = ch390_get_reg(dev, reg, &mval);
		if (ret < 0)
			return ret;

		if (!(mval & mask))
			return 0;

		usleep_range(delay_us / 2, delay_us);
		elapsed_us += delay_us;
	}

	return -ETIMEDOUT;
}

static int ch390_epcr_poll(struct ch390_priv *dev)
{
	int ret;

	ret = ch390_wait_for_condition(dev, CH390_EPCR, EPCR_ERRE, 10000, 100);
	if (ret == -ETIMEDOUT)
		netdev_err(dev->ndev, "eeprom/phy in processing get timeout\n");

	return ret;
}

static int ch390_irq_type(struct ch390_priv *dev)
{
	struct spi_device *spi = dev->spidev;
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 11, 0))
	return irq_get_trigger_type(spi->irq);
#else
	struct irq_data *d = irq_get_irq_data(spi->irq);

	return d ? irqd_get_trigger_type(d) : 0;
#endif
}

static int ch390_irq_flag(struct ch390_priv *dev)
{
	int irq_type = ch390_irq_type(dev);

	if (irq_type)
		return irq_type;

	return IRQF_TRIGGER_HIGH;
}

static bool ch390_irq_type_is_level(int irq_type)
{
	return irq_type == IRQF_TRIGGER_LOW || irq_type == IRQF_TRIGGER_HIGH;
}

static unsigned int ch390_intcr_value(struct ch390_priv *dev)
{
	if (ch390_irq_flag(dev) == IRQF_TRIGGER_LOW)
		return INCR_POL_L;

	return INCR_POL_H;
}

static u8 ch390_compute_hash_bit(const u8 *mac)
{
	u32 crc = crc32_le(~0, mac, ETH_ALEN);

	return bitrev32(~crc) >> 26;
}

static void ch390_set_hash_bit(struct ch390_rxctrl *rxctrl, const u8 *mac)
{
	u8 bit = ch390_compute_hash_bit(mac);

	rxctrl->hash_table[bit / 16] |= BIT(bit % 16);
}

static int __ch390_set_rx_mode(struct net_device *ndev)
{
	struct ch390_priv *dev = to_ch390_priv(ndev);
	struct netdev_hw_addr *ha;
	struct ch390_rxctrl rxctrl;
	u8 broadcast[ETH_ALEN];
	int ret;

	netif_addr_lock_bh(ndev);

	memset(&rxctrl, 0, sizeof(rxctrl));
	rxctrl.rcr_all = RCR_DIS_CRC | RCR_RXEN;

	eth_broadcast_addr(broadcast);
	ch390_set_hash_bit(&rxctrl, broadcast);

	if (ndev->flags & IFF_PROMISC) {
		rxctrl.rcr_all |= RCR_PRMSC;
	} else if (ndev->flags & IFF_ALLMULTI) {
		rxctrl.rcr_all |= RCR_ALL;
	} else {
		netdev_for_each_mc_addr(ha, ndev)
			ch390_set_hash_bit(&rxctrl, ha->addr);
	}

	netif_addr_unlock_bh(ndev);

	ret = ch390_set_regs(dev, CH390_MAR, rxctrl.hash_table,
			     sizeof(rxctrl.hash_table), TYPE_U16);
	if (ret < 0)
		return ret;

	return ch390_set_reg(dev, CH390_RCR, rxctrl.rcr_all);
}

static int ch390_core_reset(struct ch390_priv *dev)
{
	int ret;

	dev->bc.fifo_rst_counter++;

	ret = ch390_set_reg(dev, CH390_NCR, NCR_RST); /* NCR reset */
	if (ret < 0)
		return ret;

	ret = ch390_set_reg(dev, CH390_MLEDCR, dev->lcr_all); /* LEDMode1 */
	if (ret < 0)
		return ret;

	return ch390_set_reg(dev, CH390_INTCR, ch390_intcr_value(dev));
}

static int ch390_update_fcr(struct ch390_priv *dev)
{
	u8 fcr = 0;

	if (dev->pause.rx_pause)
		fcr |= FCR_BKPM | FCR_FLCE;
	if (dev->pause.tx_pause)
		fcr |= FCR_TXPEN;

	return ch390_update_bits(dev, CH390_FCR, FCR_RXTX_BITS, fcr);
}

static int ch390_disable_interrupt(struct ch390_priv *dev)
{
	return ch390_set_reg(dev, CH390_IMR, IMR_PAR);
}

static int ch390_enable_interrupt(struct ch390_priv *dev)
{
	return ch390_set_reg(dev, CH390_IMR, dev->imr_all);
}

static int ch390_clear_interrupt(struct ch390_priv *dev)
{
	return ch390_update_bits(dev, CH390_ISR, ISR_CLR_INT, ISR_CLR_INT);
}

static int ch390_eeprom_read(struct ch390_priv *dev, int offset, u8 *val)
{
	int ret;

	mutex_lock(&dev->phy_mutex);

	ret = ch390_set_reg(dev, CH390_EPAR, offset);
	if (ret < 0)
		goto out;

	ret = ch390_set_reg(dev, CH390_EPCR, EPCR_ERPRR);
	if (ret < 0)
		goto out;

	ret = ch390_epcr_poll(dev);
	if (ret < 0)
		goto out;

	ret = ch390_set_reg(dev, CH390_EPCR, 0);
	if (ret < 0)
		goto out;

	ret = ch390_get_regs(dev, CH390_EPDRL, val, 2, TYPE_U8);

out:
	mutex_unlock(&dev->phy_mutex);
	return ret;
}

static int ch390_eeprom_write(struct ch390_priv *dev, int offset, u8 *data)
{
	int ret;

	mutex_lock(&dev->phy_mutex);

	ret = ch390_set_reg(dev, CH390_EPAR, offset);
	if (ret < 0)
		goto out;

	ret = ch390_set_regs(dev, CH390_EPDRL, data, 2, TYPE_U8);
	if (ret < 0)
		goto out;

	ret = ch390_set_reg(dev, CH390_EPCR, EPCR_WEP | EPCR_ERPRW);
	if (ret < 0)
		goto out;

	ret = ch390_epcr_poll(dev);
	if (ret < 0)
		goto out;

	ret = ch390_set_reg(dev, CH390_EPCR, 0);

out:
	mutex_unlock(&dev->phy_mutex);
	return ret;
}

static int ch390_phyread(void *context, u8 reg, unsigned int *val)
{
	struct ch390_priv *dev = context;
	u16 phy_val;
	int ret;

	mutex_lock(&dev->phy_mutex);

	ret = ch390_set_reg(dev, CH390_EPAR, CH390_PHY | reg);
	if (ret < 0)
		goto out;

	ret = ch390_set_reg(dev, CH390_EPCR, EPCR_ERPRR | EPCR_EPOS);
	if (ret < 0)
		goto out;

	ret = ch390_epcr_poll(dev);
	if (ret < 0)
		goto out;

	ret = ch390_set_reg(dev, CH390_EPCR, 0);
	if (ret < 0)
		goto out;

	phy_val = 0;

	ret = ch390_get_regs(dev, CH390_EPDRL, &phy_val, sizeof(phy_val),
			     TYPE_U16);
	if (!ret)
		*val = phy_val;

out:
	mutex_unlock(&dev->phy_mutex);
	return ret;
}

static int ch390_phywrite(void *context, u8 reg, u16 val)
{
	struct ch390_priv *dev = context;
	int ret;

	mutex_lock(&dev->phy_mutex);

	ret = ch390_set_reg(dev, CH390_EPAR, CH390_PHY | reg);
	if (ret < 0)
		goto out;

	ret = ch390_set_regs(dev, CH390_EPDRL, &val, 2, TYPE_U16);
	if (ret < 0)
		goto out;

	ret = ch390_set_reg(dev, CH390_EPCR, EPCR_EPOS | EPCR_ERPRW);
	if (ret < 0)
		goto out;

	ret = ch390_epcr_poll(dev);
	if (ret < 0)
		goto out;

	ret = ch390_set_reg(dev, CH390_EPCR, 0);

out:
	mutex_unlock(&dev->phy_mutex);
	return ret;
}

static int ch390_mdio_read(struct mii_bus *bus, int addr, int regnum)
{
	struct ch390_priv *dev = bus->priv;
	unsigned int val = 0xffff;
	int ret;

	if (addr == CH390_PHY_ADDR) {
		if (regnum < 0 || regnum > CH390_MDIO_C22_REG_MAX)
			return -EINVAL;

		ret = ch390_phyread(dev, (u8)regnum, &val);
		if (ret < 0)
			return ret;
	}

	return val;
}

static int ch390_mdio_write(struct mii_bus *bus, int addr, int regnum, u16 val)
{
	struct ch390_priv *dev = bus->priv;

	if (addr != CH390_PHY_ADDR)
		return -ENODEV;

	if (regnum < 0 || regnum > CH390_MDIO_C22_REG_MAX)
		return -EINVAL;

	if ((regnum == MII_BMCR) && (val & BMCR_RESET))
		return 0;

	return ch390_phywrite(dev, (u8)regnum, val);
}

static int ch390_page_read(struct ch390_priv *dev, int page, int regnum)
{
	struct mii_bus *bus = dev->mdiobus;
	int ret;

	mutex_lock(&bus->mdio_lock);

	ret = ch390_mdio_write(bus, CH390_PHY_ADDR, CH390_PHY_PAG_SEL, page);
	if (ret < 0)
		goto out_unlock;

	ret = ch390_mdio_read(bus, CH390_PHY_ADDR, regnum);
	if (ret < 0)
		netdev_err(dev->ndev, "error %d reading PHY page %d reg %d\n",
			   ret, page, regnum);

	ch390_mdio_write(bus, CH390_PHY_ADDR, CH390_PHY_PAG_SEL,
			 CH390_PHY_PAGE0);

out_unlock:
	mutex_unlock(&bus->mdio_lock);
	return ret;
}

static int ch390_map_chipid(struct ch390_priv *dev)
{
	struct device *device = &dev->spidev->dev;
	unsigned short wid;
	u8 buff[6];
	int ret;

	ret = ch390_get_regs(dev, CH390_VIDL, buff, sizeof(buff), TYPE_U8);
	if (ret < 0)
		return ret;

	wid = get_unaligned_le16(buff + 2);
	if (wid != CH390_ID) {
		dev_err(device, "chipid error as %04x !\n", wid);
		return -ENODEV;
	}

	dev_info(device, "chip %04x found\n", wid);
	return 0;
}

/*
 * Read ch390_PAR registers which is the mac address loaded from EEPROM while power-on
 */
static int ch390_map_etherdev_par(struct net_device *ndev,
				  struct ch390_priv *dev)
{
	u8 addr[ETH_ALEN];
	int ret;

	ret = ch390_get_regs(dev, CH390_PAR, addr, sizeof(addr), TYPE_U8);
	if (ret < 0)
		return ret;

	if (!is_valid_ether_addr(addr)) {
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 4, 0))
		eth_hw_addr_random(ndev);
#else
		random_ether_addr(ndev->dev_addr);
#endif

		ret = ch390_set_regs(dev, CH390_PAR, ndev->dev_addr, ETH_ALEN,
				     TYPE_U8);
		if (ret < 0)
			return ret;

		dev_dbg(&dev->spidev->dev, "Use random MAC address\n");
		return 0;
	}

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 16, 0))
	eth_hw_addr_set(ndev, addr);
#else
	memcpy(ndev->dev_addr, addr, ETH_ALEN);
#endif

	return 0;
}

/*
 * ethtool-ops
 */
static void ch390_get_drvinfo(struct net_device *ndev,
			      struct ethtool_drvinfo *info)
{
#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 1, 0)
	strscpy(info->driver, DRVNAME_CH390, sizeof(info->driver));
	strscpy(info->fw_version, VERSION_DESC, sizeof(info->fw_version));
#else
	strlcpy(info->driver, DRVNAME_CH390, sizeof(info->driver));
	strlcpy(info->fw_version, VERSION_DESC, sizeof(info->fw_version));
#endif
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
	struct ch390_priv *dev = to_ch390_priv(ndev);

	dev->msg_enable = value;
}

static u32 ch390_get_msglevel(struct net_device *ndev)
{
	struct ch390_priv *dev = to_ch390_priv(ndev);

	return dev->msg_enable;
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

static int ch390_get_eeprom(struct net_device *ndev, struct ethtool_eeprom *ee,
			    u8 *data)
{
	struct ch390_priv *dev = to_ch390_priv(ndev);
	int offset = ee->offset;
	int len = ee->len;
	int i, ret = 0;

	if ((len | offset) & 1)
		return -EINVAL;

	ee->magic = CH390_EEPROM_MAGIC;

	for (i = 0; i < len; i += 2) {
		ret = ch390_eeprom_read(dev, (offset + i) / 2, data + i);
		if (ret < 0)
			break;
	}
	return ret;
}

static int ch390_set_eeprom(struct net_device *ndev, struct ethtool_eeprom *ee,
			    u8 *data)
{
	struct ch390_priv *dev = to_ch390_priv(ndev);
	int offset = ee->offset;
	int len = ee->len;
	int i, ret = 0;

	if ((len | offset) & 1)
		return -EINVAL;

	if (ee->magic != CH390_EEPROM_MAGIC)
		return -EINVAL;

	for (i = 0; i < len; i += 2) {
		ret = ch390_eeprom_write(dev, (offset + i) / 2, data + i);
		if (ret < 0)
			break;
	}
	return ret;
}

static void ch390_get_pauseparam(struct net_device *ndev,
				 struct ethtool_pauseparam *pause)
{
	struct ch390_priv *dev = to_ch390_priv(ndev);

	*pause = dev->pause;
}

static int ch390_set_pauseparam(struct net_device *ndev,
				struct ethtool_pauseparam *pause)
{
	struct ch390_priv *dev = to_ch390_priv(ndev);
	int advertise = 0;
	int ret;

	dev->pause = *pause;

	if (pause->autoneg == AUTONEG_DISABLE) {
		mutex_lock(&dev->spi_lockm);
		ret = ch390_update_fcr(dev);
		mutex_unlock(&dev->spi_lockm);
		return ret;
	}

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(4, 20, 0))
	(void)advertise;
	phy_set_sym_pause(dev->phydev, pause->rx_pause, pause->tx_pause,
			  pause->autoneg);
#else
	if (pause->rx_pause && pause->tx_pause)
		advertise |= ADVERTISE_PAUSE_CAP;
	else
		advertise &= ~ADVERTISE_PAUSE_CAP;

	if (pause->rx_pause && !pause->tx_pause)
		advertise |= ADVERTISE_PAUSE_ASYM;
	else
		advertise &= ~ADVERTISE_PAUSE_ASYM;

	mdiobus_write(dev->mdiobus, CH390_PHY_ADDR, MII_ADVERTISE, advertise);
#endif
	phy_start_aneg(dev->phydev);
	return 0;
}

static void ch390_get_strings(struct net_device *ndev, u32 stringset, u8 *data)
{
	if (stringset == ETH_SS_STATS)
		memcpy(data, ch390_gstrings, sizeof(ch390_gstrings));
}

static int ch390_get_sset_count(struct net_device *ndev, int sset)
{
	switch (sset) {
	case ETH_SS_STATS:
		return ARRAY_SIZE(ch390_gstrings);
	default:
		return -EOPNOTSUPP;
	}
}

static void ch390_get_ethtool_stats(struct net_device *ndev,
				    struct ethtool_stats *stats, u64 *data)
{
	struct ch390_priv *dev = to_ch390_priv(ndev);

	data[0] = dev->set_phypn_triggers;
	data[1] = dev->linkup_restarts;
	data[2] = dev->bc.status_err_counter;
	data[3] = dev->bc.large_err_counter;
	data[4] = dev->bc.rx_err_counter;
	data[5] = dev->bc.tx_err_counter;
	data[6] = dev->bc.fifo_rst_counter;
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
	.get_strings = ch390_get_strings,
	.get_sset_count = ch390_get_sset_count,
	.get_ethtool_stats = ch390_get_ethtool_stats,
};

static void ch390_enable_pwrsave(struct ch390_priv *dev)
{
	struct mii_bus *bus = dev->mdiobus;
	int val, ret;

	mutex_lock(&bus->mdio_lock);

	ret = ch390_mdio_write(bus, CH390_PHY_ADDR, CH390_PHY_PAG_SEL,
			       CH390_PHY_PAGE0);
	if (ret < 0)
		goto out_unlock;

	val = ch390_mdio_read(bus, CH390_PHY_ADDR, CH390_PHY_PWR_SAVE);
	if (val < 0) {
		ret = val;
		goto out_unlock;
	}

	val |= CH390_PHY_ENPWR_SAVE;
	ret = ch390_mdio_write(bus, CH390_PHY_ADDR, CH390_PHY_PWR_SAVE, val);

out_unlock:
	mutex_unlock(&bus->mdio_lock);
	if (ret < 0)
		netdev_err(dev->ndev, "failed to enable PHY power save: %d\n",
			   ret);
}

static int ch390_all_start(struct ch390_priv *dev, bool enable_interrupt)
{
	int ret;

	ch390_lock_epcr_users(dev);

	ret = ch390_core_reset(dev);
	if (ret < 0)
		goto out_epcr_unlock;

	ret = ch390_disable_interrupt(dev);
	if (ret < 0)
		goto out_epcr_unlock;

	/*
	 * After ch390_core_reset phy must be reopen
	 */
	ret = ch390_set_reg(dev, CH390_GPR, 0);
	if (ret < 0)
		goto out_epcr_unlock;

	msleep(1);

out_epcr_unlock:
	ch390_unlock_epcr_users(dev);
	if (ret < 0)
		return ret;

	ch390_enable_pwrsave(dev);

	ret = ch390_update_fcr(dev);
	if (ret < 0)
		return ret;

	ret = ch390_clear_interrupt(dev);
	if (ret < 0)
		return ret;

	ret = __ch390_set_rx_mode(dev->ndev);
	if (ret < 0)
		return ret;

	if (enable_interrupt)
		return ch390_enable_interrupt(dev);

	return 0;
}

static int ch390_all_restart(struct ch390_priv *dev)
{
	return ch390_all_start(dev, true);
}

static int ch390_all_stop(struct ch390_priv *dev)
{
	int ret;

	ch390_lock_epcr_users(dev);

	/*
	 * GPR power off of the internal phy,
	 * the internal phy still could be accessed after this GPR power off control
	 */
	ret = ch390_set_reg(dev, CH390_GPR, GPR_PHYPD);
	if (ret < 0)
		goto out_unlock;

	ret = ch390_set_reg(dev, CH390_RCR, RCR_DIS_CRC);

out_unlock:
	ch390_unlock_epcr_users(dev);
	return ret;
}

static int ch390_reset_rx_fifo(struct ch390_priv *dev)
{
	int ret;

	ret = ch390_set_reg(dev, CH390_RCR, 0);
	if (ret < 0)
		return ret;

	ret = ch390_set_reg(dev, CH390_MPTRCR, 0x01);
	if (ret < 0)
		return ret;

	ret = ch390_set_reg(dev, CH390_MRRH, 0x0c);
	if (ret < 0)
		return ret;

	msleep(1);

	return __ch390_set_rx_mode(dev->ndev);
}

static int ch390_read_rx_ready(struct ch390_priv *dev, u8 *rxbyte)
{
	u8 ready[CH390_RX_READY_BYTES];
	int ret;

	ret = ch390_read_mem(dev, OPC_MEM_DMY_R, ready, sizeof(ready));
	if (ret < 0)
		return ret;

	*rxbyte = ready[CH390_RX_READY_VALID_OFFSET];

	return 0;
}

/*
 * read packets from the fifo memory
 * return value:
 *    0 - rx fifo drained or stop requested
 *   <0 - read data error, caller escape from rx operation
 */
static int ch390_loop_rx(struct ch390_priv *dev)
{
	struct net_device *ndev = dev->ndev;
	u8 rxbyte;
	int ret, rxlen;
	struct sk_buff *skb;
	u8 *rdptr;

	while (!test_bit(CH390_DEV_STOPPING, &dev->flags)) {
		ret = ch390_read_rx_ready(dev, &rxbyte);
		if (ret < 0)
			return ret;

		if (rxbyte & CH390_PKT_ERR) {
			ret = ch390_reset_rx_fifo(dev);
			if (ret < 0)
				return ret;

			return 0; /* packet-erro */
		}

		if (rxbyte != CH390_PKT_RDY)
			break; /* exhaust-empty */

		ret = ch390_read_mem(dev, OPC_MEM_READ, &dev->rxhdr,
				     CH390_RXHDR_SIZE);
		if (ret < 0) {
			ch390_reset_rx_fifo(dev);
			return ret;
		}

		rxlen = le16_to_cpu(dev->rxhdr.rxlen);

		if (rxlen < 4 || rxlen > CH390_PKT_MAX) {
			netdev_dbg(ndev, "rxhdr-byte (%02x)\n",
				   dev->rxhdr.headbyte);

			dev->bc.large_err_counter++;
			netdev_dbg(ndev, "check rxlen-error (%d)\n", rxlen);

			ret = ch390_reset_rx_fifo(dev);
			if (ret < 0)
				return ret;

			return 0;
		}

		if (dev->rxhdr.status & RSR_ERR_BITS) {
			netdev_dbg(ndev, "rxhdr-byte (%02x)\n",
				   dev->rxhdr.headbyte);
			dev->bc.status_err_counter++;
			netdev_dbg(ndev, "check rxstatus-error (%02x)\n",
				   dev->rxhdr.status);

			ret = ch390_read_mem(dev, OPC_MEM_READ, NULL, rxlen);
			if (ret < 0) {
				ch390_reset_rx_fifo(dev);
				return ret;
			}

			continue;
		}

		skb = netdev_alloc_skb_ip_align(ndev, rxlen);
		if (!skb) {
			ret = ch390_read_mem(dev, OPC_MEM_READ, NULL, rxlen);
			if (ret < 0) {
				ch390_reset_rx_fifo(dev);
				return ret;
			}

			continue;
		}

		/* Read the appended CRC to advance FIFO, then trim it away. */
		rdptr = skb_put(skb, rxlen);

		ret = ch390_read_mem(dev, OPC_MEM_READ, rdptr, rxlen);
		if (ret < 0) {
			dev->bc.rx_err_counter++;
			dev_kfree_skb(skb);
			ch390_reset_rx_fifo(dev);
			return ret;
		}

		skb_trim(skb, rxlen - 4);
		skb->protocol = eth_type_trans(skb, dev->ndev);
		if (dev->ndev->features & NETIF_F_RXCSUM)
			skb_checksum_none_assert(skb);
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 18, 0))
		netif_rx(skb);
#else
		netif_rx_ni(skb);
#endif
		dev->ndev->stats.rx_bytes += rxlen - 4;
		dev->ndev->stats.rx_packets++;
	}

	return 0;
}

static int ch390_single_tx(struct ch390_priv *dev, u8 *buff, unsigned int len)
{
	int ret;
	unsigned int temp_low = len & 0xff;
	unsigned int temp_high = (len >> 8) & 0xff;

	ret = ch390_write_mem(dev, buff, len);
	if (ret < 0)
		return ret;

	ret = ch390_wait_for_condition(dev, CH390_TCR, TCR_TXREQ,
				       CH390_TX_TIMEOUT_US,
				       CH390_TX_POLL_DELAY_US);
	if (ret < 0)
		return ret;

	ret = ch390_set_reg(dev, CH390_TXPLL, temp_low);
	if (ret < 0)
		return ret;

	ret = ch390_set_reg(dev, CH390_TXPLH, temp_high);
	if (ret < 0)
		return ret;

	return ch390_set_reg(dev, CH390_TCR, TCR_TXREQ);
}

static int ch390_loop_tx(struct ch390_priv *dev)
{
	struct net_device *ndev = dev->ndev;
	struct sk_buff *skb;
	int ntx = 0;
	int ret;

	if (test_bit(CH390_DEV_STOPPING, &dev->flags))
		return 0;

	if (!netif_carrier_ok(ndev)) {
		while ((skb = skb_dequeue(&dev->txq))) {
			dev_kfree_skb(skb);
			ndev->stats.tx_dropped++;
		}

		return 0;
	}

	while ((skb = skb_dequeue(&dev->txq))) {
		unsigned int len;

		len = skb->len;

		if (skb_cow_head(skb, CH390_TX_OVERHEAD)) {
			ntx++;
			dev_kfree_skb(skb);
			dev->bc.tx_err_counter++;
			ndev->stats.tx_errors++;
			ndev->stats.tx_dropped++;
			if (netif_queue_stopped(ndev) &&
			    skb_queue_len(&dev->txq) < CH390_TX_QUE_LO_WATER)
				netif_wake_queue(ndev);
			return ntx;
		}

		if (test_bit(CH390_DEV_STOPPING, &dev->flags) ||
		    !netif_carrier_ok(ndev)) {
			skb_queue_head(&dev->txq, skb);
			return ntx;
		}

		ntx++;
		skb_push(skb, CH390_TX_OVERHEAD);
		skb->data[0] = OPC_MEM_WRITE;
		ret = ch390_single_tx(dev, skb->data, len);
		dev_kfree_skb(skb);

		if (ret < 0) {
			dev->bc.tx_err_counter++;
			ndev->stats.tx_errors++;
			ndev->stats.tx_dropped++;
			netif_stop_queue(ndev);
			return ret;
		}

		/* start_xmit only queues the skb; refresh watchdog on real SPI TX. */
		netif_trans_update(ndev);
		ndev->stats.tx_bytes += len;
		ndev->stats.tx_packets++;
		if (netif_queue_stopped(ndev) &&
		    skb_queue_len(&dev->txq) < CH390_TX_QUE_LO_WATER)
			netif_wake_queue(ndev);
	}

	return ntx;
}

static irqreturn_t ch390_rx_threaded_irq(int irq, void *pw)
{
	struct ch390_priv *dev = pw;
	bool tx_pending;
	int ret;

	mutex_lock(&dev->spi_lockm);

	if (test_bit(CH390_DEV_STOPPING, &dev->flags))
		goto out_unlock;

	ret = ch390_clear_interrupt(dev);
	if (ret < 0)
		goto out_unlock;

	ret = ch390_loop_rx(dev); /* threaded irq rx */
	if (ret < 0)
		goto out_unlock;

	/*
	 * To exit and has mutex unlock while rx error
	 */
out_unlock:
	tx_pending = !skb_queue_empty(&dev->txq);
	mutex_unlock(&dev->spi_lockm);

	if (tx_pending)
		queue_work(dev->wq, &dev->tx_work);

	return IRQ_HANDLED;
}

static void ch390_tx_delay(struct work_struct *work)
{
	struct ch390_priv *dev = container_of(work, struct ch390_priv, tx_work);
	bool tx_pending;
	int ret;

	mutex_lock(&dev->spi_lockm);

	if (test_bit(CH390_DEV_STOPPING, &dev->flags)) {
		mutex_unlock(&dev->spi_lockm);
		return;
	}

	ret = ch390_loop_tx(dev);
	if (ret < 0)
		netdev_err(dev->ndev, "transmit packet error\n");
	tx_pending = ret >= 0 && !skb_queue_empty(&dev->txq);

	mutex_unlock(&dev->spi_lockm);

	if (tx_pending)
		queue_work(dev->wq, &dev->tx_work);
}

static void ch390_tx_timeout_work(struct work_struct *work)
{
	struct ch390_priv *dev =
		container_of(work, struct ch390_priv, tx_timeout_work);
	bool tx_pending = false;
	int ret;

	mutex_lock(&dev->spi_lockm);

	if (test_bit(CH390_DEV_STOPPING, &dev->flags))
		goto out_unlock;

	ret = ch390_all_restart(dev);
	if (ret < 0) {
		netdev_err(dev->ndev, "failed to recover TX timeout: %d\n",
			   ret);
		goto out_unlock;
	}

	if (netif_running(dev->ndev) && netif_carrier_ok(dev->ndev) &&
	    netif_queue_stopped(dev->ndev))
		netif_wake_queue(dev->ndev);
	tx_pending = !skb_queue_empty(&dev->txq);

out_unlock:
	mutex_unlock(&dev->spi_lockm);

	if (tx_pending)
		queue_work(dev->wq, &dev->tx_work);
}

static void ch390_rx_mode_delay(struct work_struct *work)
{
	struct ch390_priv *dev =
		container_of(work, struct ch390_priv, rx_mode_work);
	int ret;

	mutex_lock(&dev->spi_lockm);

	if (test_bit(CH390_DEV_STOPPING, &dev->flags))
		goto out_unlock;

	ret = __ch390_set_rx_mode(dev->ndev);
	if (ret < 0)
		netdev_err(dev->ndev, "failed to apply rx mode: %d\n", ret);

out_unlock:
	mutex_unlock(&dev->spi_lockm);
}

static int ch390_request_irq(struct ch390_priv *dev);

/*
 * Open network device
 * Called when the network device is marked active, such as a user executing
 * 'ifconfig up' on the device
 */
static int ch390_open(struct net_device *ndev)
{
	struct ch390_priv *dev = to_ch390_priv(ndev);
	int ret;

	netif_stop_queue(ndev);
	netif_carrier_off(ndev);
	clear_bit(CH390_DEV_STOPPING, &dev->flags);

	dev->imr_all = IMR_PAR | IMR_PRI;
	dev->lcr_all = MLEDCR_LED_MOD1;

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(4, 20, 0))
	phy_support_sym_pause(dev->phydev);
#else
	dev->phydev->supported &= ~SUPPORTED_Asym_Pause;
	dev->phydev->advertising &= ~SUPPORTED_Asym_Pause;
	dev->phydev->supported |= SUPPORTED_Pause;
	dev->phydev->advertising |= SUPPORTED_Pause;
#endif

	/* flow control parameters init */
	dev->pause.rx_pause = true;
	dev->pause.tx_pause = true;
	dev->pause.autoneg = AUTONEG_DISABLE;

	if (dev->phydev->autoneg)
		dev->pause.autoneg = AUTONEG_ENABLE;

	mutex_lock(&dev->spi_lockm);
	ret = ch390_all_start(dev, false);
	mutex_unlock(&dev->spi_lockm);
	if (ret < 0) {
		set_bit(CH390_DEV_STOPPING, &dev->flags);
		return ret;
	}

	ret = ch390_request_irq(dev);
	if (ret < 0) {
		mutex_lock(&dev->spi_lockm);
		ch390_all_stop(dev);
		mutex_unlock(&dev->spi_lockm);
		set_bit(CH390_DEV_STOPPING, &dev->flags);
		return ret;
	}

	mutex_lock(&dev->spi_lockm);
	ret = ch390_enable_interrupt(dev);
	mutex_unlock(&dev->spi_lockm);
	if (ret < 0) {
		free_irq(dev->spidev->irq, dev);
		mutex_lock(&dev->spi_lockm);
		ch390_all_stop(dev);
		mutex_unlock(&dev->spi_lockm);
		set_bit(CH390_DEV_STOPPING, &dev->flags);
		return ret;
	}

	phy_start(dev->phydev);

	clear_bit(CH390_LINKDN_CHECK_PENDING, &dev->flags);
	clear_bit(CH390_LINKDN_CHECK_AUTONEG, &dev->flags);
	clear_bit(CH390_LINKUP_CHECK_ACTIVE, &dev->flags);
	set_bit(CH390_LINK_CHECK_ENABLED, &dev->flags);
	mod_delayed_work(dev->wq, &dev->linkdn_work,
			 msecs_to_jiffies(CH390_LINK_DOWN_INTVAL_MS));

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
	struct ch390_priv *dev = to_ch390_priv(ndev);
	int ret, stop_ret;

	netif_stop_queue(ndev);
	netif_carrier_off(ndev);
	set_bit(CH390_DEV_STOPPING, &dev->flags);
	clear_bit(CH390_LINK_CHECK_ENABLED, &dev->flags);
	clear_bit(CH390_LINKDN_CHECK_PENDING, &dev->flags);
	clear_bit(CH390_LINKDN_CHECK_AUTONEG, &dev->flags);
	clear_bit(CH390_LINKUP_CHECK_ACTIVE, &dev->flags);
	cancel_delayed_work_sync(&dev->linkdn_work);
	cancel_delayed_work_sync(&dev->linkup_work);

	mutex_lock(&dev->spi_lockm);
	ret = ch390_disable_interrupt(dev);
	stop_ret = ch390_all_stop(dev);
	mutex_unlock(&dev->spi_lockm);

	free_irq(dev->spidev->irq, dev);

	cancel_work_sync(&dev->tx_work);
	cancel_work_sync(&dev->tx_timeout_work);
	cancel_work_sync(&dev->rx_mode_work);

	phy_stop(dev->phydev);

	skb_queue_purge(&dev->txq);

	if (ret < 0)
		netdev_warn(ndev,
			    "failed to disable interrupts during stop: %d\n",
			    ret);
	if (stop_ret < 0)
		netdev_warn(ndev, "failed to stop hardware: %d\n", stop_ret);

	return 0;
}

/*
 * event: play a schedule starter in condition
 */
static netdev_tx_t ch390_start_xmit(struct sk_buff *skb,
				    struct net_device *ndev)
{
	struct ch390_priv *dev = to_ch390_priv(ndev);

	if (unlikely(test_bit(CH390_DEV_STOPPING, &dev->flags))) {
		dev_kfree_skb(skb);
		ndev->stats.tx_dropped++;
		return NETDEV_TX_OK;
	}

	skb_queue_tail(&dev->txq, skb);
	if (skb_queue_len(&dev->txq) > CH390_TX_QUE_HI_WATER)
		netif_stop_queue(ndev); /* enforce limit queue size */

	queue_work(dev->wq, &dev->tx_work);

	return NETDEV_TX_OK;
}

/*
 * event: play with a schedule starter
 */
static void ch390_set_rx_mode(struct net_device *ndev)
{
	struct ch390_priv *dev = to_ch390_priv(ndev);

	if (!test_bit(CH390_DEV_STOPPING, &dev->flags))
		queue_work(dev->wq, &dev->rx_mode_work);
}

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 6, 0))
static void ch390_tx_timeout(struct net_device *ndev, unsigned int txqueue)
#else
static void ch390_tx_timeout(struct net_device *ndev)
#endif
{
	struct ch390_priv *dev = to_ch390_priv(ndev);

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 6, 0))
	(void)txqueue;
#endif

	dev->bc.tx_err_counter++;
	ndev->stats.tx_errors++;

	if (!test_bit(CH390_DEV_STOPPING, &dev->flags))
		queue_work(dev->wq, &dev->tx_timeout_work);
}

/*
 * event: write into the mac registers and eeprom directly
 */
static int ch390_set_mac_address(struct net_device *ndev, void *p)
{
	struct ch390_priv *dev = to_ch390_priv(ndev);
	struct sockaddr *addr = p;
	int ret;

	if (!(ndev->priv_flags & IFF_LIVE_ADDR_CHANGE) && netif_running(ndev))
		return -EBUSY;
	if (!is_valid_ether_addr(addr->sa_data))
		return -EADDRNOTAVAIL;

	mutex_lock(&dev->spi_lockm);
	ret = ch390_set_regs(dev, CH390_PAR, addr->sa_data, ETH_ALEN, TYPE_U8);
	mutex_unlock(&dev->spi_lockm);
	if (ret < 0)
		return ret;

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 9, 0))
	eth_commit_mac_addr_change(ndev, p);
#else
	memcpy(ndev->dev_addr, addr->sa_data, ndev->addr_len);
#endif
	return 0;
}

static int ch390_do_ioctl(struct net_device *ndev, struct ifreq *ifr, int cmd)
{
	struct ch390_priv *priv = netdev_priv(ndev);
	struct mii_ioctl_data *mii = if_mii(ifr);
	int ret;

	if (!priv->mdiobus)
		return -ENODEV;

	switch (cmd) {
	case SIOCGMIIPHY:
		mii->phy_id = CH390_PHY_ADDR;
		return 0;

	case SIOCGMIIREG:
		ret = mdiobus_read(priv->mdiobus, mii->phy_id, mii->reg_num);
		if (ret < 0)
			return ret;
		mii->val_out = ret;
		return 0;

	case SIOCSMIIREG:
		return mdiobus_write(priv->mdiobus, mii->phy_id, mii->reg_num,
				     mii->val_in);

	default:
		return -EOPNOTSUPP;
	}
}

static const struct net_device_ops ch390_netdev_ops = {
	.ndo_open = ch390_open,
	.ndo_stop = ch390_stop,
	.ndo_start_xmit = ch390_start_xmit,
	.ndo_tx_timeout = ch390_tx_timeout,
	.ndo_set_rx_mode = ch390_set_rx_mode,
	.ndo_validate_addr = eth_validate_addr,
	.ndo_set_mac_address = ch390_set_mac_address,
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 15, 0))
	.ndo_eth_ioctl = ch390_do_ioctl,
#else
	.ndo_do_ioctl = ch390_do_ioctl,
#endif
};

static void ch390_operation_clear(struct ch390_priv *dev)
{
	dev->bc.status_err_counter = 0;
	dev->bc.large_err_counter = 0;
	dev->bc.rx_err_counter = 0;
	dev->bc.tx_err_counter = 0;
	dev->bc.fifo_rst_counter = 0;
}

static int ch390_mdio_register(struct ch390_priv *dev)
{
	struct spi_device *spi = dev->spidev;
	int ret;

	dev->mdiobus = mdiobus_alloc();
	if (!dev->mdiobus)
		return -ENOMEM;

	dev->mdiobus->priv = dev;
	dev->mdiobus->read = ch390_mdio_read;
	dev->mdiobus->write = ch390_mdio_write;
	dev->mdiobus->name = "ch390-mdiobus";
	dev->mdiobus->phy_mask = (u32)~BIT(1);
	dev->mdiobus->parent = &spi->dev;
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(6, 8, 0))
	snprintf(dev->mdiobus->id, MII_BUS_ID_SIZE, "ch390-%s.%u",
		 dev_name(&spi->dev), spi_get_chipselect(spi, 0));
#else
	snprintf(dev->mdiobus->id, MII_BUS_ID_SIZE, "ch390-%s.%u",
		 dev_name(&spi->dev), spi->chip_select);
#endif

	ret = mdiobus_register(dev->mdiobus);
	if (ret) {
		netdev_err(dev->ndev, "can't register MDIO bus\n");
		goto out;
	}

	return 0;
out:
	mdiobus_free(dev->mdiobus);
	return ret;
}

static void ch390_mdio_unregister(struct ch390_priv *dev)
{
	mdiobus_unregister(dev->mdiobus);
	mdiobus_free(dev->mdiobus);
}

static void ch390_handle_link_change(struct net_device *ndev)
{
	struct ch390_priv *dev = to_ch390_priv(ndev);

	phy_print_status(dev->phydev);

	if (test_bit(CH390_DEV_STOPPING, &dev->flags) || !netif_running(ndev)) {
		netif_stop_queue(ndev);
		netif_carrier_off(ndev);
		clear_bit(CH390_LINKDN_CHECK_PENDING, &dev->flags);
		clear_bit(CH390_LINKUP_CHECK_ACTIVE, &dev->flags);
		return;
	}

	/*
	 * only write pause settings to mac. since mac and phy are integrated
	 * together, such as link state, speed and duplex are sync already
	 */
	if (dev->phydev->link) {
		netif_carrier_on(ndev);
		clear_bit(CH390_LINKDN_CHECK_PENDING, &dev->flags);

		if (dev->pause.autoneg == AUTONEG_ENABLE) {
			dev->pause.rx_pause = dev->phydev->pause;
			dev->pause.tx_pause = dev->phydev->pause;
		}

		if (netif_queue_stopped(ndev))
			netif_wake_queue(ndev);

		if (mutex_trylock(&dev->spi_lockm)) {
			ch390_update_fcr(dev);
			mutex_unlock(&dev->spi_lockm);
		}

		if (test_bit(CH390_LINK_CHECK_ENABLED, &dev->flags)) {
			set_bit(CH390_LINKUP_CHECK_ACTIVE, &dev->flags);
			if (!delayed_work_pending(&dev->linkup_work) &&
			    !work_busy(&dev->linkup_work.work))
				mod_delayed_work(
					dev->wq, &dev->linkup_work,
					msecs_to_jiffies(
						CH390_LINK_UP_INTVAL_MS));
		}
		cancel_delayed_work_sync(&dev->linkdn_work);
	} else {
		netif_stop_queue(ndev);
		netif_carrier_off(ndev);
		clear_bit(CH390_LINKDN_CHECK_PENDING, &dev->flags);
		clear_bit(CH390_LINKUP_CHECK_ACTIVE, &dev->flags);
		if (!delayed_work_pending(&dev->linkdn_work) &&
		    !work_busy(&dev->linkdn_work.work) &&
		    test_bit(CH390_LINK_CHECK_ENABLED, &dev->flags))
			mod_delayed_work(
				dev->wq, &dev->linkdn_work,
				msecs_to_jiffies(CH390_LINK_DOWN_INTVAL_MS));
		cancel_delayed_work_sync(&dev->linkup_work);
	}
}

/*
 * phy connect as poll mode
 */
static int ch390_phy_connect(struct ch390_priv *dev)
{
	struct phy_device *phydev;
	int ret;

	phydev = mdiobus_get_phy(dev->mdiobus, CH390_PHY_ADDR);
	if (!phydev)
		return -ENODEV;

	ret = phy_connect_direct(dev->ndev, phydev, ch390_handle_link_change,
				 PHY_INTERFACE_MODE_MII);
	if (ret)
		return ret;

	dev->phydev = phydev;
	return 0;
}

static int ch390_setup_irq(struct ch390_priv *dev)
{
	struct net_device *ndev = dev->ndev;
	struct spi_device *spi = dev->spidev;

	/* if your platform supports acquire irq number from dts */
#ifdef USE_IRQ_FROM_DTS
	int irq_type;

	ndev->irq = spi->irq; /* by dts */
	if (spi->irq <= 0) {
		netdev_err(ndev, "invalid irq %d from dts\n", spi->irq);
		return -EINVAL;
	}

	irq_type = ch390_irq_type(dev);
	if (irq_type && !ch390_irq_type_is_level(irq_type)) {
		netdev_err(
			ndev,
			"unsupported irq trigger type 0x%x, only level low/high are supported\n",
			irq_type);
		return -EINVAL;
	}
#else
	unsigned long flags = IRQ_TYPE_LEVEL_HIGH;
	int gpio_irq;
	int ret;

	ret = devm_gpio_request(&spi->dev, GPIO_NUMBER, "gpioint");
	if (ret) {
		dev_err(&spi->dev, "gpio_request failed!\n");
		return ret;
	}
	ret = gpio_direction_input(GPIO_NUMBER);
	if (ret) {
		dev_err(&spi->dev, "gpio_direction_input failed!\n");
		return ret;
	}

	gpio_irq = gpio_to_irq(GPIO_NUMBER);
	if (gpio_irq < 0)
		return gpio_irq;

	ret = irq_set_irq_type(gpio_irq, flags);
	if (ret)
		return ret;

	spi->irq = gpio_irq;
	ndev->irq = spi->irq;
#endif

	return 0;
}

static int ch390_request_irq(struct ch390_priv *dev)
{
	struct net_device *ndev = dev->ndev;
	struct spi_device *spi = dev->spidev;
	int ret;

	ret = request_threaded_irq(spi->irq, NULL, ch390_rx_threaded_irq,
				   ch390_irq_flag(dev) | IRQF_ONESHOT,
				   ndev->name, dev);
	if (ret < 0) {
		netdev_err(ndev, "failed to get irq!\n");
		return ret;
	}

	return 0;
}

static ssize_t reg_dump_show(struct device *device,
			     struct device_attribute *attr, char *buf)
{
	struct net_device *ndev = dev_get_drvdata(device);
	struct ch390_priv *dev;
	int i, len = 0;
	u8 val;

	dev_info(device, "reg_dump_show");
	if (!ndev) {
		dev_err(device, "net_device is NULL\n");
		return -EINVAL;
	}

	dev = netdev_priv(ndev);
	if (!dev) {
		dev_err(device, "ch390 is NULL\n");
		return -EINVAL;
	}

	for (i = 0; i < sizeof(reg_labels) / sizeof(reg_labels[0]); i++) {
		if (ch390_get_reg(dev, reg_labels[i].reg, &val) != 0) {
			dev_err(device, "Failed to read register %s\n",
				reg_labels[i].name);
			return -EIO;
		}
		len += sprintf(buf + len, "%s: 0x%02x\n", reg_labels[i].name,
			       val);
	}

	return len;
}

static ssize_t reg_dump_store(struct device *device,
			      struct device_attribute *attr, const char *buf,
			      size_t count)
{
	struct net_device *ndev = dev_get_drvdata(device);
	struct ch390_priv *dev;
	unsigned int reg;
	u8 val;
	char reg_name[32];

	dev_info(device, "reg_dump_store\n");
	if (!ndev) {
		dev_err(device, "net_device is NULL\n");
		return -EINVAL;
	}

	dev = netdev_priv(ndev);

	if (sscanf(buf, "%31s %02hhx", reg_name, &val) == 2) {
		int i;

		for (i = 0; i < sizeof(reg_labels) / sizeof(reg_labels[0]);
		     i++) {
			if (strcmp(reg_labels[i].name, reg_name) == 0) {
				reg = reg_labels[i].reg;
				if (ch390_set_reg(dev, reg, val) < 0)
					dev_info(
						device,
						"set reg: 0x%02x - value: 0x%02x filed!\n",
						reg, val);
				else
					dev_info(
						device,
						"set reg: 0x%02x - value: 0x%02x success!\n",
						reg, val);
				break;
			}
		}
	}

	return count;
}

static DEVICE_ATTR(reg_dump, S_IRUGO | S_IWUSR, reg_dump_show, reg_dump_store);

static struct attribute *ch390_attributes[] = { &dev_attr_reg_dump.attr, NULL };

static struct attribute_group ch390_attribute_group = {
	.attrs = ch390_attributes,
};

static int ch390_create_sysfs(struct ch390_priv *dev)
{
	struct spi_device *spi = dev->spidev;
	char *link_name = dev->link_name;
	u8 cs_num;
	const char *ctrl_name;
	int ret;

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(6, 8, 0))
	ctrl_name = dev_name(&spi->controller->dev);
	cs_num = spi->chip_select[0];
#else
	ctrl_name = dev_name(&spi->master->dev);
	cs_num = spi->chip_select;
#endif

	ret = sysfs_create_group(&spi->dev.kobj, &ch390_attribute_group);
	if (ret) {
		dev_err(&spi->dev, "sysfs_create_group() failed!!");
		return ret;
	}

	snprintf(link_name, sizeof(dev->link_name), "ch390-%s-%d", ctrl_name,
		 cs_num);

	ret = sysfs_create_link(NULL, &spi->dev.kobj, link_name);
	if (ret < 0) {
		dev_err(&spi->dev, "Failed to create link: %s", link_name);
		sysfs_remove_group(&spi->dev.kobj, &ch390_attribute_group);
		return ret;
	}

	dev_info(&spi->dev, "sysfs_create_group() and link %s succeeded!",
		 link_name);

	return ret;
}

static int ch390_check_link_stat(struct ch390_priv *dev)
{
	unsigned int bmsr = 0;
	int ret;

	ret = mdiobus_read(dev->mdiobus, CH390_PHY_ADDR, MII_BMSR);
	if (ret < 0)
		return ret;
	bmsr = ret;

	if (!(bmsr & BMSR_LSTATUS)) {
		ret = mdiobus_read(dev->mdiobus, CH390_PHY_ADDR, MII_BMSR);
		if (ret < 0)
			return ret;
		bmsr = ret;
	}

	ch390_reg_dbg(dev, "bmsr=0x%04x link=%d\n", bmsr & 0xffff,
		      !!(bmsr & BMSR_LSTATUS));

	return !!(bmsr & BMSR_LSTATUS);
}

static bool ch390_check_base_reg(struct ch390_priv *dev)
{
	unsigned int val = 0;
	int ret;

	ret = mdiobus_read(dev->mdiobus, CH390_PHY_ADDR, MII_EXPANSION);
	if (ret < 0) {
		netdev_err(dev->ndev, "failed to read expansion register: %d\n",
			   ret);
		return false;
	}
	val = ret;

	ch390_reg_dbg(dev, "expansion=0x%04x\n", val & 0xffff);

	return !(val & EXPANSION_NWAY);
}

static int ch390_set_link_interrupt(struct ch390_priv *dev, bool enable)
{
	struct mii_bus *bus = dev->mdiobus;
	int val, ret;

	ret = ch390_mdio_write(bus, CH390_PHY_ADDR, CH390_PHY_PAG_SEL,
			       CH390_PHY_PAGE7);
	if (ret < 0)
		return ret;

	val = ch390_mdio_read(bus, CH390_PHY_ADDR, CH390_INTERRUPT_MASK);
	if (val < 0)
		return val;

	if (enable)
		val |= CH390_INT_LINKCHG;
	else
		val &= ~CH390_INT_LINKCHG;

	return ch390_mdio_write(bus, CH390_PHY_ADDR, CH390_INTERRUPT_MASK, val);
}

static int ch390_modify_page0(struct ch390_priv *dev, u32 regnum, u16 mask,
			      u16 set)
{
	struct mii_bus *bus = dev->mdiobus;
	int val, ret;

	ret = ch390_mdio_write(bus, CH390_PHY_ADDR, CH390_PHY_PAG_SEL,
			       CH390_PHY_PAGE0);
	if (ret < 0)
		return ret;

	val = ch390_mdio_read(bus, CH390_PHY_ADDR, regnum);
	if (val < 0)
		return val;

	val &= ~mask;
	val |= set;

	return ch390_mdio_write(bus, CH390_PHY_ADDR, regnum, val);
}

static void ch390_restore_page0(struct ch390_priv *dev)
{
	struct mii_bus *bus = dev->mdiobus;
	int ret;

	ret = ch390_mdio_write(bus, CH390_PHY_ADDR, CH390_PHY_PAG_SEL,
			       CH390_PHY_PAGE0);
	if (ret < 0)
		netdev_err(dev->ndev, "failed to restore PHY page0: %d\n", ret);
}

static void ch390_set_phypn(struct ch390_priv *dev)
{
	struct mii_bus *bus = dev->mdiobus;
	bool link_int_disabled = false;
	int ret, enable_ret;

	ch390_recovery_dbg(dev, "trigger phypn pulse\n");

	mutex_lock(&bus->mdio_lock);

	ret = ch390_set_link_interrupt(dev, false);
	if (ret < 0) {
		netdev_err(dev->ndev, "failed to disable link interrupt: %d\n",
			   ret);
		goto out;
	}
	link_int_disabled = true;

	ret = ch390_modify_page0(dev, dev->regs->set_phypn_reg,
				 dev->regs->set_phypn_mask,
				 dev->regs->set_phypn_mask);
	if (ret < 0) {
		netdev_err(dev->ndev,
			   "failed to set polarity recovery bit: %d\n", ret);
		goto out;
	}

	ret = ch390_modify_page0(dev, dev->regs->set_phypn_reg,
				 dev->regs->set_phypn_mask, 0);
	if (ret < 0) {
		netdev_err(dev->ndev,
			   "failed to clear polarity recovery bit: %d\n", ret);
		goto out;
	}

	ret = ch390_mdio_read(bus, CH390_PHY_ADDR, CH390_INTERRUPT_IND);
	if (ret < 0)
		netdev_err(dev->ndev, "failed to read interrupt status: %d\n",
			   ret);

out:
	if (link_int_disabled) {
		enable_ret = ch390_set_link_interrupt(dev, true);
		if (enable_ret < 0)
			netdev_err(dev->ndev,
				   "failed to enable link interrupt: %d\n",
				   enable_ret);
	}

	ch390_restore_page0(dev);
	mutex_unlock(&bus->mdio_lock);
}

static void ch390_latch_phypn(struct ch390_priv *dev)
{
	struct mii_bus *bus = dev->mdiobus;
	int ret;

	ch390_recovery_dbg(dev, "trigger phypn set\n");

	mutex_lock(&bus->mdio_lock);

	ret = ch390_modify_page0(dev, dev->regs->set_phypn_reg,
				 dev->regs->set_phypn_mask,
				 dev->regs->set_phypn_mask);
	if (ret < 0)
		netdev_err(dev->ndev,
			   "failed to set polarity recovery bit: %d\n", ret);

	ch390_restore_page0(dev);
	mutex_unlock(&bus->mdio_lock);
}

static bool ch390_prepare_phypn_check(struct ch390_priv *dev, bool autoneg,
				      unsigned int *delay_ms)
{
	int phy_stat;

	phy_stat = ch390_page_read(dev, CH390_PHY_PAGE0,
				   dev->regs->check_phypn_reg);
	if (phy_stat < 0) {
		netdev_err(dev->ndev, "failed to read PHY status0: %d\n",
			   phy_stat);
		return false;
	}

	if (!(phy_stat & dev->regs->check_phypn_mask))
		return false;

	*delay_ms = autoneg ? 300 : 200;
	ch390_reg_dbg(dev,
		      "polarity set, autoneg=%d status0=0x%04x delay=%ums\n",
		      autoneg, phy_stat & 0xffff, *delay_ms);

	return true;
}

static int ch390_finish_phypn_check(struct ch390_priv *dev, bool autoneg)
{
	int link;

	link = ch390_check_link_stat(dev);
	if (link < 0)
		return link;

	ch390_reg_dbg(dev, "polarity recheck autoneg=%d link=%d\n", autoneg,
		      link);

	if (!link && (!autoneg || ch390_check_base_reg(dev))) {
		dev->set_phypn_triggers++;
		dev->dev_ops->set_phypn(dev);
	}

	return link;
}

/* Select the link-down maintenance sequence from the current PHY mode. */
static bool ch390_link_processing(struct ch390_priv *dev, bool *autoneg,
				  unsigned int *delay_ms)
{
	unsigned int bmcr = 0;
	int ret;

	ret = mdiobus_read(dev->mdiobus, CH390_PHY_ADDR, MII_BMCR);
	if (ret < 0)
		return false;
	bmcr = ret;

	ch390_reg_dbg(dev, "bmcr=0x%04x\n", bmcr & 0xffff);

	if (bmcr & BMCR_ANENABLE) {
		*autoneg = true;
		return ch390_prepare_phypn_check(dev, *autoneg, delay_ms);
	}

	if (!(bmcr & BMCR_SPEED100)) {
		*autoneg = false;
		return ch390_prepare_phypn_check(dev, *autoneg, delay_ms);
	}

	return false;
}

/* Periodic link-down monitor. Keeps maintenance active while link is absent. */
static void ch390_linkdn_work(struct work_struct *work)
{
	struct ch390_priv *dev = container_of(to_delayed_work(work),
					      struct ch390_priv, linkdn_work);
	unsigned int delay_ms = 0;
	bool phypn_pending;
	bool autoneg = false;
	int link = 0;

	if (!test_bit(CH390_LINK_CHECK_ENABLED, &dev->flags))
		return;

	phypn_pending = test_bit(CH390_LINKDN_CHECK_PENDING, &dev->flags);

	if (!mutex_trylock(&dev->spi_lockm))
		goto out_reschedule;

	if (!test_bit(CH390_LINK_CHECK_ENABLED, &dev->flags))
		goto out_unlock;

	if (phypn_pending) {
		clear_bit(CH390_LINKDN_CHECK_PENDING, &dev->flags);
		autoneg = test_bit(CH390_LINKDN_CHECK_AUTONEG, &dev->flags);
		link = ch390_finish_phypn_check(dev, autoneg);
		phypn_pending = false;
		if (link < 0)
			goto out_unlock;
	} else {
		link = ch390_check_link_stat(dev);
		if (link < 0)
			goto out_unlock;

		ch390_flow_dbg(dev, "link-down worker link=%d\n", link);

		if (!link)
			phypn_pending =
				ch390_link_processing(dev, &autoneg, &delay_ms);
	}

out_unlock:
	mutex_unlock(&dev->spi_lockm);

	if (phypn_pending && test_bit(CH390_LINK_CHECK_ENABLED, &dev->flags)) {
		if (autoneg)
			set_bit(CH390_LINKDN_CHECK_AUTONEG, &dev->flags);
		else
			clear_bit(CH390_LINKDN_CHECK_AUTONEG, &dev->flags);
		set_bit(CH390_LINKDN_CHECK_PENDING, &dev->flags);
		mod_delayed_work(dev->wq, &dev->linkdn_work,
				 msecs_to_jiffies(delay_ms));
		return;
	}

out_reschedule:
	if (test_bit(CH390_LINK_CHECK_ENABLED, &dev->flags) && link <= 0)
		mod_delayed_work(dev->wq, &dev->linkdn_work,
				 msecs_to_jiffies(CH390_LINK_DOWN_INTVAL_MS));
}

/* Use phylib's resolved speed for link-up monitoring. */
static bool ch390_is_100m_link(struct ch390_priv *dev)
{
	struct phy_device *phydev = dev->phydev;

	ch390_flow_dbg(dev, "resolved speed=%d\n", phydev->speed);

	return phydev->speed == SPEED_100;
}

static int ch390_read_linkup_sample(struct ch390_priv *dev)
{
	int val;

	val = ch390_page_read(dev, dev->regs->check_link_page,
			      dev->regs->check_link_reg);
	if (val < 0) {
		netdev_err(dev->ndev, "failed to read link-up status: %d\n",
			   val);
		return val;
	}

	ch390_reg_dbg(dev, "link-up page=0x%02x reg=0x%02x val=0x%04x\n",
		      dev->regs->check_link_page, dev->regs->check_link_reg,
		      val & 0xffff);

	return !!(val & dev->regs->check_link_mask);
}

static void ch390_restart_false_link(struct ch390_priv *dev)
{
	int link, ret;

	link = ch390_check_link_stat(dev);
	ch390_flow_dbg(dev, "link-up status bad count=%d link=%d\n",
		       CH390_LINK_UP_FAIL_COUNT, link);

	if (link > 0 && ch390_is_100m_link(dev)) {
		dev->linkup_restarts++;
		ret = ch390_all_restart(dev);
		if (ret < 0)
			netdev_err(dev->ndev,
				   "failed to reset after false link: %d\n",
				   ret);
	}
}

/* Link-up health check: sample PHY-side status before recovery. */
static bool ch390_check_link(struct ch390_priv *dev)
{
	bool all_bad = true;
	int ok_run = 0;
	int sample;
	int i;

	for (i = 0; i < CH390_LINK_UP_FAIL_COUNT; i++) {
		sample = ch390_read_linkup_sample(dev);
		if (sample < 0)
			return false;

		/* Two adjacent good samples are enough to trust the link. */
		if (sample) {
			all_bad = false;
			if (++ok_run >= CH390_LINK_UP_PASS_COUNT) {
				ch390_flow_dbg(dev,
					       "link-up status ok count=%d\n",
					       ok_run);
				return true;
			}
		} else {
			ok_run = 0;
		}

		if (i + 1 < CH390_LINK_UP_FAIL_COUNT)
			usleep_range(CH390_LINK_UP_SAMPLE_DELAY_US,
				     CH390_LINK_UP_SAMPLE_DELAY_US + 500);
	}

	/* Recovery is reserved for a fully bad sample window. */
	if (all_bad)
		ch390_restart_false_link(dev);

	return false;
}

/* Periodic link-up monitor. Runs only while link remains active. */
static void ch390_linkup_work(struct work_struct *work)
{
	struct ch390_priv *dev = container_of(to_delayed_work(work),
					      struct ch390_priv, linkup_work);
	int link = 1;

	if (!test_bit(CH390_LINK_CHECK_ENABLED, &dev->flags) ||
	    !test_bit(CH390_LINKUP_CHECK_ACTIVE, &dev->flags))
		return;

	if (!mutex_trylock(&dev->spi_lockm))
		goto out_reschedule;

	if (!test_bit(CH390_LINK_CHECK_ENABLED, &dev->flags) ||
	    !test_bit(CH390_LINKUP_CHECK_ACTIVE, &dev->flags))
		goto out_unlock;

	if (!ch390_is_100m_link(dev)) {
		clear_bit(CH390_LINKUP_CHECK_ACTIVE, &dev->flags);
		goto out_unlock;
	}

	if (ch390_check_link(dev)) {
		clear_bit(CH390_LINKUP_CHECK_ACTIVE, &dev->flags);
		goto out_unlock;
	}

	link = ch390_check_link_stat(dev);
	ch390_flow_dbg(dev, "link-up worker link=%d\n", link);

	if (link == 0)
		clear_bit(CH390_LINKUP_CHECK_ACTIVE, &dev->flags);

out_unlock:
	mutex_unlock(&dev->spi_lockm);

out_reschedule:
	if (test_bit(CH390_LINK_CHECK_ENABLED, &dev->flags) &&
	    test_bit(CH390_LINKUP_CHECK_ACTIVE, &dev->flags) && link != 0)
		mod_delayed_work(dev->wq, &dev->linkup_work,
				 msecs_to_jiffies(CH390_LINK_UP_INTVAL_MS));
}

static int ch390_init(struct ch390_priv *dev)
{
	int val;

	val = ch390_page_read(dev, CH390_PHY_PAGE0, CH390_PHY_STATUS0);
	if (val < 0) {
		netdev_err(dev->ndev, "failed to read PHY_STATUS0: %d\n", val);
		return val;
	}

	if (val == 0) {
		dev->regs = &ch390_pulse_phypn_regs;
		dev->dev_ops = &ch390_pulse_phypn_ops;
	} else {
		dev->regs = &ch390_latch_phypn_regs;
		dev->dev_ops = &ch390_latch_phypn_ops;
	}

	printk(KERN_INFO "ch390-%d device probe, driver version: %s\n", val,
	       VERSION_DESC);

	return 0;
}

static int ch390_probe(struct spi_device *spi)
{
	struct device *device = &spi->dev;
	struct net_device *ndev;
	struct ch390_priv *dev;
	int ret;

	ndev = alloc_etherdev(sizeof(*dev));
	if (!ndev)
		return -ENOMEM;

	SET_NETDEV_DEV(ndev, device);
	dev_set_drvdata(device, ndev);

	dev = netdev_priv(ndev);

	dev->msg_enable = 0;
	dev->reg_stride = 1;
	dev->spidev = spi;
	dev->ndev = ndev;

	dev->spi_tx_buf = kzalloc(CH390_SPI_READ_BUF_SIZE, GFP_KERNEL);
	if (!dev->spi_tx_buf) {
		ret = -ENOMEM;
		goto out1;
	}

	dev->spi_rx_buf = kzalloc(CH390_SPI_READ_BUF_SIZE, GFP_KERNEL);
	if (!dev->spi_rx_buf) {
		ret = -ENOMEM;
		goto out1;
	}

	ndev->netdev_ops = &ch390_netdev_ops;
	ndev->ethtool_ops = &ch390_ethtool_ops;
	ndev->watchdog_timeo = 5 * HZ;

	mutex_init(&dev->spi_lockm);
	mutex_init(&dev->phy_mutex);
	mutex_init(&dev->reg_mutex);

	clear_bit(CH390_LINK_CHECK_ENABLED, &dev->flags);
	clear_bit(CH390_LINKDN_CHECK_PENDING, &dev->flags);
	clear_bit(CH390_LINKDN_CHECK_AUTONEG, &dev->flags);
	clear_bit(CH390_LINKUP_CHECK_ACTIVE, &dev->flags);
	set_bit(CH390_DEV_STOPPING, &dev->flags);

	dev->wq = alloc_workqueue("ch390_wq", WQ_MEM_RECLAIM, 1);
	if (!dev->wq) {
		dev_err(&spi->dev, "failed to create workqueue\n");
		ret = -ENOMEM;
		goto out1;
	}

	INIT_DELAYED_WORK(&dev->linkdn_work, ch390_linkdn_work);
	INIT_DELAYED_WORK(&dev->linkup_work, ch390_linkup_work);
	INIT_WORK(&dev->rx_mode_work, ch390_rx_mode_delay);
	INIT_WORK(&dev->tx_work, ch390_tx_delay);
	INIT_WORK(&dev->tx_timeout_work, ch390_tx_timeout_work);

	ret = ch390_map_chipid(dev);
	if (ret)
		goto out_wq;

	ret = ch390_map_etherdev_par(ndev, dev);
	if (ret < 0)
		goto out_wq;

	ret = ch390_mdio_register(dev);
	if (ret)
		goto out_wq;

	ret = ch390_init(dev);
	if (ret)
		goto out_mdio;

	ret = ch390_phy_connect(dev);
	if (ret)
		goto out_mdio;

	ch390_operation_clear(dev);
	skb_queue_head_init(&dev->txq);

	ret = ch390_setup_irq(dev);
	if (ret)
		goto out_phy_disconnect;

	ret = register_netdev(ndev);
	if (ret) {
		dev_err(device, "device register failed: %d\n", ret);
		goto out_phy_disconnect;
	}

	ret = ch390_create_sysfs(dev);
	if (ret) {
		dev_err(device, "device create sysfs failed: %d\n", ret);
		goto out_unregister;
	}

	return 0;

out_unregister:
	unregister_netdev(ndev);
out_phy_disconnect:
	phy_disconnect(dev->phydev);
out_mdio:
	ch390_mdio_unregister(dev);
out_wq:
	set_bit(CH390_DEV_STOPPING, &dev->flags);
	clear_bit(CH390_LINKDN_CHECK_PENDING, &dev->flags);
	clear_bit(CH390_LINKDN_CHECK_AUTONEG, &dev->flags);
	clear_bit(CH390_LINKUP_CHECK_ACTIVE, &dev->flags);
	cancel_work_sync(&dev->tx_work);
	cancel_work_sync(&dev->tx_timeout_work);
	cancel_work_sync(&dev->rx_mode_work);
	cancel_delayed_work_sync(&dev->linkdn_work);
	cancel_delayed_work_sync(&dev->linkup_work);
	destroy_workqueue(dev->wq);
	dev->wq = NULL;
out1:
	kfree(dev->spi_rx_buf);
	kfree(dev->spi_tx_buf);
	dev_set_drvdata(device, NULL);
	free_netdev(ndev);
	return ret;
}

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 18, 0))
static void ch390_drv_remove(struct spi_device *spi)
{
	struct device *device = &spi->dev;
	struct net_device *ndev = dev_get_drvdata(device);
	struct ch390_priv *dev = to_ch390_priv(ndev);

	sysfs_remove_link(NULL, dev->link_name);
	sysfs_remove_group(&spi->dev.kobj, &ch390_attribute_group);
	unregister_netdev(ndev);
	set_bit(CH390_DEV_STOPPING, &dev->flags);
	clear_bit(CH390_LINK_CHECK_ENABLED, &dev->flags);
	clear_bit(CH390_LINKDN_CHECK_PENDING, &dev->flags);
	clear_bit(CH390_LINKDN_CHECK_AUTONEG, &dev->flags);
	clear_bit(CH390_LINKUP_CHECK_ACTIVE, &dev->flags);
	cancel_work_sync(&dev->tx_work);
	cancel_work_sync(&dev->tx_timeout_work);
	cancel_work_sync(&dev->rx_mode_work);
	cancel_delayed_work_sync(&dev->linkdn_work);
	cancel_delayed_work_sync(&dev->linkup_work);
	destroy_workqueue(dev->wq);
	dev->wq = NULL;
	phy_disconnect(dev->phydev);
	ch390_mdio_unregister(dev);
	kfree(dev->spi_rx_buf);
	kfree(dev->spi_tx_buf);
	dev_set_drvdata(device, NULL);
	free_netdev(ndev);

	printk(KERN_INFO "ch390 device driver remove\n");
}
#else
static int ch390_drv_remove(struct spi_device *spi)
{
	struct device *device = &spi->dev;
	struct net_device *ndev = dev_get_drvdata(device);
	struct ch390_priv *dev = to_ch390_priv(ndev);

	sysfs_remove_link(NULL, dev->link_name);
	sysfs_remove_group(&spi->dev.kobj, &ch390_attribute_group);
	unregister_netdev(ndev);
	set_bit(CH390_DEV_STOPPING, &dev->flags);
	clear_bit(CH390_LINK_CHECK_ENABLED, &dev->flags);
	clear_bit(CH390_LINKDN_CHECK_PENDING, &dev->flags);
	clear_bit(CH390_LINKDN_CHECK_AUTONEG, &dev->flags);
	clear_bit(CH390_LINKUP_CHECK_ACTIVE, &dev->flags);
	cancel_work_sync(&dev->tx_work);
	cancel_work_sync(&dev->tx_timeout_work);
	cancel_work_sync(&dev->rx_mode_work);
	cancel_delayed_work_sync(&dev->linkdn_work);
	cancel_delayed_work_sync(&dev->linkup_work);
	destroy_workqueue(dev->wq);
	dev->wq = NULL;
	phy_disconnect(dev->phydev);
	ch390_mdio_unregister(dev);
	kfree(dev->spi_rx_buf);
	kfree(dev->spi_tx_buf);
	dev_set_drvdata(device, NULL);
	free_netdev(ndev);

	printk(KERN_INFO "ch390 device driver remove\n");

	return 0;
}
#endif

static const struct of_device_id ch390_match_table[] = {
	{ .compatible = "ch390_ethernet" },
	{}
};
MODULE_DEVICE_TABLE(of, ch390_match_table);

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
