// SPDX-License-Identifier: GPL-2.0-only
/*
 *
 *   Copyright (C) 2009-2016 John Crispin <blogic@openwrt.org>
 *   Copyright (C) 2009-2016 Felix Fietkau <nbd@openwrt.org>
 *   Copyright (C) 2013-2016 Michael Lee <igvtee@gmail.com>
 */

#include <linux/of_device.h>
#include <linux/of_mdio.h>
#include <linux/of_net.h>
#include <linux/of_address.h>
#include <linux/mfd/syscon.h>
#include <linux/regmap.h>
#include <linux/clk.h>
#include <linux/pm_runtime.h>
#include <linux/if_vlan.h>
#include <linux/reset.h>
#include <linux/tcp.h>
#include <linux/interrupt.h>
#include <linux/pinctrl/devinfo.h>
#include <linux/phylink.h>
#include <linux/gpio/consumer.h>
#include <net/dsa.h>

#include "mtk_eth_soc.h"
#include "mtk_eth_dbg.h"
#include "mtk_eth_reset.h"

#if defined(CONFIG_NET_MEDIATEK_HNAT) || defined(CONFIG_NET_MEDIATEK_HNAT_MODULE)
#include "mtk_hnat/nf_hnat_mtk.h"
#endif

static int mtk_msg_level = -1;
atomic_t reset_lock = ATOMIC_INIT(0);
atomic_t force = ATOMIC_INIT(0);

module_param_named(msg_level, mtk_msg_level, int, 0);
MODULE_PARM_DESC(msg_level, "Message level (-1=defaults,0=none,...,16=all)");
DECLARE_COMPLETION(wait_ser_done);

#define MTK_ETHTOOL_STAT(x) { #x, \
			      offsetof(struct mtk_hw_stats, x) / sizeof(u64) }

static const struct mtk_reg_map mtk_reg_map = {
	.tx_irq_mask		= 0x1a1c,
	.tx_irq_status		= 0x1a18,
	.pdma = {
		.tx_ptr		= 0x0800,
		.tx_cnt_cfg	= 0x0804,
		.pctx_ptr	= 0x0808,
		.pdtx_ptr	= 0x080c,
		.rx_ptr		= 0x0900,
		.rx_cnt_cfg	= 0x0904,
		.pcrx_ptr	= 0x0908,
		.glo_cfg	= 0x0a04,
		.rst_idx	= 0x0a08,
		.delay_irq	= 0x0a0c,
		.irq_status	= 0x0a20,
		.irq_mask	= 0x0a28,
		.int_grp	= 0x0a50,
		.int_grp2	= 0x0a54,
	},
	.qdma = {
		.qtx_cfg	= 0x1800,
		.qtx_sch	= 0x1804,
		.rx_ptr		= 0x1900,
		.rx_cnt_cfg	= 0x1904,
		.qcrx_ptr	= 0x1908,
		.glo_cfg	= 0x1a04,
		.rst_idx	= 0x1a08,
		.delay_irq	= 0x1a0c,
		.fc_th		= 0x1a10,
		.tx_sch_rate	= 0x1a14,
		.int_grp	= 0x1a20,
		.int_grp2	= 0x1a24,
		.hred2		= 0x1a44,
		.ctx_ptr	= 0x1b00,
		.dtx_ptr	= 0x1b04,
		.crx_ptr	= 0x1b10,
		.drx_ptr	= 0x1b14,
		.fq_head	= 0x1b20,
		.fq_tail	= 0x1b24,
		.fq_count	= 0x1b28,
		.fq_blen	= 0x1b2c,
	},
	.gdm1_cnt		= 0x2400,
	.gdma_to_ppe0		= 0x4444,
	.ppe_base = {
		[0]		= 0x0c00,
	},
	.wdma_base = {
		[0]		= 0x2800,
		[1]		= 0x2c00,
	},
};

static const struct mtk_reg_map mt7628_reg_map = {
	.tx_irq_mask		= 0x0a28,
	.tx_irq_status		= 0x0a20,
	.pdma = {
		.tx_ptr		= 0x0800,
		.tx_cnt_cfg	= 0x0804,
		.pctx_ptr	= 0x0808,
		.pdtx_ptr	= 0x080c,
		.rx_ptr		= 0x0900,
		.rx_cnt_cfg	= 0x0904,
		.pcrx_ptr	= 0x0908,
		.glo_cfg	= 0x0a04,
		.rst_idx	= 0x0a08,
		.delay_irq	= 0x0a0c,
		.irq_status	= 0x0a20,
		.irq_mask	= 0x0a28,
		.int_grp	= 0x0a50,
		.int_grp2	= 0x0a54,
	},
};

static const struct mtk_reg_map mt7986_reg_map = {
	.tx_irq_mask		= 0x461c,
	.tx_irq_status		= 0x4618,
	.pdma = {
		.tx_ptr		= 0x4000,
		.tx_cnt_cfg	= 0x4004,
		.pctx_ptr	= 0x4008,
		.pdtx_ptr	= 0x400c,
		.rx_ptr		= 0x4100,
		.rx_cnt_cfg	= 0x4104,
		.pcrx_ptr	= 0x4108,
		.glo_cfg	= 0x4204,
		.rst_idx	= 0x4208,
		.delay_irq	= 0x420c,
		.irq_status	= 0x4220,
		.irq_mask	= 0x4228,
		.int_grp	= 0x4250,
		.int_grp2	= 0x4254,
	},
	.qdma = {
		.qtx_cfg	= 0x4400,
		.qtx_sch	= 0x4404,
		.rx_ptr		= 0x4500,
		.rx_cnt_cfg	= 0x4504,
		.qcrx_ptr	= 0x4508,
		.glo_cfg	= 0x4604,
		.rst_idx	= 0x4608,
		.delay_irq	= 0x460c,
		.fc_th		= 0x4610,
		.int_grp	= 0x4620,
		.int_grp2	= 0x4624,
		.hred2		= 0x4644,
		.ctx_ptr	= 0x4700,
		.dtx_ptr	= 0x4704,
		.crx_ptr	= 0x4710,
		.drx_ptr	= 0x4714,
		.fq_head	= 0x4720,
		.fq_tail	= 0x4724,
		.fq_count	= 0x4728,
		.fq_blen	= 0x472c,
		.tx_sch_rate	= 0x4798,
	},
	.gdm1_cnt		= 0x1c00,
	.gdma_to_ppe0		= 0x3333,
	.ppe_base = {
		[0]		= 0x2000,
		[1]		= 0x2400,
	},
	.wdma_base = {
		[0]		= 0x4800,
		[1]		= 0x4c00,
	},
};

static const struct mtk_reg_map mt7988_reg_map = {
	.tx_irq_mask		= 0x461c,
	.tx_irq_status		= 0x4618,
	.pdma = {
		.tx_ptr		= 0x6800,
		.tx_cnt_cfg	= 0x6804,
		.pctx_ptr	= 0x6808,
		.pdtx_ptr	= 0x680c,
		.rx_ptr		= 0x6900,
		.rx_cnt_cfg	= 0x6904,
		.pcrx_ptr	= 0x6908,
		.glo_cfg	= 0x6a04,
		.rst_idx	= 0x6a08,
		.delay_irq	= 0x6a0c,
		.irq_status	= 0x6a20,
		.irq_mask	= 0x6a28,
		.int_grp	= 0x6a50,
		.int_grp2	= 0x6a54,
	},
	.qdma = {
		.qtx_cfg	= 0x4400,
		.qtx_sch	= 0x4404,
		.rx_ptr		= 0x4500,
		.rx_cnt_cfg	= 0x4504,
		.qcrx_ptr	= 0x4508,
		.glo_cfg	= 0x4604,
		.rst_idx	= 0x4608,
		.delay_irq	= 0x460c,
		.fc_th		= 0x4610,
		.int_grp	= 0x4620,
		.int_grp2	= 0x4624,
		.hred2		= 0x4644,
		.ctx_ptr	= 0x4700,
		.dtx_ptr	= 0x4704,
		.crx_ptr	= 0x4710,
		.drx_ptr	= 0x4714,
		.fq_head	= 0x4720,
		.fq_tail	= 0x4724,
		.fq_count	= 0x4728,
		.fq_blen	= 0x472c,
		.tx_sch_rate	= 0x4798,
	},
	.gdm1_cnt		= 0x1c00,
	.gdma_to_ppe0		= 0x3333,
	.ppe_base = {
		[0]		= 0x2000,
		[1]		= 0x2400,
		[2]		= 0x2c00,
	},
	.wdma_base = {
		[0]		= 0x4800,
		[1]		= 0x4c00,
		[2]		= 0x5000,
	},
};

/* strings used by ethtool */
static const struct mtk_ethtool_stats {
	char str[ETH_GSTRING_LEN];
	u32 offset;
} mtk_ethtool_stats[] = {
	MTK_ETHTOOL_STAT(tx_bytes),
	MTK_ETHTOOL_STAT(tx_packets),
	MTK_ETHTOOL_STAT(tx_skip),
	MTK_ETHTOOL_STAT(tx_collisions),
	MTK_ETHTOOL_STAT(rx_bytes),
	MTK_ETHTOOL_STAT(rx_packets),
	MTK_ETHTOOL_STAT(rx_overflow),
	MTK_ETHTOOL_STAT(rx_fcs_errors),
	MTK_ETHTOOL_STAT(rx_short_errors),
	MTK_ETHTOOL_STAT(rx_long_errors),
	MTK_ETHTOOL_STAT(rx_checksum_errors),
	MTK_ETHTOOL_STAT(rx_flow_control_packets),
};

static const char * const mtk_clks_source_name[] = {
	"ethif", "sgmiitop", "esw", "gp0", "gp1", "gp2", "gp3",
	"xgp1", "xgp2", "xgp3", "crypto", "fe", "trgpll",
	"sgmii_tx250m", "sgmii_rx250m", "sgmii_cdr_ref", "sgmii_cdr_fb",
	"sgmii2_tx250m", "sgmii2_rx250m", "sgmii2_cdr_ref", "sgmii2_cdr_fb",
	"sgmii_ck", "eth2pll", "wocpu0", "wocpu1",
	"ethwarp_wocpu2", "ethwarp_wocpu1", "ethwarp_wocpu0",
	"top_usxgmii0_sel", "top_usxgmii1_sel", "top_sgm0_sel", "top_sgm1_sel",
	"top_xfi_phy0_xtal_sel", "top_xfi_phy1_xtal_sel", "top_eth_gmii_sel",
	"top_eth_refck_50m_sel", "top_eth_sys_200m_sel", "top_eth_sys_sel",
	"top_eth_xgmii_sel", "top_eth_mii_sel", "top_netsys_sel",
	"top_netsys_500m_sel", "top_netsys_pao_2x_sel",
	"top_netsys_sync_250m_sel", "top_netsys_ppefb_250m_sel",
	"top_netsys_warp_sel", "top_macsec_sel",
};

void mtk_w32(struct mtk_eth *eth, u32 val, unsigned reg)
{
	__raw_writel(val, eth->base + reg);
}

u32 mtk_r32(struct mtk_eth *eth, unsigned reg)
{
	return __raw_readl(eth->base + reg);
}

u32 mtk_m32(struct mtk_eth *eth, u32 mask, u32 set, unsigned reg)
{
	u32 val;

	val = mtk_r32(eth, reg);
	val &= ~mask;
	val |= set;
	mtk_w32(eth, val, reg);
	return reg;
}

static int mtk_mdio_busy_wait(struct mtk_eth *eth)
{
	unsigned long t_start = jiffies;

	while (1) {
		if (!(mtk_r32(eth, MTK_PHY_IAC) & PHY_IAC_ACCESS))
			return 0;
		if (time_after(jiffies, t_start + PHY_IAC_TIMEOUT))
			break;
		cond_resched();
	}

	dev_err(eth->dev, "mdio: MDIO timeout\n");
	return -1;
}

u32 _mtk_mdio_write(struct mtk_eth *eth, int phy_addr,
			   int phy_reg, u16 write_data)
{
	if (mtk_mdio_busy_wait(eth))
		return -1;

	write_data &= 0xffff;

	if (phy_reg & MII_ADDR_C45) {
		mtk_w32(eth, PHY_IAC_ACCESS | PHY_IAC_START_C45 | PHY_IAC_ADDR_C45 |
			((mdiobus_c45_devad(phy_reg) & 0x1f) << PHY_IAC_REG_SHIFT) |
			((phy_addr & 0x1f) << PHY_IAC_ADDR_SHIFT) | mdiobus_c45_regad(phy_reg),
			MTK_PHY_IAC);

		if (mtk_mdio_busy_wait(eth))
			return -1;

		mtk_w32(eth, PHY_IAC_ACCESS | PHY_IAC_START_C45 | PHY_IAC_WRITE |
			((mdiobus_c45_devad(phy_reg) & 0x1f) << PHY_IAC_REG_SHIFT) |
			((phy_addr & 0x1f) << PHY_IAC_ADDR_SHIFT) | write_data,
			MTK_PHY_IAC);
	} else {
		mtk_w32(eth, PHY_IAC_ACCESS | PHY_IAC_START | PHY_IAC_WRITE |
			((phy_reg & 0x1f) << PHY_IAC_REG_SHIFT) |
			((phy_addr & 0x1f) << PHY_IAC_ADDR_SHIFT) | write_data,
			MTK_PHY_IAC);
	}

	if (mtk_mdio_busy_wait(eth))
		return -1;

	return 0;
}

u32 _mtk_mdio_read(struct mtk_eth *eth, int phy_addr, int phy_reg)
{
	u32 d;

	if (mtk_mdio_busy_wait(eth))
		return 0xffff;

	if (phy_reg & MII_ADDR_C45) {
		mtk_w32(eth, PHY_IAC_ACCESS | PHY_IAC_START_C45 | PHY_IAC_ADDR_C45 |
			((mdiobus_c45_devad(phy_reg) & 0x1f) << PHY_IAC_REG_SHIFT) |
			((phy_addr & 0x1f) << PHY_IAC_ADDR_SHIFT) | mdiobus_c45_regad(phy_reg),
			MTK_PHY_IAC);

		if (mtk_mdio_busy_wait(eth))
			return 0xffff;

		mtk_w32(eth, PHY_IAC_ACCESS | PHY_IAC_START_C45 | PHY_IAC_READ_C45 |
			((mdiobus_c45_devad(phy_reg) & 0x1f) << PHY_IAC_REG_SHIFT) |
			((phy_addr & 0x1f) << PHY_IAC_ADDR_SHIFT),
			MTK_PHY_IAC);
	} else {
		mtk_w32(eth, PHY_IAC_ACCESS | PHY_IAC_START | PHY_IAC_READ |
			((phy_reg & 0x1f) << PHY_IAC_REG_SHIFT) |
			((phy_addr & 0x1f) << PHY_IAC_ADDR_SHIFT),
			MTK_PHY_IAC);
	}

	if (mtk_mdio_busy_wait(eth))
		return 0xffff;

	d = mtk_r32(eth, MTK_PHY_IAC) & 0xffff;

	return d;
}

static int mtk_mdio_write(struct mii_bus *bus, int phy_addr,
			  int phy_reg, u16 val)
{
	struct mtk_eth *eth = bus->priv;

	return _mtk_mdio_write(eth, phy_addr, phy_reg, val);
}

static int mtk_mdio_read(struct mii_bus *bus, int phy_addr, int phy_reg)
{
	struct mtk_eth *eth = bus->priv;

	return _mtk_mdio_read(eth, phy_addr, phy_reg);
}

static int mtk_mdio_reset(struct mii_bus *bus)
{
	/* The mdiobus_register will trigger a reset pulse when enabling Bus reset,
	 * we just need to wait until device ready.
	 */
	mdelay(20);

	return 0;
}

static int mt7621_gmac0_rgmii_adjust(struct mtk_eth *eth,
				     phy_interface_t interface)
{
	u32 val = 0;

	/* Check DDR memory type.
	 * Currently TRGMII mode with DDR2 memory is not supported.
	 */
	regmap_read(eth->ethsys, ETHSYS_SYSCFG, &val);
	if (interface == PHY_INTERFACE_MODE_TRGMII &&
	    val & SYSCFG_DRAM_TYPE_DDR2) {
		dev_err(eth->dev,
			"TRGMII mode with DDR2 memory is not supported!\n");
		return -EOPNOTSUPP;
	}

	val = (interface == PHY_INTERFACE_MODE_TRGMII) ?
		ETHSYS_TRGMII_MT7621_DDR_PLL : 0;

	regmap_update_bits(eth->ethsys, ETHSYS_CLKCFG0,
			   ETHSYS_TRGMII_MT7621_MASK, val);

	return 0;
}

static void mtk_gmac0_rgmii_adjust(struct mtk_eth *eth,
				   phy_interface_t interface, int speed)
{
	u32 val;
	int ret;

	if (interface == PHY_INTERFACE_MODE_TRGMII) {
		mtk_w32(eth, TRGMII_MODE, INTF_MODE);
		val = 500000000;
		ret = clk_set_rate(eth->clks[MTK_CLK_TRGPLL], val);
		if (ret)
			dev_err(eth->dev, "Failed to set trgmii pll: %d\n", ret);
		return;
	}

	val = (speed == SPEED_1000) ?
		INTF_MODE_RGMII_1000 : INTF_MODE_RGMII_10_100;
	mtk_w32(eth, val, INTF_MODE);

	regmap_update_bits(eth->ethsys, ETHSYS_CLKCFG0,
			   ETHSYS_TRGMII_CLK_SEL362_5,
			   ETHSYS_TRGMII_CLK_SEL362_5);

	val = (speed == SPEED_1000) ? 250000000 : 500000000;
	ret = clk_set_rate(eth->clks[MTK_CLK_TRGPLL], val);
	if (ret)
		dev_err(eth->dev, "Failed to set trgmii pll: %d\n", ret);

	val = (speed == SPEED_1000) ?
		RCK_CTRL_RGMII_1000 : RCK_CTRL_RGMII_10_100;
	mtk_w32(eth, val, TRGMII_RCK_CTRL);

	val = (speed == SPEED_1000) ?
		TCK_CTRL_RGMII_1000 : TCK_CTRL_RGMII_10_100;
	mtk_w32(eth, val, TRGMII_TCK_CTRL);
}

static void mtk_setup_bridge_switch(struct mtk_eth *eth)
{
	int val;

	/* Force Port1 XGMAC Link Up */
	val = mtk_r32(eth, MTK_XGMAC_STS(MTK_GMAC1_ID));
	mtk_w32(eth, val | MTK_XGMAC_FORCE_MODE(MTK_GMAC1_ID),
		MTK_XGMAC_STS(MTK_GMAC1_ID));

	/* Adjust GSW bridge IPG to 11*/
	val = mtk_r32(eth, MTK_GSW_CFG);
	val &= ~(GSWTX_IPG_MASK | GSWRX_IPG_MASK);
	val |= (GSW_IPG_11 << GSWTX_IPG_SHIFT) |
	       (GSW_IPG_11 << GSWRX_IPG_SHIFT);
	mtk_w32(eth, val, MTK_GSW_CFG);
}

static bool mtk_check_gmac23_idle(struct mtk_mac *mac)
{
	u32 mac_fsm, gdm_fsm;

	mac_fsm = mtk_r32(mac->hw, MTK_MAC_FSM(mac->id));

	switch (mac->id) {
	case MTK_GMAC2_ID:
		gdm_fsm = mtk_r32(mac->hw, MTK_FE_GDM2_FSM);
		break;
	case MTK_GMAC3_ID:
		gdm_fsm = mtk_r32(mac->hw, MTK_FE_GDM3_FSM);
		break;
	default:
		return true;
	};

	if ((mac_fsm & 0xFFFF0000) == 0x01010000 &&
	    (gdm_fsm & 0xFFFF0000) == 0x00000000)
		return true;

	return false;
}

static void mtk_setup_eee(struct mtk_mac *mac, bool enable)
{
	struct mtk_eth *eth = mac->hw;
	u32 mcr, mcr_cur;
	u32 val;

	mcr = mcr_cur = mtk_r32(eth, MTK_MAC_MCR(mac->id));
	mcr &= ~(MAC_MCR_FORCE_EEE100 | MAC_MCR_FORCE_EEE1000);

	if (enable) {
		mac->tx_lpi_enabled = 1;

		val = FIELD_PREP(MAC_EEE_WAKEUP_TIME_1000, 19) |
		      FIELD_PREP(MAC_EEE_WAKEUP_TIME_100, 33) |
		      FIELD_PREP(MAC_EEE_LPI_TXIDLE_THD,
				 mac->tx_lpi_timer) |
		      FIELD_PREP(MAC_EEE_RESV0, 14);
		mtk_w32(eth, val, MTK_MAC_EEE(mac->id));

		switch (mac->speed) {
		case SPEED_1000:
			mcr |= MAC_MCR_FORCE_EEE1000;
			break;
		case SPEED_100:
			mcr |= MAC_MCR_FORCE_EEE100;
			break;
		};
	} else {
		mac->tx_lpi_enabled = 0;

		mtk_w32(eth, 0x00000002, MTK_MAC_EEE(mac->id));
	}

	/* Only update control register when needed! */
	if (mcr != mcr_cur)
		mtk_w32(eth, mcr, MTK_MAC_MCR(mac->id));
}

static int mtk_get_hwver(struct mtk_eth *eth)
{
	struct device_node *np;
	struct regmap *hwver;
	u32 info = 0;

	eth->hwver = MTK_HWID_V1;

	np = of_parse_phandle(eth->dev->of_node, "mediatek,hwver", 0);
	if (!np)
		return -EINVAL;

	hwver = syscon_node_to_regmap(np);
	if (IS_ERR(hwver))
		return PTR_ERR(hwver);

	regmap_read(hwver, 0x8, &info);

	if (MTK_HAS_CAPS(eth->soc->caps, MTK_NETSYS_V3))
		eth->hwver = FIELD_GET(HWVER_BIT_NETSYS_3, info);
	else
		eth->hwver = FIELD_GET(HWVER_BIT_NETSYS_1_2, info);

	of_node_put(np);

	return 0;
}

static struct phylink_pcs *mtk_mac_select_pcs(struct phylink_config *config,
					      phy_interface_t interface)
{
	struct mtk_mac *mac = container_of(config, struct mtk_mac,
					   phylink_config);
	struct mtk_eth *eth = mac->hw;
	unsigned int sid;

	if (interface == PHY_INTERFACE_MODE_SGMII ||
	    phy_interface_mode_is_8023z(interface)) {
		sid = (MTK_HAS_CAPS(eth->soc->caps, MTK_SHARED_SGMII)) ?
		       0 : mtk_mac2xgmii_id(eth, mac->id);

		return mtk_sgmii_select_pcs(eth->sgmii, sid);
	} else if (interface == PHY_INTERFACE_MODE_USXGMII ||
		   interface == PHY_INTERFACE_MODE_10GKR ||
		   interface == PHY_INTERFACE_MODE_5GBASER) {
		if (MTK_HAS_CAPS(eth->soc->caps, MTK_NETSYS_V3) &&
		    mac->id != MTK_GMAC1_ID) {
			sid = mtk_mac2xgmii_id(eth, mac->id);

			return mtk_usxgmii_select_pcs(eth->usxgmii, sid);
		}
	}

	return NULL;
}

static int mtk_mac_prepare(struct phylink_config *config, unsigned int mode,
			   phy_interface_t iface)
{
	struct mtk_mac *mac = container_of(config, struct mtk_mac,
					   phylink_config);
	u32 val;

	if (mac->type == MTK_XGDM_TYPE && mac->id != MTK_GMAC1_ID) {
		val = mtk_r32(mac->hw, MTK_XMAC_MCR(mac->id));
		val &= 0xfffffff0;
		val |= XMAC_MCR_TRX_DISABLE;
		mtk_w32(mac->hw, val, MTK_XMAC_MCR(mac->id));

		val = mtk_r32(mac->hw, MTK_XGMAC_STS(mac->id));
		val |= MTK_XGMAC_FORCE_MODE(mac->id);
		val &= ~MTK_XGMAC_FORCE_LINK(mac->id);
		mtk_w32(mac->hw, val, MTK_XGMAC_STS(mac->id));
	}

	return 0;
}

static void mtk_mac_config(struct phylink_config *config, unsigned int mode,
			   const struct phylink_link_state *state)
{
	struct mtk_mac *mac = container_of(config, struct mtk_mac,
					   phylink_config);
	struct mtk_eth *eth = mac->hw;
	u32 i;
	int val = 0, ge_mode, err = 0;
	unsigned int mac_type = mac->type;

	/* MT76x8 has no hardware settings between for the MAC */
	if (!MTK_HAS_CAPS(eth->soc->caps, MTK_SOC_MT7628) &&
	    mac->interface != state->interface) {
		/* Setup soc pin functions */
		switch (state->interface) {
		case PHY_INTERFACE_MODE_TRGMII:
			if (mac->id)
				goto err_phy;
			if (!MTK_HAS_CAPS(mac->hw->soc->caps,
					  MTK_GMAC1_TRGMII))
				goto err_phy;
			/* fall through */
		case PHY_INTERFACE_MODE_RGMII_TXID:
		case PHY_INTERFACE_MODE_RGMII_RXID:
		case PHY_INTERFACE_MODE_RGMII_ID:
		case PHY_INTERFACE_MODE_RGMII:
		case PHY_INTERFACE_MODE_MII:
		case PHY_INTERFACE_MODE_REVMII:
		case PHY_INTERFACE_MODE_RMII:
			mac->type = MTK_GDM_TYPE;
			if (MTK_HAS_CAPS(eth->soc->caps, MTK_RGMII)) {
				err = mtk_gmac_rgmii_path_setup(eth, mac->id);
				if (err)
					goto init_err;
			}
			break;
		case PHY_INTERFACE_MODE_1000BASEX:
		case PHY_INTERFACE_MODE_2500BASEX:
		case PHY_INTERFACE_MODE_SGMII:
			mac->type = MTK_GDM_TYPE;
			if (MTK_HAS_CAPS(eth->soc->caps, MTK_SGMII)) {
				err = mtk_gmac_sgmii_path_setup(eth, mac->id);
				if (err)
					goto init_err;
			}
			break;
		case PHY_INTERFACE_MODE_GMII:
			mac->type = MTK_GDM_TYPE;
			if (MTK_HAS_CAPS(eth->soc->caps, MTK_GEPHY)) {
				err = mtk_gmac_gephy_path_setup(eth, mac->id);
				if (err)
					goto init_err;
			}
			break;
		case PHY_INTERFACE_MODE_XGMII:
			mac->type = MTK_XGDM_TYPE;
			if (MTK_HAS_CAPS(eth->soc->caps, MTK_XGMII)) {
				err = mtk_gmac_xgmii_path_setup(eth, mac->id);
				if (err)
					goto init_err;
			}
			break;
		case PHY_INTERFACE_MODE_USXGMII:
		case PHY_INTERFACE_MODE_10GKR:
		case PHY_INTERFACE_MODE_5GBASER:
			mac->type = MTK_XGDM_TYPE;
			if (MTK_HAS_CAPS(eth->soc->caps, MTK_USXGMII)) {
				err = mtk_gmac_usxgmii_path_setup(eth, mac->id);
				if (err)
					goto init_err;
			}
			break;
		default:
			goto err_phy;
		}

		/* Setup clock for 1st gmac */
		if (!mac->id && state->interface != PHY_INTERFACE_MODE_SGMII &&
		    !phy_interface_mode_is_8023z(state->interface) &&
		    MTK_HAS_CAPS(mac->hw->soc->caps, MTK_GMAC1_TRGMII)) {
			if (MTK_HAS_CAPS(mac->hw->soc->caps,
					 MTK_TRGMII_MT7621_CLK)) {
				if (mt7621_gmac0_rgmii_adjust(mac->hw,
							      state->interface))
					goto err_phy;
			} else {
				mtk_gmac0_rgmii_adjust(mac->hw,
						       state->interface,
						       state->speed);

				/* mt7623_pad_clk_setup */
				for (i = 0 ; i < NUM_TRGMII_CTRL; i++)
					mtk_w32(mac->hw,
						TD_DM_DRVP(8) | TD_DM_DRVN(8),
						TRGMII_TD_ODT(i));

				/* Assert/release MT7623 RXC reset */
				mtk_m32(mac->hw, 0, RXC_RST | RXC_DQSISEL,
					TRGMII_RCK_CTRL);
				mtk_m32(mac->hw, RXC_RST, 0, TRGMII_RCK_CTRL);
			}
		}

		ge_mode = 0;
		switch (state->interface) {
		case PHY_INTERFACE_MODE_MII:
		case PHY_INTERFACE_MODE_GMII:
			ge_mode = 1;
			break;
		case PHY_INTERFACE_MODE_REVMII:
			ge_mode = 2;
			break;
		case PHY_INTERFACE_MODE_RMII:
			if (mac->id)
				goto err_phy;
			ge_mode = 3;
			break;
		default:
			break;
		}

		/* put the gmac into the right mode */
		spin_lock(&eth->syscfg0_lock);
		regmap_read(eth->ethsys, ETHSYS_SYSCFG0, &val);
		val &= ~SYSCFG0_GE_MODE(SYSCFG0_GE_MASK, mac->id);
		val |= SYSCFG0_GE_MODE(ge_mode, mac->id);
		regmap_write(eth->ethsys, ETHSYS_SYSCFG0, val);
		spin_unlock(&eth->syscfg0_lock);

		mac->interface = state->interface;
	}

	/* SGMII */
	if (state->interface == PHY_INTERFACE_MODE_SGMII ||
	    phy_interface_mode_is_8023z(state->interface)) {
		/* The path GMAC to SGMII will be enabled once the SGMIISYS is
		 * being setup done.
		 */
		spin_lock(&eth->syscfg0_lock);
		regmap_read(eth->ethsys, ETHSYS_SYSCFG0, &val);

		regmap_update_bits(eth->ethsys, ETHSYS_SYSCFG0,
				   SYSCFG0_SGMII_MASK,
				   ~(u32)SYSCFG0_SGMII_MASK);

		/* Save the syscfg0 value for mac_finish */
		mac->syscfg0 = val;
		spin_unlock(&eth->syscfg0_lock);
	} else if (state->interface == PHY_INTERFACE_MODE_USXGMII ||
		   state->interface == PHY_INTERFACE_MODE_10GKR ||
		   state->interface == PHY_INTERFACE_MODE_5GBASER) {
		/* Nothing to do */
	} else if (phylink_autoneg_inband(mode)) {
		dev_err(eth->dev,
			"In-band mode not supported in non SGMII mode!\n");
		return;
	}

	/* Setup gmac */
	if (mac->type == MTK_XGDM_TYPE) {
		mtk_w32(mac->hw, MTK_GDMA_XGDM_SEL, MTK_GDMA_EG_CTRL(mac->id));
		mtk_w32(mac->hw, MAC_MCR_FORCE_LINK_DOWN, MTK_MAC_MCR(mac->id));

		if (MTK_HAS_CAPS(eth->soc->caps, MTK_NETSYS_V3)) {
			if (mac->id == MTK_GMAC1_ID)
				mtk_setup_bridge_switch(eth);
		}
	} else if (mac->type == MTK_GDM_TYPE) {
		val = mtk_r32(eth, MTK_GDMA_EG_CTRL(mac->id));
		mtk_w32(eth, val & ~MTK_GDMA_XGDM_SEL,
			MTK_GDMA_EG_CTRL(mac->id));

		/* FIXME: In current hardware design, we have to reset FE
		 * when swtiching XGDM to GDM. Therefore, here trigger an SER
		 * to let GDM go back to the initial state.
		 */
		if (mac->type != mac_type && !mtk_check_gmac23_idle(mac)) {
			if (!test_bit(MTK_RESETTING, &mac->hw->state)) {
				atomic_inc(&force);
				schedule_work(&eth->pending_work);
			}
		}
	}

	return;

err_phy:
	dev_err(eth->dev, "%s: GMAC%d mode %s not supported!\n", __func__,
		mac->id, phy_modes(state->interface));
	return;

init_err:
	dev_err(eth->dev, "%s: GMAC%d mode %s err: %d!\n", __func__,
		mac->id, phy_modes(state->interface), err);
}

static int mtk_mac_finish(struct phylink_config *config, unsigned int mode,
			  phy_interface_t interface)
{
	struct mtk_mac *mac = container_of(config, struct mtk_mac,
					   phylink_config);
	struct mtk_eth *eth = mac->hw;

	/* Enable SGMII */
	if (interface == PHY_INTERFACE_MODE_SGMII ||
	    phy_interface_mode_is_8023z(interface)) {
		spin_lock(&eth->syscfg0_lock);
		regmap_update_bits(eth->ethsys, ETHSYS_SYSCFG0,
				   SYSCFG0_SGMII_MASK, mac->syscfg0);
		spin_unlock(&eth->syscfg0_lock);
	}

	return 0;
}

static int mtk_mac_pcs_get_state(struct phylink_config *config,
				 struct phylink_link_state *state)
{
	struct mtk_mac *mac = container_of(config, struct mtk_mac,
					   phylink_config);

	if (mac->type == MTK_XGDM_TYPE) {
		u32 sts = mtk_r32(mac->hw, MTK_XGMAC_STS(mac->id));

		if (mac->id == MTK_GMAC2_ID)
			sts = sts >> 16;

		state->duplex = DUPLEX_FULL;

		switch (FIELD_GET(MTK_USXGMII_PCS_MODE, sts)) {
		case 0:
			state->speed = SPEED_10000;
			break;
		case 1:
			state->speed = SPEED_5000;
			break;
		case 2:
			state->speed = SPEED_2500;
			break;
		case 3:
			state->speed = SPEED_1000;
			break;
		}

		state->interface = mac->interface;
		state->link = FIELD_GET(MTK_USXGMII_PCS_LINK, sts);
	} else if (mac->type == MTK_GDM_TYPE) {
		struct mtk_eth *eth = mac->hw;
		struct mtk_sgmii *ss = eth->sgmii;
		u32 id = mtk_mac2xgmii_id(eth, mac->id);
		u32 pmsr = mtk_r32(mac->hw, MTK_MAC_MSR(mac->id));
		u32 bm, adv, rgc3, sgm_mode;

		state->interface = mac->interface;

		regmap_read(ss->pcs[id].regmap, SGMSYS_PCS_CONTROL_1, &bm);
		if (bm & SGMII_AN_ENABLE) {
			regmap_read(ss->pcs[id].regmap,
				    SGMSYS_PCS_ADVERTISE, &adv);

			phylink_mii_c22_pcs_decode_state(
				state,
				FIELD_GET(SGMII_BMSR, bm),
				FIELD_GET(SGMII_LPA, adv));
		} else {
			state->link = !!(bm & SGMII_LINK_STATYS);

			regmap_read(ss->pcs[id].regmap,
				    SGMSYS_SGMII_MODE, &sgm_mode);

			switch (sgm_mode & SGMII_SPEED_MASK) {
			case SGMII_SPEED_10:
				state->speed = SPEED_10;
				break;
			case SGMII_SPEED_100:
				state->speed = SPEED_100;
				break;
			case SGMII_SPEED_1000:
				regmap_read(ss->pcs[id].regmap,
					    ss->pcs[id].ana_rgc3, &rgc3);
				rgc3 = FIELD_GET(RG_PHY_SPEED_3_125G, rgc3);
				state->speed = rgc3 ? SPEED_2500 : SPEED_1000;
				break;
			}

			if (sgm_mode & SGMII_DUPLEX_HALF)
				state->duplex = DUPLEX_HALF;
			else
				state->duplex = DUPLEX_FULL;
		}

		state->pause &= (MLO_PAUSE_RX | MLO_PAUSE_TX);
		if (pmsr & MAC_MSR_RX_FC)
			state->pause |= MLO_PAUSE_RX;
		if (pmsr & MAC_MSR_TX_FC)
			state->pause |= MLO_PAUSE_TX;
	}

	return 1;
}

static int mtk_gdm_fsm_get(struct mtk_mac *mac, u32 gdm)
{
	u32 fsm = mtk_r32(mac->hw, gdm);
	u32 ret = 0, val = 0;

	switch (mac->type) {
	case MTK_GDM_TYPE:
		ret = fsm == 0;
		break;
	case MTK_XGDM_TYPE:
		ret = fsm == 0x10000000;
		break;
	default:
		break;
	}

	if ((mac->type == MTK_XGDM_TYPE) && (mac->id != MTK_GMAC1_ID)) {
		val = mtk_r32(mac->hw, MTK_MAC_FSM(mac->id));
		if ((val == 0x02010100) || (val == 0x01010100)) {
			ret = (mac->interface == PHY_INTERFACE_MODE_XGMII) ?
				((fsm & 0x0fffffff) == 0) : ((fsm & 0x00ffffff) == 0);
		} else
			ret = 0;
	}

	return ret;
}

static void mtk_gdm_fsm_poll(struct mtk_mac *mac)
{
	u32 gdm = 0, i = 0;

	switch (mac->id) {
	case MTK_GMAC1_ID:
		gdm = MTK_FE_GDM1_FSM;
		break;
	case MTK_GMAC2_ID:
		gdm = MTK_FE_GDM2_FSM;
		break;
	case MTK_GMAC3_ID:
		gdm = MTK_FE_GDM3_FSM;
		break;
	default:
		pr_info("%s mac id invalid", __func__);
		break;
	}

	while (i < 3) {
		if (mtk_gdm_fsm_get(mac, gdm))
			break;
		msleep(500);
		i++;
	}

	if (i == 3)
		pr_info("%s fsm invalid", __func__);
}

static void mtk_pse_port_link_set(struct mtk_mac *mac, bool up,
				  phy_interface_t interface)
{
	u32 fe_glo_cfg, val = 0;

	if (!up && interface == PHY_INTERFACE_MODE_XGMII) {
		void __iomem *base;

		base = ioremap(0x0F0CFB00, SZ_4K);
		if (base) {
			/* wait for internal 2.5G PHY to turn off */
			usleep_range(100, 1000);
			/* enable the XGMAC clock for 10 msecs to
			 * flush the packets.
			 */
			writel(readl(base) | BIT(9), base);
			usleep_range(10000, 11000);
			writel(readl(base) & ~BIT(9), base);
			iounmap(base);
		}
	}

	fe_glo_cfg = mtk_r32(mac->hw, MTK_FE_GLO_CFG(mac->id));
	switch (mac->id) {
	case MTK_GMAC1_ID:
		val = MTK_FE_LINK_DOWN_P1;
		break;
	case MTK_GMAC2_ID:
		val = MTK_FE_LINK_DOWN_P2;
		break;
	case MTK_GMAC3_ID:
		val = MTK_FE_LINK_DOWN_P15;
		break;
	}

	if (!up)
		fe_glo_cfg |= val;
	else
		fe_glo_cfg &= ~val;

	mtk_w32(mac->hw, fe_glo_cfg, MTK_FE_GLO_CFG(mac->id));
	mtk_gdm_fsm_poll(mac);
}

static void mtk_mac_link_down(struct phylink_config *config, unsigned int mode,
			      phy_interface_t interface)
{
	struct mtk_mac *mac = container_of(config, struct mtk_mac,
					   phylink_config);
	struct mtk_eth *eth = mac->hw;
	unsigned int id;
	u32 mcr, sts;

	mtk_pse_port_link_set(mac, false, interface);
	if (mac->type == MTK_GDM_TYPE) {
		mcr = mtk_r32(mac->hw, MTK_MAC_MCR(mac->id));
		mcr &= ~(MAC_MCR_TX_EN | MAC_MCR_RX_EN | MAC_MCR_FORCE_LINK);
		mtk_w32(mac->hw, mcr, MTK_MAC_MCR(mac->id));
	} else if (mac->type == MTK_XGDM_TYPE && mac->id != MTK_GMAC1_ID) {
		struct mtk_usxgmii_pcs *mpcs;

		mcr = mtk_r32(mac->hw, MTK_XMAC_MCR(mac->id));
		mcr &= 0xfffffff0;
		mcr |= XMAC_MCR_TRX_DISABLE;
		mtk_w32(mac->hw, mcr, MTK_XMAC_MCR(mac->id));

		sts = mtk_r32(mac->hw, MTK_XGMAC_STS(mac->id));
		sts &= ~MTK_XGMAC_FORCE_LINK(mac->id);
		mtk_w32(mac->hw, sts, MTK_XGMAC_STS(mac->id));

		id = mtk_mac2xgmii_id(eth, mac->id);
		mpcs = &eth->usxgmii->pcs[id];
	}
}

static void mtk_set_queue_speed(struct mtk_eth *eth, unsigned int idx,
				int speed)
{
	const struct mtk_soc_data *soc = eth->soc;
	u32 val;

	if (!MTK_HAS_CAPS(soc->caps, MTK_QDMA))
		return;

	val = MTK_QTX_SCH_MIN_RATE_EN |
	      /* minimum: 10 Mbps */
	      FIELD_PREP(MTK_QTX_SCH_MIN_RATE_MAN, 1) |
	      FIELD_PREP(MTK_QTX_SCH_MIN_RATE_EXP, 4) |
	      MTK_QTX_SCH_LEAKY_BUCKET_SIZE;

	if (MTK_HAS_CAPS(eth->soc->caps, MTK_NETSYS_V1))
		val |= MTK_QTX_SCH_LEAKY_BUCKET_EN;

	if (IS_ENABLED(CONFIG_SOC_MT7621)) {
		switch (speed) {
		case SPEED_10:
			val |= MTK_QTX_SCH_MAX_RATE_EN |
			       FIELD_PREP(MTK_QTX_SCH_MAX_RATE_MAN, 103) |
			       FIELD_PREP(MTK_QTX_SCH_MAX_RATE_EXP, 2) |
			       FIELD_PREP(MTK_QTX_SCH_MAX_RATE_WEIGHT, 1);
			break;
		case SPEED_100:
			val |= MTK_QTX_SCH_MAX_RATE_EN |
			       FIELD_PREP(MTK_QTX_SCH_MAX_RATE_MAN, 103) |
			       FIELD_PREP(MTK_QTX_SCH_MAX_RATE_EXP, 3);
			       FIELD_PREP(MTK_QTX_SCH_MAX_RATE_WEIGHT, 1);
			break;
		case SPEED_1000:
			val |= MTK_QTX_SCH_MAX_RATE_EN |
			       FIELD_PREP(MTK_QTX_SCH_MAX_RATE_MAN, 105) |
			       FIELD_PREP(MTK_QTX_SCH_MAX_RATE_EXP, 4) |
			       FIELD_PREP(MTK_QTX_SCH_MAX_RATE_WEIGHT, 10);
			break;
		default:
			break;
		}
	} else {
		switch (speed) {
		case SPEED_10:
			val |= MTK_QTX_SCH_MAX_RATE_EN |
			       FIELD_PREP(MTK_QTX_SCH_MAX_RATE_MAN, 1) |
			       FIELD_PREP(MTK_QTX_SCH_MAX_RATE_EXP, 4) |
			       FIELD_PREP(MTK_QTX_SCH_MAX_RATE_WEIGHT, 1);
			break;
		case SPEED_100:
			val |= MTK_QTX_SCH_MAX_RATE_EN |
			       FIELD_PREP(MTK_QTX_SCH_MAX_RATE_MAN, 1) |
			       FIELD_PREP(MTK_QTX_SCH_MAX_RATE_EXP, 5);
			       FIELD_PREP(MTK_QTX_SCH_MAX_RATE_WEIGHT, 1);
			break;
		case SPEED_1000:
			val |= MTK_QTX_SCH_MAX_RATE_EN |
			       FIELD_PREP(MTK_QTX_SCH_MAX_RATE_MAN, 10) |
			       FIELD_PREP(MTK_QTX_SCH_MAX_RATE_EXP, 5) |
			       FIELD_PREP(MTK_QTX_SCH_MAX_RATE_WEIGHT, 10);
			break;
		case SPEED_2500:
			val |= MTK_QTX_SCH_MAX_RATE_EN |
			       FIELD_PREP(MTK_QTX_SCH_MAX_RATE_MAN, 25) |
			       FIELD_PREP(MTK_QTX_SCH_MAX_RATE_EXP, 5) |
			       FIELD_PREP(MTK_QTX_SCH_MAX_RATE_WEIGHT, 10);
			break;
		case SPEED_10000:
			val |= MTK_QTX_SCH_MAX_RATE_EN |
			       FIELD_PREP(MTK_QTX_SCH_MAX_RATE_MAN, 100) |
			       FIELD_PREP(MTK_QTX_SCH_MAX_RATE_EXP, 5) |
			       FIELD_PREP(MTK_QTX_SCH_MAX_RATE_WEIGHT, 10);
			break;
		default:
			break;
		}
	}

	mtk_w32(eth, (idx / MTK_QTX_PER_PAGE) & MTK_QTX_CFG_PAGE, MTK_QDMA_PAGE);
	mtk_w32(eth, val, MTK_QTX_SCH(idx));
}

static void mtk_mac_link_up(struct phylink_config *config,
			    struct phy_device *phy,
			    unsigned int mode, phy_interface_t interface,
			    int speed, int duplex, bool tx_pause, bool rx_pause)
{
	struct mtk_mac *mac = container_of(config, struct mtk_mac,
					   phylink_config);
	u32 mcr, mcr_cur, sts;

	mac->speed = speed;

	if (mac->type == MTK_GDM_TYPE) {
		mcr_cur = mtk_r32(mac->hw, MTK_MAC_MCR(mac->id));
		mcr = mcr_cur;
		mcr &= ~(MAC_MCR_SPEED_100 | MAC_MCR_SPEED_1000 |
			 MAC_MCR_FORCE_DPX | MAC_MCR_FORCE_TX_FC |
			 MAC_MCR_FORCE_RX_FC);
		mcr |= MAC_MCR_MAX_RX_1536 | MAC_MCR_IPG_CFG | MAC_MCR_FORCE_MODE |
		       MAC_MCR_BACKOFF_EN | MAC_MCR_BACKPR_EN | MAC_MCR_FORCE_LINK;

		/* Configure speed */
		switch (speed) {
		case SPEED_2500:
		case SPEED_1000:
			mcr |= MAC_MCR_SPEED_1000;
			break;
		case SPEED_100:
			mcr |= MAC_MCR_SPEED_100;
			break;
		}

		/* Configure duplex */
		if (duplex == DUPLEX_FULL ||
		    interface == PHY_INTERFACE_MODE_SGMII)
			mcr |= MAC_MCR_FORCE_DPX;
		else if (interface == PHY_INTERFACE_MODE_GMII)
			mcr |= MAC_MCR_PRMBL_LMT_EN;

		/* Configure pause modes -
		 * phylink will avoid these for half duplex
		 */
		if (tx_pause)
			mcr |= MAC_MCR_FORCE_TX_FC;
		if (rx_pause)
			mcr |= MAC_MCR_FORCE_RX_FC;

		mcr |= MAC_MCR_TX_EN | MAC_MCR_RX_EN;

		/* Only update control register when needed! */
		if (mcr != mcr_cur)
			mtk_w32(mac->hw, mcr, MTK_MAC_MCR(mac->id));

		if (mode == MLO_AN_PHY && phy)
			mtk_setup_eee(mac, phy_init_eee(phy, false) >= 0);
	} else if (mac->type == MTK_XGDM_TYPE && mac->id != MTK_GMAC1_ID) {
		if (mode == MLO_AN_INBAND)
			mdelay(1000);

		/* Eliminate the interference(before link-up) caused by PHY noise */
		mtk_m32(mac->hw, XMAC_LOGIC_RST, 0x0, MTK_XMAC_LOGIC_RST(mac->id));
		mdelay(20);
		mtk_m32(mac->hw, XMAC_GLB_CNTCLR, 0x1, MTK_XMAC_CNT_CTRL(mac->id));

		sts = mtk_r32(mac->hw, MTK_XGMAC_STS(mac->id));
		sts |= MTK_XGMAC_FORCE_LINK(mac->id);
		mtk_w32(mac->hw, sts, MTK_XGMAC_STS(mac->id));

		mcr = mtk_r32(mac->hw, MTK_XMAC_MCR(mac->id));

		mcr &= ~(XMAC_MCR_FORCE_TX_FC |	XMAC_MCR_FORCE_RX_FC);
		/* Configure pause modes -
		 * phylink will avoid these for half duplex
		 */
		if (tx_pause)
			mcr |= XMAC_MCR_FORCE_TX_FC;
		if (rx_pause)
			mcr |= XMAC_MCR_FORCE_RX_FC;

		mcr &= ~(XMAC_MCR_TRX_DISABLE);
		mtk_w32(mac->hw, mcr, MTK_XMAC_MCR(mac->id));
	}
	mtk_pse_port_link_set(mac, true, interface);
}

static void mtk_validate(struct phylink_config *config,
			 unsigned long *supported,
			 struct phylink_link_state *state)
{
	struct mtk_mac *mac = container_of(config, struct mtk_mac,
					   phylink_config);
	__ETHTOOL_DECLARE_LINK_MODE_MASK(mask) = { 0, };

	if (state->interface != PHY_INTERFACE_MODE_NA &&
	    state->interface != PHY_INTERFACE_MODE_MII &&
	    state->interface != PHY_INTERFACE_MODE_GMII &&
	    !(MTK_HAS_CAPS(mac->hw->soc->caps, MTK_RGMII) &&
	      phy_interface_mode_is_rgmii(state->interface)) &&
	    !(MTK_HAS_CAPS(mac->hw->soc->caps, MTK_TRGMII) &&
	      !mac->id && state->interface == PHY_INTERFACE_MODE_TRGMII) &&
	    !(MTK_HAS_CAPS(mac->hw->soc->caps, MTK_SGMII) &&
	      (state->interface == PHY_INTERFACE_MODE_SGMII ||
	       phy_interface_mode_is_8023z(state->interface))) &&
	    !(MTK_HAS_CAPS(mac->hw->soc->caps, MTK_XGMII) &&
	      (state->interface == PHY_INTERFACE_MODE_XGMII)) &&
	    !(MTK_HAS_CAPS(mac->hw->soc->caps, MTK_USXGMII) &&
	      (state->interface == PHY_INTERFACE_MODE_USXGMII)) &&
	    !(MTK_HAS_CAPS(mac->hw->soc->caps, MTK_USXGMII) &&
	      (state->interface == PHY_INTERFACE_MODE_10GKR))) {
		linkmode_zero(supported);
		return;
	}

	phylink_set_port_modes(mask);
	phylink_set(mask, Autoneg);

	switch (state->interface) {
	case PHY_INTERFACE_MODE_USXGMII:
	case PHY_INTERFACE_MODE_10GKR:
		phylink_set(mask, 10000baseKR_Full);
		phylink_set(mask, 10000baseT_Full);
		phylink_set(mask, 10000baseCR_Full);
		phylink_set(mask, 10000baseSR_Full);
		phylink_set(mask, 10000baseLR_Full);
		phylink_set(mask, 10000baseLRM_Full);
		phylink_set(mask, 10000baseER_Full);
		phylink_set(mask, 100baseT_Half);
		phylink_set(mask, 100baseT_Full);
		phylink_set(mask, 1000baseT_Half);
		phylink_set(mask, 1000baseT_Full);
		phylink_set(mask, 1000baseX_Full);
		phylink_set(mask, 2500baseT_Full);
		phylink_set(mask, 5000baseT_Full);
		break;
	case PHY_INTERFACE_MODE_TRGMII:
		phylink_set(mask, 1000baseT_Full);
		break;
	case PHY_INTERFACE_MODE_XGMII:
		/* fall through */
	case PHY_INTERFACE_MODE_1000BASEX:
		phylink_set(mask, 1000baseX_Full);
		/* fall through */
	case PHY_INTERFACE_MODE_2500BASEX:
		phylink_set(mask, 2500baseX_Full);
		phylink_set(mask, 2500baseT_Full);
		/* fall through */
	case PHY_INTERFACE_MODE_GMII:
	case PHY_INTERFACE_MODE_RGMII:
	case PHY_INTERFACE_MODE_RGMII_ID:
	case PHY_INTERFACE_MODE_RGMII_RXID:
	case PHY_INTERFACE_MODE_RGMII_TXID:
		phylink_set(mask, 1000baseT_Half);
		/* fall through */
	case PHY_INTERFACE_MODE_SGMII:
		phylink_set(mask, 1000baseT_Full);
		phylink_set(mask, 1000baseX_Full);
		/* fall through */
	case PHY_INTERFACE_MODE_MII:
	case PHY_INTERFACE_MODE_RMII:
	case PHY_INTERFACE_MODE_REVMII:
	case PHY_INTERFACE_MODE_NA:
	default:
		phylink_set(mask, 10baseT_Half);
		phylink_set(mask, 10baseT_Full);
		phylink_set(mask, 100baseT_Half);
		phylink_set(mask, 100baseT_Full);
		break;
	}

	if (state->interface == PHY_INTERFACE_MODE_NA) {

		if (MTK_HAS_CAPS(mac->hw->soc->caps, MTK_USXGMII)) {
			phylink_set(mask, 10000baseKR_Full);
			phylink_set(mask, 10000baseT_Full);
			phylink_set(mask, 10000baseCR_Full);
			phylink_set(mask, 10000baseSR_Full);
			phylink_set(mask, 10000baseLR_Full);
			phylink_set(mask, 10000baseLRM_Full);
			phylink_set(mask, 10000baseER_Full);
			phylink_set(mask, 1000baseKX_Full);
			phylink_set(mask, 1000baseT_Full);
			phylink_set(mask, 1000baseX_Full);
			phylink_set(mask, 2500baseX_Full);
			phylink_set(mask, 2500baseT_Full);
			phylink_set(mask, 5000baseT_Full);
		}
		if (MTK_HAS_CAPS(mac->hw->soc->caps, MTK_SGMII)) {
			phylink_set(mask, 1000baseT_Full);
			phylink_set(mask, 1000baseX_Full);
			phylink_set(mask, 2500baseX_Full);
		}
		if (MTK_HAS_CAPS(mac->hw->soc->caps, MTK_RGMII)) {
			phylink_set(mask, 1000baseT_Full);
			phylink_set(mask, 1000baseT_Half);
			phylink_set(mask, 1000baseX_Full);
		}
		if (MTK_HAS_CAPS(mac->hw->soc->caps, MTK_GEPHY)) {
			phylink_set(mask, 1000baseT_Full);
			phylink_set(mask, 1000baseT_Half);
		}
	}

	if (mac->type == MTK_XGDM_TYPE) {
		phylink_clear(mask, 10baseT_Half);
		phylink_clear(mask, 100baseT_Half);
		phylink_clear(mask, 1000baseT_Half);
	}

	phylink_set(mask, Pause);
	phylink_set(mask, Asym_Pause);

	linkmode_and(supported, supported, mask);
	linkmode_and(state->advertising, state->advertising, mask);

	/* We can only operate at 2500BaseX or 1000BaseX. If requested
	 * to advertise both, only report advertising at 2500BaseX.
	 */
	phylink_helper_basex_speed(state);
}

static const struct phylink_mac_ops mtk_phylink_ops = {
	.validate = mtk_validate,
	.mac_select_pcs = mtk_mac_select_pcs,
	.mac_link_state = mtk_mac_pcs_get_state,
	.mac_prepare = mtk_mac_prepare,
	.mac_config = mtk_mac_config,
	.mac_finish = mtk_mac_finish,
	.mac_link_down = mtk_mac_link_down,
	.mac_link_up = mtk_mac_link_up,
};

static int mtk_mdc_init(struct mtk_eth *eth)
{
	struct device_node *mii_np;
	int max_clk = 2500000, divider;
	int ret = 0;
	u32 val;

	mii_np = of_get_child_by_name(eth->dev->of_node, "mdio-bus");
	if (!mii_np) {
		dev_err(eth->dev, "no %s child node found", "mdio-bus");
		return -ENODEV;
	}

	if (!of_device_is_available(mii_np)) {
		ret = -ENODEV;
		goto err_put_node;
	}

	if (!of_property_read_u32(mii_np, "clock-frequency", &val)) {
		if (val > MDC_MAX_FREQ ||
		    val < MDC_MAX_FREQ / MDC_MAX_DIVIDER) {
			dev_err(eth->dev, "MDIO clock frequency out of range");
			ret = -EINVAL;
			goto err_put_node;
		}
		max_clk = val;
	}

	divider = min_t(unsigned int, DIV_ROUND_UP(MDC_MAX_FREQ, max_clk), 63);

	/* Configure MDC Turbo Mode */
	if (MTK_HAS_CAPS(eth->soc->caps, MTK_NETSYS_V3)) {
		val = mtk_r32(eth, MTK_MAC_MISC);
		val |= MISC_MDC_TURBO;
		mtk_w32(eth, val, MTK_MAC_MISC);
	} else {
		val = mtk_r32(eth, MTK_PPSC);
		val |= PPSC_MDC_TURBO;
		mtk_w32(eth, val, MTK_PPSC);
	}

	/* Configure MDC Divider */
	val = mtk_r32(eth, MTK_PPSC);
	val &= ~PPSC_MDC_CFG;
	val |= FIELD_PREP(PPSC_MDC_CFG, divider);
	mtk_w32(eth, val, MTK_PPSC);

	dev_info(eth->dev, "MDC is running on %d Hz\n", MDC_MAX_FREQ / divider);

err_put_node:
	of_node_put(mii_np);
	return ret;
}

static int mtk_mdio_init(struct mtk_eth *eth)
{
	struct device_node *mii_np;
	int ret;

	mii_np = of_get_child_by_name(eth->dev->of_node, "mdio-bus");
	if (!mii_np) {
		dev_err(eth->dev, "no %s child node found", "mdio-bus");
		return -ENODEV;
	}

	if (!of_device_is_available(mii_np)) {
		ret = -ENODEV;
		goto err_put_node;
	}

	eth->mii_bus = devm_mdiobus_alloc(eth->dev);
	if (!eth->mii_bus) {
		ret = -ENOMEM;
		goto err_put_node;
	}

	eth->mii_bus->name = "mdio";
	eth->mii_bus->read = mtk_mdio_read;
	eth->mii_bus->write = mtk_mdio_write;
	eth->mii_bus->reset = mtk_mdio_reset;
	eth->mii_bus->priv = eth;
	eth->mii_bus->parent = eth->dev;

	if (snprintf(eth->mii_bus->id, MII_BUS_ID_SIZE, "%pOFn", mii_np) < 0) {
		ret = -ENOMEM;
		goto err_put_node;
	}

	ret = of_mdiobus_register(eth->mii_bus, mii_np);

err_put_node:
	of_node_put(mii_np);
	return ret;
}

static void mtk_mdio_cleanup(struct mtk_eth *eth)
{
	if (!eth->mii_bus)
		return;

	mdiobus_unregister(eth->mii_bus);
}

static inline void mtk_tx_irq_disable(struct mtk_eth *eth, u32 mask)
{
	unsigned long flags;
	u32 val;

	if (MTK_HAS_CAPS(eth->soc->caps, MTK_QDMA)) {
		spin_lock_irqsave(&eth->tx_irq_lock, flags);
		val = mtk_r32(eth, eth->soc->reg_map->tx_irq_mask);
		mtk_w32(eth, val & ~mask, eth->soc->reg_map->tx_irq_mask);
		spin_unlock_irqrestore(&eth->tx_irq_lock, flags);
	} else {
		spin_lock_irqsave(&eth->txrx_irq_lock, flags);
		val = mtk_r32(eth, eth->soc->reg_map->pdma.irq_mask);
		mtk_w32(eth, val & ~mask, eth->soc->reg_map->pdma.irq_mask);
		spin_unlock_irqrestore(&eth->txrx_irq_lock, flags);
	}
}

static inline void mtk_tx_irq_enable(struct mtk_eth *eth, u32 mask)
{
	unsigned long flags;
	u32 val;

	if (MTK_HAS_CAPS(eth->soc->caps, MTK_QDMA)) {
		spin_lock_irqsave(&eth->tx_irq_lock, flags);
		val = mtk_r32(eth, eth->soc->reg_map->tx_irq_mask);
		mtk_w32(eth, val | mask, eth->soc->reg_map->tx_irq_mask);
		spin_unlock_irqrestore(&eth->tx_irq_lock, flags);
	} else {
		spin_lock_irqsave(&eth->txrx_irq_lock, flags);
		val = mtk_r32(eth, eth->soc->reg_map->pdma.irq_mask);
		mtk_w32(eth, val | mask, eth->soc->reg_map->pdma.irq_mask);
		spin_unlock_irqrestore(&eth->txrx_irq_lock, flags);
	}
}

static inline void mtk_rx_irq_disable(struct mtk_eth *eth, u32 mask)
{
	unsigned long flags;
	u32 val;
	spinlock_t *irq_lock;

	if (MTK_HAS_CAPS(eth->soc->caps, MTK_QDMA))
		irq_lock = &eth->rx_irq_lock;
	else
		irq_lock = &eth->txrx_irq_lock;

	spin_lock_irqsave(irq_lock, flags);
	val = mtk_r32(eth, eth->soc->reg_map->pdma.irq_mask);
	mtk_w32(eth, val & ~mask, eth->soc->reg_map->pdma.irq_mask);
	spin_unlock_irqrestore(irq_lock, flags);
}

static inline void mtk_rx_irq_enable(struct mtk_eth *eth, u32 mask)
{
	unsigned long flags;
	u32 val;
	spinlock_t *irq_lock;

	if (MTK_HAS_CAPS(eth->soc->caps, MTK_QDMA))
		irq_lock = &eth->rx_irq_lock;
	else
		irq_lock = &eth->txrx_irq_lock;

	spin_lock_irqsave(irq_lock, flags);
	val = mtk_r32(eth, eth->soc->reg_map->pdma.irq_mask);
	mtk_w32(eth, val | mask, eth->soc->reg_map->pdma.irq_mask);
	spin_unlock_irqrestore(irq_lock, flags);
}

static int mtk_set_mac_address(struct net_device *dev, void *p)
{
	int ret = eth_mac_addr(dev, p);
	struct mtk_mac *mac = netdev_priv(dev);
	struct mtk_eth *eth = mac->hw;
	const char *macaddr = dev->dev_addr;

	if (ret)
		return ret;

	if (unlikely(test_bit(MTK_RESETTING, &mac->hw->state)))
		return -EBUSY;

	spin_lock_bh(&mac->hw->page_lock);
	if (MTK_HAS_CAPS(eth->soc->caps, MTK_SOC_MT7628)) {
		mtk_w32(mac->hw, (macaddr[0] << 8) | macaddr[1],
			MT7628_SDM_MAC_ADRH);
		mtk_w32(mac->hw, (macaddr[2] << 24) | (macaddr[3] << 16) |
			(macaddr[4] << 8) | macaddr[5],
			MT7628_SDM_MAC_ADRL);
	} else {
		mtk_w32(mac->hw, (macaddr[0] << 8) | macaddr[1],
			MTK_GDMA_MAC_ADRH(mac->id));
		mtk_w32(mac->hw, (macaddr[2] << 24) | (macaddr[3] << 16) |
			(macaddr[4] << 8) | macaddr[5],
			MTK_GDMA_MAC_ADRL(mac->id));
	}
	spin_unlock_bh(&mac->hw->page_lock);

	return 0;
}

void mtk_stats_update_mac(struct mtk_mac *mac)
{
	struct mtk_eth *eth = mac->hw;
	const struct mtk_reg_map *reg_map = eth->soc->reg_map;
	struct mtk_hw_stats *hw_stats = mac->hw_stats;
	unsigned int offs = hw_stats->reg_offset;
	u64 stats;

	u64_stats_update_begin(&hw_stats->syncp);

	hw_stats->rx_bytes += mtk_r32(mac->hw, reg_map->gdm1_cnt + offs);
	stats =  mtk_r32(mac->hw, reg_map->gdm1_cnt + 0x4 + offs);
	if (stats)
		hw_stats->rx_bytes += (stats << 32);
	hw_stats->rx_packets +=
		mtk_r32(mac->hw, reg_map->gdm1_cnt + 0x08 + offs);
	hw_stats->rx_overflow +=
		mtk_r32(mac->hw, reg_map->gdm1_cnt + 0x10 + offs);
	hw_stats->rx_fcs_errors +=
		mtk_r32(mac->hw, reg_map->gdm1_cnt + 0x14 + offs);
	hw_stats->rx_short_errors +=
		mtk_r32(mac->hw, reg_map->gdm1_cnt + 0x18 + offs);
	hw_stats->rx_long_errors +=
		mtk_r32(mac->hw, reg_map->gdm1_cnt + 0x1c + offs);
	hw_stats->rx_checksum_errors +=
		mtk_r32(mac->hw, reg_map->gdm1_cnt + 0x20 + offs);
	hw_stats->rx_flow_control_packets +=
		mtk_r32(mac->hw, reg_map->gdm1_cnt + 0x24 + offs);

	if (MTK_HAS_CAPS(eth->soc->caps, MTK_NETSYS_V3)) {
		hw_stats->tx_skip +=
			mtk_r32(mac->hw, reg_map->gdm1_cnt + 0x50 + offs);
		hw_stats->tx_collisions +=
			mtk_r32(mac->hw, reg_map->gdm1_cnt + 0x54 + offs);
		hw_stats->tx_bytes +=
			mtk_r32(mac->hw, reg_map->gdm1_cnt + 0x40 + offs);
		stats =  mtk_r32(mac->hw, reg_map->gdm1_cnt + 0x44 + offs);
		if (stats)
			hw_stats->tx_bytes += (stats << 32);
		hw_stats->tx_packets +=
			mtk_r32(mac->hw, reg_map->gdm1_cnt + 0x48 + offs);
	} else {
		hw_stats->tx_skip +=
			mtk_r32(mac->hw, reg_map->gdm1_cnt + 0x28 + offs);
		hw_stats->tx_collisions +=
			mtk_r32(mac->hw, reg_map->gdm1_cnt + 0x2c + offs);
		hw_stats->tx_bytes +=
			mtk_r32(mac->hw, reg_map->gdm1_cnt + 0x30 + offs);
		stats =  mtk_r32(mac->hw, reg_map->gdm1_cnt + 0x34 + offs);
		if (stats)
			hw_stats->tx_bytes += (stats << 32);
		hw_stats->tx_packets +=
			mtk_r32(mac->hw, reg_map->gdm1_cnt + 0x38 + offs);
	}

	u64_stats_update_end(&hw_stats->syncp);
}

static void mtk_stats_update(struct mtk_eth *eth)
{
	int i;

	for (i = 0; i < MTK_MAC_COUNT; i++) {
		if (!eth->mac[i] || !eth->mac[i]->hw_stats)
			continue;
		if (spin_trylock(&eth->mac[i]->hw_stats->stats_lock)) {
			mtk_stats_update_mac(eth->mac[i]);
			spin_unlock(&eth->mac[i]->hw_stats->stats_lock);
		}
	}
}

static void mtk_get_stats64(struct net_device *dev,
			    struct rtnl_link_stats64 *storage)
{
	struct mtk_mac *mac = netdev_priv(dev);
	struct mtk_hw_stats *hw_stats = mac->hw_stats;
	unsigned int start;

	if (netif_running(dev) && netif_device_present(dev)) {
		if (spin_trylock_bh(&hw_stats->stats_lock)) {
			mtk_stats_update_mac(mac);
			spin_unlock_bh(&hw_stats->stats_lock);
		}
	}

	do {
		start = u64_stats_fetch_begin_irq(&hw_stats->syncp);
		storage->rx_packets = hw_stats->rx_packets;
		storage->tx_packets = hw_stats->tx_packets;
		storage->rx_bytes = hw_stats->rx_bytes;
		storage->tx_bytes = hw_stats->tx_bytes;
		storage->collisions = hw_stats->tx_collisions;
		storage->rx_length_errors = hw_stats->rx_short_errors +
			hw_stats->rx_long_errors;
		storage->rx_over_errors = hw_stats->rx_overflow;
		storage->rx_crc_errors = hw_stats->rx_fcs_errors;
		storage->rx_errors = hw_stats->rx_checksum_errors;
		storage->tx_aborted_errors = hw_stats->tx_skip;
	} while (u64_stats_fetch_retry_irq(&hw_stats->syncp, start));

	storage->tx_errors = dev->stats.tx_errors;
	storage->rx_dropped = dev->stats.rx_dropped;
	storage->tx_dropped = dev->stats.tx_dropped;
}

static inline int mtk_max_frag_size(int mtu)
{
	/* make sure buf_size will be at least MTK_MAX_RX_LENGTH */
	if (mtu + MTK_RX_ETH_HLEN < MTK_MAX_RX_LENGTH)
		mtu = MTK_MAX_RX_LENGTH - MTK_RX_ETH_HLEN;

	return SKB_DATA_ALIGN(MTK_RX_HLEN + mtu) +
		SKB_DATA_ALIGN(sizeof(struct skb_shared_info));
}

static inline int mtk_max_buf_size(int frag_size)
{
	int buf_size = frag_size - NET_SKB_PAD - NET_IP_ALIGN -
		       SKB_DATA_ALIGN(sizeof(struct skb_shared_info));

	WARN_ON(buf_size < MTK_MAX_RX_LENGTH);

	return buf_size;
}

static bool mtk_rx_get_desc(struct mtk_eth *eth, struct mtk_rx_dma_v2 *rxd,
			    struct mtk_rx_dma_v2 *dma_rxd)
{
	rxd->rxd2 = READ_ONCE(dma_rxd->rxd2);
	if (!(rxd->rxd2 & RX_DMA_DONE))
		return false;

	rxd->rxd1 = READ_ONCE(dma_rxd->rxd1);
	rxd->rxd3 = READ_ONCE(dma_rxd->rxd3);
	rxd->rxd4 = READ_ONCE(dma_rxd->rxd4);

	if (MTK_HAS_CAPS(eth->soc->caps, MTK_NETSYS_RX_V2)) {
		rxd->rxd5 = READ_ONCE(dma_rxd->rxd5);
		rxd->rxd6 = READ_ONCE(dma_rxd->rxd6);
		rxd->rxd7 = READ_ONCE(dma_rxd->rxd7);
	}

	return true;
}

static void *mtk_max_lro_buf_alloc(gfp_t gfp_mask)
{
	unsigned int size = mtk_max_frag_size(MTK_MAX_LRO_RX_LENGTH);
	unsigned long data;

	data = __get_free_pages(gfp_mask | __GFP_COMP | __GFP_NOWARN,
				get_order(size));

	return (void *)data;
}

/* the qdma core needs scratch memory to be setup */
static int mtk_init_fq_dma(struct mtk_eth *eth)
{
	const struct mtk_soc_data *soc = eth->soc;
	dma_addr_t phy_ring_tail;
	int cnt = soc->txrx.fq_dma_size;
	dma_addr_t dma_addr;
	u64 addr64 = 0;
	int i, j, len;

	if (!eth->soc->has_sram) {
		eth->scratch_ring = dma_alloc_coherent(eth->dma_dev,
					       cnt * soc->txrx.txd_size,
					       &eth->phy_scratch_ring,
					       GFP_KERNEL);
	} else {
		eth->scratch_ring = eth->sram_base;
	}

	if (unlikely(!eth->scratch_ring))
                        return -ENOMEM;

	phy_ring_tail = eth->phy_scratch_ring +
			(dma_addr_t)soc->txrx.txd_size * (cnt - 1);

	for (j = 0; j < DIV_ROUND_UP(soc->txrx.fq_dma_size, MTK_FQ_DMA_LENGTH); j++) {
		len = min_t(int, cnt - j * MTK_FQ_DMA_LENGTH, MTK_FQ_DMA_LENGTH);

		eth->scratch_head[j] = kcalloc(len, MTK_QDMA_PAGE_SIZE, GFP_KERNEL);
		if (unlikely(!eth->scratch_head[j]))
			return -ENOMEM;

		dma_addr = dma_map_single(eth->dma_dev,
					  eth->scratch_head[j], len * MTK_QDMA_PAGE_SIZE,
					  DMA_FROM_DEVICE);
		if (unlikely(dma_mapping_error(eth->dma_dev, dma_addr)))
			return -ENOMEM;

		for (i = 0; i < len; i++) {
			struct mtk_tx_dma_v2 *txd;

			txd = eth->scratch_ring + (j * MTK_FQ_DMA_LENGTH + i) * soc->txrx.txd_size;
			txd->txd1 = dma_addr + i * MTK_QDMA_PAGE_SIZE;
			if (j * MTK_FQ_DMA_LENGTH + i < cnt)
				txd->txd2 = eth->phy_scratch_ring +
					(j * MTK_FQ_DMA_LENGTH + i + 1) * soc->txrx.txd_size;

			addr64 = (MTK_HAS_CAPS(eth->soc->caps, MTK_8GB_ADDRESSING)) ?
				  TX_DMA_SDP1(dma_addr + i * MTK_QDMA_PAGE_SIZE) : 0;

			txd->txd3 = TX_DMA_PLEN0(MTK_QDMA_PAGE_SIZE) | addr64;
			txd->txd4 = 0;

			if (MTK_HAS_CAPS(eth->soc->caps, MTK_NETSYS_V2) ||
			    MTK_HAS_CAPS(eth->soc->caps, MTK_NETSYS_V3)) {
				txd->txd5 = 0;
				txd->txd6 = 0;
				txd->txd7 = 0;
				txd->txd8 = 0;
			}
		}
	}

	mtk_w32(eth, eth->phy_scratch_ring, soc->reg_map->qdma.fq_head);
	mtk_w32(eth, phy_ring_tail, soc->reg_map->qdma.fq_tail);
	mtk_w32(eth, (cnt << 16) | cnt, soc->reg_map->qdma.fq_count);
	mtk_w32(eth, MTK_QDMA_PAGE_SIZE << 16, soc->reg_map->qdma.fq_blen);

	return 0;
}

static inline void *mtk_qdma_phys_to_virt(struct mtk_tx_ring *ring, u32 desc)
{
	return ring->dma + (desc - ring->phys);
}

static inline struct mtk_tx_buf *mtk_desc_to_tx_buf(struct mtk_tx_ring *ring,
						    void *txd, u32 txd_size)
{
	int idx = (txd - ring->dma) / txd_size;

	return &ring->buf[idx];
}

static struct mtk_tx_dma *qdma_to_pdma(struct mtk_tx_ring *ring,
				       void *dma)
{
	return ring->dma_pdma - ring->dma + dma;
}

static int txd_to_idx(struct mtk_tx_ring *ring, void *dma, u32 txd_size)
{
	return (dma - ring->dma) / txd_size;
}

static void mtk_tx_unmap(struct mtk_eth *eth, struct mtk_tx_buf *tx_buf,
			 bool napi)
{
	if (MTK_HAS_CAPS(eth->soc->caps, MTK_QDMA)) {
		if (tx_buf->flags & MTK_TX_FLAGS_SINGLE0) {
			dma_unmap_single(eth->dma_dev,
					 dma_unmap_addr(tx_buf, dma_addr0),
					 dma_unmap_len(tx_buf, dma_len0),
					 DMA_TO_DEVICE);
		} else if (tx_buf->flags & MTK_TX_FLAGS_PAGE0) {
			dma_unmap_page(eth->dma_dev,
				       dma_unmap_addr(tx_buf, dma_addr0),
				       dma_unmap_len(tx_buf, dma_len0),
				       DMA_TO_DEVICE);
		}
	} else {
		if (dma_unmap_len(tx_buf, dma_len0)) {
			dma_unmap_page(eth->dma_dev,
				       dma_unmap_addr(tx_buf, dma_addr0),
				       dma_unmap_len(tx_buf, dma_len0),
				       DMA_TO_DEVICE);
		}

		if (dma_unmap_len(tx_buf, dma_len1)) {
			dma_unmap_page(eth->dma_dev,
				       dma_unmap_addr(tx_buf, dma_addr1),
				       dma_unmap_len(tx_buf, dma_len1),
				       DMA_TO_DEVICE);
		}
	}

	tx_buf->flags = 0;
	if (tx_buf->skb &&
	    (tx_buf->skb != (struct sk_buff *)MTK_DMA_DUMMY_DESC)) {
		if (napi)
			napi_consume_skb(tx_buf->skb, napi);
		else
			dev_kfree_skb_any(tx_buf->skb);
	}
	tx_buf->skb = NULL;
}

static void setup_tx_buf(struct mtk_eth *eth, struct mtk_tx_buf *tx_buf,
			 struct mtk_tx_dma *txd, dma_addr_t mapped_addr,
			 size_t size, int idx)
{
	u64 addr64 = 0;

	if (MTK_HAS_CAPS(eth->soc->caps, MTK_QDMA)) {
		dma_unmap_addr_set(tx_buf, dma_addr0, mapped_addr);
		dma_unmap_len_set(tx_buf, dma_len0, size);
	} else {
		addr64 = (MTK_HAS_CAPS(eth->soc->caps, MTK_8GB_ADDRESSING)) ?
			  TX_DMA_SDP1(mapped_addr) : 0;

		if (idx & 1) {
			txd->txd3 = mapped_addr;
			if (MTK_HAS_CAPS(eth->soc->caps, MTK_NETSYS_V3))
				txd->txd4 = TX_DMA_PLEN1(size) | addr64;
			else
				txd->txd2 |= TX_DMA_PLEN1(size);
			dma_unmap_addr_set(tx_buf, dma_addr1, mapped_addr);
			dma_unmap_len_set(tx_buf, dma_len1, size);
		} else {
			tx_buf->skb = (struct sk_buff *)MTK_DMA_DUMMY_DESC;
			txd->txd1 = mapped_addr;
			if (MTK_HAS_CAPS(eth->soc->caps, MTK_NETSYS_V3))
				txd->txd2 = TX_DMA_PLEN0(size) | addr64;
			else
				txd->txd2 = TX_DMA_PLEN0(size);
			dma_unmap_addr_set(tx_buf, dma_addr0, mapped_addr);
			dma_unmap_len_set(tx_buf, dma_len0, size);
		}
	}
}

static void mtk_tx_set_dma_desc_v1(struct sk_buff *skb, struct net_device *dev, void *txd,
				struct mtk_tx_dma_desc_info *info)
{
	struct mtk_mac *mac = netdev_priv(dev);
	struct mtk_eth *eth = mac->hw;
	struct mtk_tx_dma *desc = txd;
	u32 data;

	WRITE_ONCE(desc->txd1, info->addr);

	data = TX_DMA_SWC | QID_LOW_BITS(info->qid) | TX_DMA_PLEN0(info->size);
	if (info->last)
		data |= TX_DMA_LS0;
	WRITE_ONCE(desc->txd3, data);

	data = (mac->id + 1) << TX_DMA_FPORT_SHIFT; /* forward port */
	data |= QID_HIGH_BITS(info->qid);
	if (info->first) {
		if (info->gso)
			data |= TX_DMA_TSO;
		/* tx checksum offload */
		if (info->csum)
			data |= TX_DMA_CHKSUM;
		/* vlan header offload */
		if (info->vlan)
			data |= TX_DMA_INS_VLAN | info->vlan_tci;
	}

#if defined(CONFIG_NET_MEDIATEK_HNAT) || defined(CONFIG_NET_MEDIATEK_HNAT_MODULE)
	if (HNAT_SKB_CB2(skb)->magic == 0x78681415) {
		data &= ~(0x7 << TX_DMA_FPORT_SHIFT);
		data |= 0x4 << TX_DMA_FPORT_SHIFT;
	}

	trace_printk("[%s] skb_shinfo(skb)->nr_frags=%x HNAT_SKB_CB2(skb)->magic=%x txd4=%x<-----\n",
		     __func__, skb_shinfo(skb)->nr_frags, HNAT_SKB_CB2(skb)->magic, data);
#endif
	WRITE_ONCE(desc->txd4, data);
}

static void mtk_tx_set_dma_desc_v2(struct sk_buff *skb, struct net_device *dev, void *txd,
				struct mtk_tx_dma_desc_info *info)
{
	struct mtk_mac *mac = netdev_priv(dev);
	struct mtk_eth *eth = mac->hw;
	struct mtk_tx_dma_v2 *desc = txd;
	u32 data = 0;

	WRITE_ONCE(desc->txd1, info->addr);

	data = TX_DMA_PLEN0(info->size);
	if (info->last)
		data |= TX_DMA_LS0;
	WRITE_ONCE(desc->txd3, data);

	data = ((mac->id == MTK_GMAC3_ID) ?
		PSE_GDM3_PORT : (mac->id + 1)) << TX_DMA_FPORT_SHIFT_V2; /* forward port */
	data |= TX_DMA_SWC_V2 | QID_BITS_V2(info->qid);
#if defined(CONFIG_NET_MEDIATEK_HNAT) || defined(CONFIG_NET_MEDIATEK_HNAT_MODULE)
	if (HNAT_SKB_CB2(skb)->magic == 0x78681415) {
		data &= ~(0xf << TX_DMA_FPORT_SHIFT_V2);
		data |= 0x3 << TX_DMA_FPORT_SHIFT_V2;
	}

	trace_printk("[%s] skb_shinfo(skb)->nr_frags=%x HNAT_SKB_CB2(skb)->magic=%x txd4=%x<-----\n",
		     __func__, skb_shinfo(skb)->nr_frags, HNAT_SKB_CB2(skb)->magic, data);
#endif
	WRITE_ONCE(desc->txd4, data);

	data = 0;
	if (info->first) {
		if (info->gso)
			data |= TX_DMA_TSO_V2;
		/* tx checksum offload */
		if (info->csum)
			data |= TX_DMA_CHKSUM_V2;
	}
	WRITE_ONCE(desc->txd5, data);

	data = 0;
	if (info->first && info->vlan)
		data |= TX_DMA_INS_VLAN_V2 | info->vlan_tci;
	WRITE_ONCE(desc->txd6, data);

	WRITE_ONCE(desc->txd7, 0);
	WRITE_ONCE(desc->txd8, 0);
}

static void mtk_tx_set_dma_desc_v3(struct sk_buff *skb, struct net_device *dev, void *txd,
				struct mtk_tx_dma_desc_info *info)
{
	struct mtk_mac *mac = netdev_priv(dev);
	struct mtk_eth *eth = mac->hw;
	struct mtk_tx_dma_v2 *desc = txd;
	u64 addr64 = 0;
	u32 data = 0;

	addr64 = (MTK_HAS_CAPS(eth->soc->caps, MTK_8GB_ADDRESSING)) ?
		  TX_DMA_SDP1(info->addr) : 0;

	WRITE_ONCE(desc->txd1, info->addr);

	data = TX_DMA_PLEN0(info->size);
	if (info->last)
		data |= TX_DMA_LS0;
	WRITE_ONCE(desc->txd3, data | addr64);

	data = ((mac->id == MTK_GMAC3_ID) ?
		PSE_GDM3_PORT : (mac->id + 1)) << TX_DMA_FPORT_SHIFT_V2; /* forward port */
	data |= TX_DMA_SWC_V2 | QID_BITS_V2(info->qid);
#if defined(CONFIG_NET_MEDIATEK_HNAT) || defined(CONFIG_NET_MEDIATEK_HNAT_MODULE)
	if (HNAT_SKB_CB2(skb)->magic == 0x78681415) {
		data &= ~(0xf << TX_DMA_FPORT_SHIFT_V2);
		data |= 0x3 << TX_DMA_FPORT_SHIFT_V2;
	}

	trace_printk("[%s] skb_shinfo(skb)->nr_frags=%x HNAT_SKB_CB2(skb)->magic=%x txd4=%x<-----\n",
		     __func__, skb_shinfo(skb)->nr_frags, HNAT_SKB_CB2(skb)->magic, data);
#endif
	WRITE_ONCE(desc->txd4, data);

	data = 0;
	if (info->first) {
		if (info->gso)
			data |= TX_DMA_TSO_V2;
		/* tx checksum offload */
		if (info->csum)
			data |= TX_DMA_CHKSUM_V2;

		if (netdev_uses_dsa(dev))
			data |= TX_DMA_SPTAG_V3;
	}
	WRITE_ONCE(desc->txd5, data);

	data = 0;
	if (info->first && info->vlan)
		data |= TX_DMA_INS_VLAN_V2 | info->vlan_tci;
	WRITE_ONCE(desc->txd6, data);

	WRITE_ONCE(desc->txd7, 0);
	WRITE_ONCE(desc->txd8, 0);
}

static void mtk_tx_set_pdma_desc(struct sk_buff *skb, struct net_device *dev, void *txd,
				struct mtk_tx_dma_desc_info *info)
{
	struct mtk_mac *mac = netdev_priv(dev);
	struct mtk_tx_dma_v2 *desc = txd;
	u32 data = 0;

	if (info->first) {
		data = ((mac->id == MTK_GMAC3_ID) ?
			PSE_GDM3_PORT : (mac->id + 1)) << TX_DMA_FPORT_SHIFT_PDMA;
		if (info->gso)
			data |= TX_DMA_TSO_V2;
		if (info->csum)
			data |= TX_DMA_CHKSUM_V2;
		if (netdev_uses_dsa(dev))
			data |= TX_DMA_SPTAG_V3;
		WRITE_ONCE(desc->txd5, data);

		if (info->vlan) {
			WRITE_ONCE(desc->txd6, TX_DMA_INS_VLAN_V2);
			WRITE_ONCE(desc->txd7, info->vlan_tci);
		}

		WRITE_ONCE(desc->txd8, 0);
	}
}

static void mtk_tx_set_dma_desc(struct sk_buff *skb, struct net_device *dev, void *txd,
				struct mtk_tx_dma_desc_info *info)
{
	struct mtk_mac *mac = netdev_priv(dev);
	struct mtk_eth *eth = mac->hw;

	if (MTK_HAS_CAPS(eth->soc->caps, MTK_QDMA)) {
		if (MTK_HAS_CAPS(eth->soc->caps, MTK_NETSYS_V3))
			mtk_tx_set_dma_desc_v3(skb, dev, txd, info);
		else if (MTK_HAS_CAPS(eth->soc->caps, MTK_NETSYS_V2))
			mtk_tx_set_dma_desc_v2(skb, dev, txd, info);
		else
			mtk_tx_set_dma_desc_v1(skb, dev, txd, info);
	} else {
		if (MTK_HAS_CAPS(eth->soc->caps, MTK_NETSYS_V3))
			mtk_tx_set_pdma_desc(skb, dev, txd, info);
	}
}

static int mtk_tx_map(struct sk_buff *skb, struct net_device *dev,
		      int tx_num, struct mtk_tx_ring *ring, bool gso)
{
	struct mtk_tx_dma_desc_info txd_info = {
		.size = skb_headlen(skb),
		.qid = skb_get_queue_mapping(skb),
		.gso = gso,
		.csum = skb->ip_summed == CHECKSUM_PARTIAL,
		.vlan = skb_vlan_tag_present(skb),
		.vlan_tci = skb_vlan_tag_get(skb),
		.first = true,
		.last = !skb_is_nonlinear(skb),
	};
	struct netdev_queue *txq;
	struct mtk_mac *mac = netdev_priv(dev);
	struct mtk_eth *eth = mac->hw;
	const struct mtk_soc_data *soc = eth->soc;
	struct mtk_tx_dma *itxd, *txd;
	struct mtk_tx_dma *itxd_pdma, *txd_pdma;
	struct mtk_tx_buf *itx_buf, *tx_buf;
	int i, n_desc = 1;
	int queue = skb_get_queue_mapping(skb);
	int k = 0;

	if (skb->len <= 40) {
		if (skb_put_padto(skb, MTK_MIN_TX_LENGTH))
			return -ENOMEM;
		txd_info.last = !skb_is_nonlinear(skb);
		txd_info.size = skb_headlen(skb);
	}

	txq = netdev_get_tx_queue(dev, txd_info.qid);
	itxd = ring->next_free;
	itxd_pdma = qdma_to_pdma(ring, itxd);
	if (itxd == ring->last_free)
		return -ENOMEM;

	itx_buf = mtk_desc_to_tx_buf(ring, itxd, soc->txrx.txd_size);
	memset(itx_buf, 0, sizeof(*itx_buf));

	txd_info.addr = dma_map_single(eth->dma_dev, skb->data, txd_info.size,
				       DMA_TO_DEVICE);
	if (unlikely(dma_mapping_error(eth->dma_dev, txd_info.addr)))
		return -ENOMEM;

	if (MTK_HAS_CAPS(soc->caps, MTK_QDMA))
		mtk_tx_set_dma_desc(skb, dev, itxd, &txd_info);
	else
		mtk_tx_set_dma_desc(skb, dev, itxd_pdma, &txd_info);

	itx_buf->flags |= MTK_TX_FLAGS_SINGLE0;
	itx_buf->flags |= (mac->id == MTK_GMAC1_ID) ? MTK_TX_FLAGS_FPORT0 :
			  (mac->id == MTK_GMAC2_ID) ? MTK_TX_FLAGS_FPORT1 :
						      MTK_TX_FLAGS_FPORT2;
	setup_tx_buf(eth, itx_buf, itxd_pdma, txd_info.addr, txd_info.size,
		     k++);

	/* TX SG offload */
	txd = itxd;
	txd_pdma = qdma_to_pdma(ring, txd);

	for (i = 0; i < skb_shinfo(skb)->nr_frags; i++) {
		skb_frag_t *frag = &skb_shinfo(skb)->frags[i];
		unsigned int offset = 0;
		int frag_size = skb_frag_size(frag);

		while (frag_size) {
			bool new_desc = true;

			if (MTK_HAS_CAPS(soc->caps, MTK_QDMA) ||
			    (i & 0x1)) {
				txd = mtk_qdma_phys_to_virt(ring, txd->txd2);
				txd_pdma = qdma_to_pdma(ring, txd);
				if (txd == ring->last_free)
					goto err_dma;

				n_desc++;
			} else {
				new_desc = false;
			}

			memset(&txd_info, 0, sizeof(struct mtk_tx_dma_desc_info));
			txd_info.size = min(frag_size, MTK_TX_DMA_BUF_LEN);
			txd_info.qid = queue;
			txd_info.last = i == skb_shinfo(skb)->nr_frags - 1 &&
					!(frag_size - txd_info.size);
			txd_info.addr = skb_frag_dma_map(eth->dma_dev, frag,
							 offset, txd_info.size,
							 DMA_TO_DEVICE);
			if (unlikely(dma_mapping_error(eth->dma_dev,
						       txd_info.addr)))
 				goto err_dma;

			if (MTK_HAS_CAPS(soc->caps, MTK_QDMA))
				mtk_tx_set_dma_desc(skb, dev, txd, &txd_info);
			else
				mtk_tx_set_dma_desc(skb, dev, txd_pdma, &txd_info);

			tx_buf = mtk_desc_to_tx_buf(ring, txd, soc->txrx.txd_size);
			if (new_desc)
				memset(tx_buf, 0, sizeof(*tx_buf));
			tx_buf->skb = (struct sk_buff *)MTK_DMA_DUMMY_DESC;
			tx_buf->flags |= MTK_TX_FLAGS_PAGE0;
			tx_buf->flags |=
				(mac->id == MTK_GMAC1_ID) ? MTK_TX_FLAGS_FPORT0 :
				(mac->id == MTK_GMAC2_ID) ? MTK_TX_FLAGS_FPORT1 :
							    MTK_TX_FLAGS_FPORT2;

			setup_tx_buf(eth, tx_buf, txd_pdma, txd_info.addr,
				     txd_info.size, k++);

			frag_size -= txd_info.size;
			offset += txd_info.size;
		}
	}

	/* store skb to cleanup */
	itx_buf->skb = skb;

	if (!MTK_HAS_CAPS(soc->caps, MTK_QDMA)) {
		if (MTK_HAS_CAPS(eth->soc->caps, MTK_NETSYS_V3)) {
			if (k & 0x1)
				txd_pdma->txd2 |= TX_DMA_LS0;
			else
				txd_pdma->txd4 |= TX_DMA_LS1_V2;
		} else {
			if (k & 0x1)
				txd_pdma->txd2 |= TX_DMA_LS0;
			else
				txd_pdma->txd2 |= TX_DMA_LS1;
		}
	}

	netdev_tx_sent_queue(txq, skb->len);
	skb_tx_timestamp(skb);

	ring->next_free = mtk_qdma_phys_to_virt(ring, txd->txd2);
	atomic_sub(n_desc, &ring->free_count);

	/* make sure that all changes to the dma ring are flushed before we
	 * continue
	 */
	wmb();

	if (MTK_HAS_CAPS(soc->caps, MTK_QDMA)) {
		if (netif_xmit_stopped(txq) || !netdev_xmit_more())
			mtk_w32(eth, txd->txd2, soc->reg_map->qdma.ctx_ptr);
	} else {
		int next_idx = NEXT_DESP_IDX(txd_to_idx(ring, txd, soc->txrx.txd_size),
					     ring->dma_size);
		mtk_w32(eth, next_idx, soc->reg_map->pdma.pctx_ptr);
	}

	return 0;

err_dma:
	do {
		tx_buf = mtk_desc_to_tx_buf(ring, itxd, soc->txrx.txd_size);

		/* unmap dma */
		mtk_tx_unmap(eth, tx_buf, false);

		itxd->txd3 = TX_DMA_LS0 | TX_DMA_OWNER_CPU;
		if (!MTK_HAS_CAPS(soc->caps, MTK_QDMA))
			itxd_pdma->txd2 = TX_DMA_DESP2_DEF;

		itxd = mtk_qdma_phys_to_virt(ring, itxd->txd2);
		itxd_pdma = qdma_to_pdma(ring, itxd);
	} while (itxd != txd);

	return -ENOMEM;
}

static inline int mtk_cal_txd_req(struct sk_buff *skb)
{
	int i, nfrags;
	skb_frag_t *frag;

	nfrags = 1;
	if (skb_is_gso(skb)) {
		for (i = 0; i < skb_shinfo(skb)->nr_frags; i++) {
			frag = &skb_shinfo(skb)->frags[i];
			nfrags += DIV_ROUND_UP(skb_frag_size(frag),
						MTK_TX_DMA_BUF_LEN);
		}
	} else {
		nfrags += skb_shinfo(skb)->nr_frags;
	}

	return nfrags;
}

static int mtk_queue_stopped(struct mtk_eth *eth)
{
	int i;

	for (i = 0; i < MTK_MAC_COUNT; i++) {
		if (!eth->netdev[i])
			continue;
		if (netif_queue_stopped(eth->netdev[i]))
			return 1;
	}

	return 0;
}

static void mtk_wake_queue(struct mtk_eth *eth)
{
	int i;

	for (i = 0; i < MTK_MAC_COUNT; i++) {
		if (!eth->netdev[i])
			continue;
		netif_tx_wake_all_queues(eth->netdev[i]);
	}
}

static int mtk_start_xmit(struct sk_buff *skb, struct net_device *dev)
{
	struct mtk_mac *mac = netdev_priv(dev);
	struct mtk_eth *eth = mac->hw;
	struct mtk_tx_ring *ring = &eth->tx_ring;
	struct net_device_stats *stats = &dev->stats;
	bool gso = false;
	int tx_num;

	/* normally we can rely on the stack not calling this more than once,
	 * however we have 2 queues running on the same ring so we need to lock
	 * the ring access
	 */
	spin_lock(&eth->page_lock);

	if (unlikely(test_bit(MTK_RESETTING, &eth->state)))
		goto drop;

	tx_num = mtk_cal_txd_req(skb);
	if (unlikely(atomic_read(&ring->free_count) <= tx_num)) {
		netif_tx_stop_all_queues(dev);
		netif_err(eth, tx_queued, dev,
			  "Tx Ring full when queue awake!\n");
		spin_unlock(&eth->page_lock);
		return NETDEV_TX_BUSY;
	}

	/* TSO: fill MSS info in tcp checksum field */
	if (skb_is_gso(skb)) {
		if (skb_cow_head(skb, 0)) {
			netif_warn(eth, tx_err, dev,
				   "GSO expand head fail.\n");
			goto drop;
		}

		if (skb_shinfo(skb)->gso_type &
				(SKB_GSO_TCPV4 | SKB_GSO_TCPV6)) {
			gso = true;
			tcp_hdr(skb)->check = htons(skb_shinfo(skb)->gso_size);
		}
	}

	if (mtk_tx_map(skb, dev, tx_num, ring, gso) < 0)
		goto drop;

	if (unlikely(atomic_read(&ring->free_count) <= ring->thresh))
		netif_tx_stop_all_queues(dev);

	spin_unlock(&eth->page_lock);

	return NETDEV_TX_OK;

drop:
	spin_unlock(&eth->page_lock);
	stats->tx_dropped++;
	dev_kfree_skb_any(skb);
	return NETDEV_TX_OK;
}

static void mtk_update_rx_cpu_idx(struct mtk_eth *eth, struct mtk_rx_ring *ring)
{
	mtk_w32(eth, ring->calc_idx, ring->crx_idx_reg);
}

static int mtk_poll_rx(struct napi_struct *napi, int budget,
		       struct mtk_eth *eth)
{
	struct mtk_napi *rx_napi = container_of(napi, struct mtk_napi, napi);
	struct mtk_rx_ring *ring = rx_napi->rx_ring;
	int idx;
	struct sk_buff *skb;
	u8 *data, *new_data;
	struct mtk_rx_dma_v2 *rxd, trxd;
	int done = 0;

	if (unlikely(!ring))
		goto rx_done;

	while (done < budget) {
		unsigned int pktlen, *rxdcsum;
		struct net_device *netdev = NULL;
		dma_addr_t dma_addr = DMA_MAPPING_ERROR;
		u64 addr64 = 0;
		int mac = 0;

		idx = NEXT_DESP_IDX(ring->calc_idx, ring->dma_size);
		rxd = ring->dma + idx * eth->soc->txrx.rxd_size;
		data = ring->data[idx];

		if (!mtk_rx_get_desc(eth, &trxd, rxd))
			break;

		/* find out which mac the packet come from. values start at 1 */
		if (MTK_HAS_CAPS(eth->soc->caps, MTK_SOC_MT7628)) {
			mac = 0;
		} else {
			if (MTK_HAS_CAPS(eth->soc->caps, MTK_NETSYS_RX_V2)) {
				switch (RX_DMA_GET_SPORT_V2(trxd.rxd5)) {
				case PSE_GDM3_PORT:
					mac = MTK_GMAC3_ID;
					break;
				default:
					mac = RX_DMA_GET_SPORT_V2(trxd.rxd5) - 1;
					break;
				}
			} else
				mac = (trxd.rxd4 & RX_DMA_SPECIAL_TAG) ?
				      0 : RX_DMA_GET_SPORT(trxd.rxd4) - 1;
		}
		if (mac == 4) mac = 0;
		if (unlikely(mac < 0 || mac >= MTK_MAC_COUNT ||
			     !eth->netdev[mac]))
			goto release_desc;

		netdev = eth->netdev[mac];

		if (unlikely(test_bit(MTK_RESETTING, &eth->state)))
			goto release_desc;

		/* alloc new buffer */
		if (ring->frag_size <= PAGE_SIZE)
			new_data = napi_alloc_frag(ring->frag_size);
		else
			new_data = mtk_max_lro_buf_alloc(GFP_ATOMIC);
		if (unlikely(!new_data)) {
			netdev->stats.rx_dropped++;
			goto release_desc;
		}
		dma_addr = dma_map_single(eth->dma_dev,
					  new_data + NET_SKB_PAD +
					  eth->ip_align,
					  ring->buf_size,
					  DMA_FROM_DEVICE);
		if (unlikely(dma_mapping_error(eth->dma_dev, dma_addr))) {
			skb_free_frag(new_data);
			netdev->stats.rx_dropped++;
			goto release_desc;
		}

		addr64 = (MTK_HAS_CAPS(eth->soc->caps, MTK_8GB_ADDRESSING)) ?
			  ((u64)(trxd.rxd2 & 0xf)) << 32 : 0;

		dma_unmap_single(eth->dma_dev,
				 ((u64)(trxd.rxd1) | addr64),
				 ring->buf_size, DMA_FROM_DEVICE);

		/* receive data */
		skb = build_skb(data, ring->frag_size);
		if (unlikely(!skb)) {
			skb_free_frag(data);
			netdev->stats.rx_dropped++;
			goto skip_rx;
		}
		skb_reserve(skb, NET_SKB_PAD + NET_IP_ALIGN);

		pktlen = RX_DMA_GET_PLEN0(trxd.rxd2);
		skb->dev = netdev;
		skb_put(skb, pktlen);

		if ((MTK_HAS_CAPS(eth->soc->caps, MTK_NETSYS_RX_V2)))
			rxdcsum = &trxd.rxd3;
		else
			rxdcsum = &trxd.rxd4;

		if (*rxdcsum & eth->soc->txrx.rx_dma_l4_valid)
			skb->ip_summed = CHECKSUM_UNNECESSARY;
		else
			skb_checksum_none_assert(skb);
		skb->protocol = eth_type_trans(skb, netdev);

		if (netdev->features & NETIF_F_HW_VLAN_CTAG_RX) {
			if (MTK_HAS_CAPS(eth->soc->caps, MTK_NETSYS_RX_V2)) {
				if (trxd.rxd3 & RX_DMA_VTAG_V2)
					__vlan_hwaccel_put_tag(skb,
					htons(RX_DMA_VPID_V2(trxd.rxd4)),
					RX_DMA_VID_V2(trxd.rxd4));
			} else {
				if (trxd.rxd2 & RX_DMA_VTAG)
					__vlan_hwaccel_put_tag(skb,
					htons(RX_DMA_VPID(trxd.rxd3)),
					RX_DMA_VID(trxd.rxd3));
			}

			/* If netdev is attached to dsa switch, the special
			 * tag inserted in VLAN field by switch hardware can
			 * be offload by RX HW VLAN offload. Clears the VLAN
			 * information from @skb to avoid unexpected 8021d
			 * handler before packet enter dsa framework.
			 */
			if (netdev_uses_dsa(netdev))
				__vlan_hwaccel_clear_tag(skb);
		}

#if defined(CONFIG_NET_MEDIATEK_HNAT) || defined(CONFIG_NET_MEDIATEK_HNAT_MODULE)
		if (MTK_HAS_CAPS(eth->soc->caps, MTK_NETSYS_RX_V2))
			*(u32 *)(skb->head) = trxd.rxd5;
		else
			*(u32 *)(skb->head) = trxd.rxd4;

		skb_hnat_alg(skb) = 0;
		skb_hnat_filled(skb) = 0;
		skb_hnat_magic_tag(skb) = HNAT_MAGIC_TAG;

		if (skb_hnat_reason(skb) == HIT_BIND_FORCE_TO_CPU) {
			trace_printk("[%s] reason=0x%x(force to CPU) from WAN to Ext\n",
				     __func__, skb_hnat_reason(skb));
			skb->pkt_type = PACKET_HOST;
		}

		trace_printk("[%s] rxd:(entry=%x,sport=%x,reason=%x,alg=%x\n",
			     __func__, skb_hnat_entry(skb), skb_hnat_sport(skb),
			     skb_hnat_reason(skb), skb_hnat_alg(skb));
#endif
		if (mtk_hwlro_stats_ebl &&
		    IS_HW_LRO_RING(ring->ring_no) && eth->hwlro) {
			hw_lro_stats_update(ring->ring_no, &trxd);
			hw_lro_flush_stats_update(ring->ring_no, &trxd);
		}

		skb_record_rx_queue(skb, 0);
		napi_gro_receive(napi, skb);

skip_rx:
		ring->data[idx] = new_data;
		rxd->rxd1 = (unsigned int)dma_addr;

release_desc:
		if (MTK_HAS_CAPS(eth->soc->caps, MTK_8GB_ADDRESSING)) {
			if (unlikely(dma_addr == DMA_MAPPING_ERROR))
				addr64 = RX_DMA_GET_SDP1(rxd->rxd2);
			else
				addr64 = RX_DMA_SDP1(dma_addr);
		}

		if (MTK_HAS_CAPS(eth->soc->caps, MTK_SOC_MT7628))
			rxd->rxd2 = RX_DMA_LSO;
		else
			rxd->rxd2 = RX_DMA_PLEN0(ring->buf_size) | addr64;

		ring->calc_idx = idx;

		done++;
	}

rx_done:
	if (done) {
		/* make sure that all changes to the dma ring are flushed before
		 * we continue
		 */
		wmb();
		mtk_update_rx_cpu_idx(eth, ring);
	}

	return done;
}

struct mtk_poll_state {
	struct netdev_queue *txq;
	unsigned int total;
	unsigned int done;
	unsigned int bytes;
};

static void
mtk_poll_tx_done(struct mtk_eth *eth, struct mtk_poll_state *state, u8 mac,
		 struct sk_buff *skb)
{
	struct netdev_queue *txq;
	struct net_device *dev;
	unsigned int bytes = skb->len;

	state->total++;

	dev = eth->netdev[mac];
	if (!dev)
		return;

	txq = netdev_get_tx_queue(dev, skb_get_queue_mapping(skb));
	if (state->txq == txq) {
		state->done++;
		state->bytes += bytes;
		return;
	}

	if (state->txq)
		netdev_tx_completed_queue(state->txq, state->done, state->bytes);

	state->txq = txq;
	state->done = 1;
	state->bytes = bytes;
}

static void mtk_poll_tx_qdma(struct mtk_eth *eth, int budget,
			     struct mtk_poll_state *state)
{
	const struct mtk_reg_map *reg_map = eth->soc->reg_map;
	const struct mtk_soc_data *soc = eth->soc;
	struct mtk_tx_ring *ring = &eth->tx_ring;
	struct mtk_tx_dma *desc;
	struct sk_buff *skb;
	struct mtk_tx_buf *tx_buf;
	u32 cpu, dma;

	cpu = ring->last_free_ptr;
	dma = mtk_r32(eth, reg_map->qdma.drx_ptr);

	desc = mtk_qdma_phys_to_virt(ring, cpu);

	while ((cpu != dma) && budget) {
		u32 next_cpu = desc->txd2;
		int mac = 0;

		if ((desc->txd3 & TX_DMA_OWNER_CPU) == 0)
			break;

		desc = mtk_qdma_phys_to_virt(ring, desc->txd2);

		tx_buf = mtk_desc_to_tx_buf(ring, desc, soc->txrx.txd_size);
		if (tx_buf->flags & MTK_TX_FLAGS_FPORT1)
			mac = MTK_GMAC2_ID;
		else if (tx_buf->flags & MTK_TX_FLAGS_FPORT2)
			mac = MTK_GMAC3_ID;

		skb = tx_buf->skb;
		if (!skb)
			break;

		if (skb != (struct sk_buff *)MTK_DMA_DUMMY_DESC) {
			mtk_poll_tx_done(eth, state, mac, skb);
			budget--;
		}
		mtk_tx_unmap(eth, tx_buf, true);

		ring->last_free = desc;
		atomic_inc(&ring->free_count);

		cpu = next_cpu;
	}

	ring->last_free_ptr = cpu;
	mtk_w32(eth, cpu, reg_map->qdma.crx_ptr);
}

static void mtk_poll_tx_pdma(struct mtk_eth *eth, int budget,
			     struct mtk_poll_state *state)
{
	const struct mtk_soc_data *soc = eth->soc;
	struct mtk_tx_ring *ring = &eth->tx_ring;
	struct mtk_tx_dma *desc;
	struct sk_buff *skb;
	struct mtk_tx_buf *tx_buf;
	u32 cpu, dma;

	cpu = ring->cpu_idx;
	dma = mtk_r32(eth, soc->reg_map->pdma.pdtx_ptr);

	while ((cpu != dma) && budget) {
		int mac = 0;

		desc = ring->dma_pdma + cpu * eth->soc->txrx.txd_size;
		if ((desc->txd2 & TX_DMA_OWNER_CPU) == 0)
			break;

		tx_buf = &ring->buf[cpu];
		if (tx_buf->flags & MTK_TX_FLAGS_FPORT1)
			mac = MTK_GMAC2_ID;
		else if (tx_buf->flags & MTK_TX_FLAGS_FPORT2)
			mac = MTK_GMAC3_ID;

		skb = tx_buf->skb;
		if (!skb)
			break;

		if (skb != (struct sk_buff *)MTK_DMA_DUMMY_DESC) {
			mtk_poll_tx_done(eth, state, mac, skb);
			budget--;
		}

		mtk_tx_unmap(eth, tx_buf, true);

		desc = ring->dma + cpu * eth->soc->txrx.txd_size;
		ring->last_free = desc;
		atomic_inc(&ring->free_count);

		cpu = NEXT_DESP_IDX(cpu, ring->dma_size);
	}

	ring->cpu_idx = cpu;
}

static int mtk_poll_tx(struct mtk_eth *eth, int budget)
{
	struct mtk_tx_ring *ring = &eth->tx_ring;
	struct mtk_poll_state state = {};

	if (MTK_HAS_CAPS(eth->soc->caps, MTK_QDMA))
		mtk_poll_tx_qdma(eth, budget, &state);
	else
		mtk_poll_tx_pdma(eth, budget, &state);

	if (state.txq)
		netdev_tx_completed_queue(state.txq, state.done, state.bytes);

	if (mtk_queue_stopped(eth) &&
	    (atomic_read(&ring->free_count) > ring->thresh))
		mtk_wake_queue(eth);

	return state.total;
}

static void mtk_handle_status_irq(struct mtk_eth *eth)
{
	u32 status2 = mtk_r32(eth, MTK_FE_INT_STATUS);

	if (unlikely(status2 & (MTK_GDM1_AF | MTK_GDM2_AF))) {
		mtk_stats_update(eth);
		mtk_w32(eth, (MTK_GDM1_AF | MTK_GDM2_AF),
			MTK_FE_INT_STATUS);
	}
}

static int mtk_napi_tx(struct napi_struct *napi, int budget)
{
	struct mtk_eth *eth = container_of(napi, struct mtk_eth, tx_napi);
	const struct mtk_reg_map *reg_map = eth->soc->reg_map;
	u32 status, mask;
	int tx_done = 0;

	if (MTK_HAS_CAPS(eth->soc->caps, MTK_QDMA)) {
		mtk_handle_status_irq(eth);
		mtk_w32(eth, MTK_TX_DONE_INT(0), reg_map->tx_irq_status);
	} else {
		mtk_w32(eth, MTK_TX_DONE_INT(0), reg_map->pdma.irq_status);
	}
	tx_done = mtk_poll_tx(eth, budget);

	if (unlikely(netif_msg_intr(eth))) {
		if (MTK_HAS_CAPS(eth->soc->caps, MTK_QDMA)) {
			status = mtk_r32(eth, reg_map->tx_irq_status);
			mask = mtk_r32(eth, reg_map->tx_irq_mask);
		} else {
			status = mtk_r32(eth, reg_map->pdma.irq_status);
			mask = mtk_r32(eth, reg_map->pdma.irq_mask);
		}
		dev_info(eth->dev,
			 "done tx %d, intr 0x%08x/0x%x\n",
			 tx_done, status, mask);
	}

	if (tx_done == budget)
		return budget;

	if (MTK_HAS_CAPS(eth->soc->caps, MTK_QDMA))
		status = mtk_r32(eth, reg_map->tx_irq_status);
	else
		status = mtk_r32(eth, reg_map->pdma.irq_status);
	if (status & MTK_TX_DONE_INT(0))
		return budget;

	if (napi_complete(napi))
		mtk_tx_irq_enable(eth, MTK_TX_DONE_INT(0));

	return tx_done;
}

static int mtk_napi_rx(struct napi_struct *napi, int budget)
{
	struct mtk_napi *rx_napi = container_of(napi, struct mtk_napi, napi);
	struct mtk_eth *eth = rx_napi->eth;
	const struct mtk_reg_map *reg_map = eth->soc->reg_map;
	struct mtk_rx_ring *ring = rx_napi->rx_ring;
	u32 status, mask;
	int rx_done = 0;
	int remain_budget = budget;

	mtk_handle_status_irq(eth);

poll_again:
	mtk_w32(eth, MTK_RX_DONE_INT(ring->ring_no), reg_map->pdma.irq_status);
	rx_done = mtk_poll_rx(napi, remain_budget, eth);

	if (unlikely(netif_msg_intr(eth))) {
		status = mtk_r32(eth, reg_map->pdma.irq_status);
		mask = mtk_r32(eth, reg_map->pdma.irq_mask);
		dev_info(eth->dev,
			 "done rx %d, intr 0x%08x/0x%x\n",
			 rx_done, status, mask);
	}
	if (rx_done == remain_budget)
		return budget;

	status = mtk_r32(eth, reg_map->pdma.irq_status);
	if (status & MTK_RX_DONE_INT(ring->ring_no)) {
		remain_budget -= rx_done;
		goto poll_again;
	}

	if (napi_complete(napi))
		mtk_rx_irq_enable(eth, MTK_RX_DONE_INT(ring->ring_no));

	return rx_done + budget - remain_budget;
}

static int mtk_tx_alloc(struct mtk_eth *eth)
{
	const struct mtk_soc_data *soc = eth->soc;
	struct mtk_tx_ring *ring = &eth->tx_ring;
	int i, sz = soc->txrx.txd_size;
	struct mtk_tx_dma_v2 *txd, *pdma_txd;

	ring->buf = kcalloc(soc->txrx.tx_dma_size, sizeof(*ring->buf),
			       GFP_KERNEL);
	if (!ring->buf)
		goto no_tx_mem;

	if (!eth->soc->has_sram)
		ring->dma = dma_alloc_coherent(eth->dma_dev, soc->txrx.tx_dma_size * sz,
					       &ring->phys, GFP_KERNEL);
	else {
		ring->dma =  eth->sram_base + soc->txrx.fq_dma_size * sz;
		ring->phys = eth->phy_scratch_ring +
			     soc->txrx.fq_dma_size * (dma_addr_t)sz;
	}

	if (!ring->dma)
		goto no_tx_mem;

	for (i = 0; i < soc->txrx.tx_dma_size; i++) {
		int next = (i + 1) % soc->txrx.tx_dma_size;
		u32 next_ptr = ring->phys + next * sz;

		txd = ring->dma + i * sz;
		txd->txd2 = next_ptr;
		txd->txd3 = TX_DMA_LS0 | TX_DMA_OWNER_CPU;
		txd->txd4 = 0;

		if (MTK_HAS_CAPS(eth->soc->caps, MTK_NETSYS_V2) ||
		    MTK_HAS_CAPS(eth->soc->caps, MTK_NETSYS_V3)) {
			txd->txd5 = 0;
			txd->txd6 = 0;
			txd->txd7 = 0;
			txd->txd8 = 0;
		}
	}

	/* On MT7688 (PDMA only) this driver uses the ring->dma structs
	 * only as the framework. The real HW descriptors are the PDMA
	 * descriptors in ring->dma_pdma.
	 */
	if (!MTK_HAS_CAPS(eth->soc->caps, MTK_QDMA)) {
		ring->dma_pdma = dma_alloc_coherent(eth->dma_dev,
						    soc->txrx.tx_dma_size * sz,
						    &ring->phys_pdma, GFP_KERNEL);
		if (!ring->dma_pdma)
			goto no_tx_mem;

		for (i = 0; i < soc->txrx.tx_dma_size; i++) {
			pdma_txd = ring->dma_pdma + i * sz;

			pdma_txd->txd2 = TX_DMA_DESP2_DEF;
			pdma_txd->txd4 = 0;

			if (MTK_HAS_CAPS(eth->soc->caps, MTK_NETSYS_V3)) {
				pdma_txd->txd5 = 0;
				pdma_txd->txd6 = 0;
				pdma_txd->txd7 = 0;
				pdma_txd->txd8 = 0;
			}
		}
	}

	ring->dma_size = soc->txrx.tx_dma_size;
	atomic_set(&ring->free_count, soc->txrx.tx_dma_size - 2);
	ring->next_free = ring->dma;
	ring->last_free = (void *)txd;
	ring->last_free_ptr = (u32)(ring->phys + ((soc->txrx.tx_dma_size - 1) * sz));
	ring->thresh = MAX_SKB_FRAGS;
	ring->cpu_idx = 0;

	/* make sure that all changes to the dma ring are flushed before we
	 * continue
	 */
	wmb();

	if (MTK_HAS_CAPS(eth->soc->caps, MTK_QDMA)) {
		mtk_w32(eth, ring->phys, soc->reg_map->qdma.ctx_ptr);
		mtk_w32(eth, ring->phys, soc->reg_map->qdma.dtx_ptr);
		mtk_w32(eth,
			ring->phys + ((soc->txrx.tx_dma_size - 1) * sz),
			soc->reg_map->qdma.crx_ptr);
		mtk_w32(eth, ring->last_free_ptr, soc->reg_map->qdma.drx_ptr);
		mtk_w32(eth, (QDMA_RES_THRES << 8) | QDMA_RES_THRES,
			soc->reg_map->qdma.qtx_cfg);
	} else {
		mtk_w32(eth, ring->phys_pdma, soc->reg_map->pdma.tx_ptr);
		mtk_w32(eth, soc->txrx.tx_dma_size, soc->reg_map->pdma.tx_cnt_cfg);
		mtk_w32(eth, ring->cpu_idx, soc->reg_map->pdma.pctx_ptr);
		mtk_w32(eth, MTK_PST_DTX_IDX_CFG(0), soc->reg_map->pdma.rst_idx);
	}

	return 0;

no_tx_mem:
	return -ENOMEM;
}

static void mtk_tx_clean(struct mtk_eth *eth)
{
	const struct mtk_soc_data *soc = eth->soc;
	struct mtk_tx_ring *ring = &eth->tx_ring;
	int i;

	if (ring->buf) {
		for (i = 0; i < soc->txrx.tx_dma_size; i++)
			mtk_tx_unmap(eth, &ring->buf[i], false);
		kfree(ring->buf);
		ring->buf = NULL;
	}

	if (!eth->soc->has_sram && ring->dma) {
		dma_free_coherent(eth->dma_dev,
				  soc->txrx.tx_dma_size * soc->txrx.txd_size,
				  ring->dma, ring->phys);
		ring->dma = NULL;
	}

	if (ring->dma_pdma) {
		dma_free_coherent(eth->dma_dev,
				  soc->txrx.tx_dma_size * soc->txrx.txd_size,
				  ring->dma_pdma, ring->phys_pdma);
		ring->dma_pdma = NULL;
	}
}

static int mtk_rx_alloc(struct mtk_eth *eth, int ring_no, int rx_flag)
{
	const struct mtk_reg_map *reg_map = eth->soc->reg_map;
	const struct mtk_soc_data *soc = eth->soc;
	struct mtk_rx_ring *ring;
	int rx_data_len, rx_dma_size;
	int i;
	u64 addr64 = 0;

	if (rx_flag == MTK_RX_FLAGS_QDMA) {
		if (ring_no)
			return -EINVAL;
		ring = &eth->rx_ring_qdma;
	} else {
		ring = &eth->rx_ring[ring_no];
	}

	if (rx_flag == MTK_RX_FLAGS_HWLRO) {
		rx_data_len = MTK_MAX_LRO_RX_LENGTH;
		rx_dma_size = MTK_HW_LRO_DMA_SIZE;
	} else {
		rx_data_len = ETH_DATA_LEN;
		rx_dma_size = soc->txrx.rx_dma_size;
	}

	ring->frag_size = mtk_max_frag_size(rx_data_len);
	ring->buf_size = mtk_max_buf_size(ring->frag_size);
	ring->data = kcalloc(rx_dma_size, sizeof(*ring->data),
			     GFP_KERNEL);
	if (!ring->data)
		return -ENOMEM;

	for (i = 0; i < rx_dma_size; i++) {
		if (ring->frag_size <= PAGE_SIZE)
			ring->data[i] = napi_alloc_frag(ring->frag_size);
		else
			ring->data[i] = mtk_max_lro_buf_alloc(GFP_ATOMIC);
		if (!ring->data[i])
			return -ENOMEM;
	}

	if ((!eth->soc->has_sram) || (eth->soc->has_sram
				&& (rx_flag != MTK_RX_FLAGS_NORMAL)))
		ring->dma = dma_alloc_coherent(eth->dma_dev,
					       rx_dma_size * eth->soc->txrx.rxd_size,
					       &ring->phys, GFP_KERNEL);
	else {
		struct mtk_tx_ring *tx_ring = &eth->tx_ring;
		ring->dma = tx_ring->dma +
			    soc->txrx.tx_dma_size * (dma_addr_t)eth->soc->txrx.txd_size +
			    soc->txrx.rx_dma_size * (dma_addr_t)eth->soc->txrx.rxd_size * ring_no;
		ring->phys = tx_ring->phys +
			     soc->txrx.tx_dma_size * (dma_addr_t)eth->soc->txrx.txd_size +
			     soc->txrx.rx_dma_size * (dma_addr_t)eth->soc->txrx.rxd_size * ring_no;
	}

	if (!ring->dma)
		return -ENOMEM;

	for (i = 0; i < rx_dma_size; i++) {
		struct mtk_rx_dma_v2 *rxd;

		dma_addr_t dma_addr = dma_map_single(eth->dma_dev,
				ring->data[i] + NET_SKB_PAD + eth->ip_align,
				ring->buf_size,
				DMA_FROM_DEVICE);
		if (unlikely(dma_mapping_error(eth->dma_dev, dma_addr)))
			return -ENOMEM;

		rxd = ring->dma + i * eth->soc->txrx.rxd_size;
		rxd->rxd1 = (unsigned int)dma_addr;

		addr64 = (MTK_HAS_CAPS(eth->soc->caps, MTK_8GB_ADDRESSING)) ?
			  RX_DMA_SDP1(dma_addr) : 0;

		if (MTK_HAS_CAPS(eth->soc->caps, MTK_SOC_MT7628))
			rxd->rxd2 = RX_DMA_LSO;
		else
			rxd->rxd2 = RX_DMA_PLEN0(ring->buf_size) | addr64;

		rxd->rxd3 = 0;
		rxd->rxd4 = 0;

		if (MTK_HAS_CAPS(eth->soc->caps, MTK_NETSYS_RX_V2)) {
			rxd->rxd5 = 0;
			rxd->rxd6 = 0;
			rxd->rxd7 = 0;
			rxd->rxd8 = 0;
		}
	}
	ring->dma_size = rx_dma_size;
	ring->calc_idx_update = false;
	ring->calc_idx = rx_dma_size - 1;
	ring->crx_idx_reg = (rx_flag == MTK_RX_FLAGS_QDMA) ?
			     MTK_QRX_CRX_IDX_CFG(ring_no) :
			     MTK_PRX_CRX_IDX_CFG(ring_no);
	ring->ring_no = ring_no;
	/* make sure that all changes to the dma ring are flushed before we
	 * continue
	 */
	wmb();

	if (rx_flag == MTK_RX_FLAGS_QDMA) {
		mtk_w32(eth, ring->phys,
			reg_map->qdma.rx_ptr + ring_no * MTK_QRX_OFFSET);
		mtk_w32(eth, rx_dma_size,
			reg_map->qdma.rx_cnt_cfg + ring_no * MTK_QRX_OFFSET);
		mtk_w32(eth, ring->calc_idx,
			ring->crx_idx_reg);
		mtk_w32(eth, MTK_PST_DRX_IDX_CFG(ring_no),
			reg_map->qdma.rst_idx);
	} else {
		mtk_w32(eth, ring->phys,
			reg_map->pdma.rx_ptr + ring_no * MTK_QRX_OFFSET);
		mtk_w32(eth, rx_dma_size,
			reg_map->pdma.rx_cnt_cfg + ring_no * MTK_QRX_OFFSET);
		mtk_w32(eth, ring->calc_idx,
			ring->crx_idx_reg);
		mtk_w32(eth, MTK_PST_DRX_IDX_CFG(ring_no),
			reg_map->pdma.rst_idx);
	}

	return 0;
}

static void mtk_rx_clean(struct mtk_eth *eth, struct mtk_rx_ring *ring, int in_sram)
{
	int i;
	u64 addr64 = 0;

	if (ring->data && ring->dma) {
		for (i = 0; i < ring->dma_size; i++) {
			struct mtk_rx_dma *rxd;

			if (!ring->data[i])
				continue;

			rxd = ring->dma + i * eth->soc->txrx.rxd_size;
			if (!rxd->rxd1)
				continue;

			addr64 = (MTK_HAS_CAPS(eth->soc->caps,
					       MTK_8GB_ADDRESSING)) ?
				  ((u64)(rxd->rxd2 & 0xf)) << 32 : 0;

			dma_unmap_single(eth->dma_dev,
					 ((u64)(rxd->rxd1) | addr64),
					 ring->buf_size,
					 DMA_FROM_DEVICE);
			skb_free_frag(ring->data[i]);
		}
		kfree(ring->data);
		ring->data = NULL;
	}

	if(in_sram)
		return;

	if (ring->dma) {
		dma_free_coherent(eth->dma_dev,
				  ring->dma_size * eth->soc->txrx.rxd_size,
				  ring->dma,
				  ring->phys);
		ring->dma = NULL;
	}
}

static int mtk_hwlro_rx_init(struct mtk_eth *eth)
{
	int i;
	u32 val;
	u32 ring_ctrl_dw1 = 0, ring_ctrl_dw2 = 0, ring_ctrl_dw3 = 0;
	u32 lro_ctrl_dw0 = 0, lro_ctrl_dw3 = 0;

	/* set LRO rings to auto-learn modes */
	ring_ctrl_dw2 |= MTK_RING_AUTO_LERAN_MODE;

	/* validate LRO ring */
	ring_ctrl_dw2 |= MTK_RING_VLD;

	/* set AGE timer (unit: 20us) */
	ring_ctrl_dw2 |= MTK_RING_AGE_TIME_H;
	ring_ctrl_dw1 |= MTK_RING_AGE_TIME_L;

	/* set max AGG timer (unit: 20us) */
	ring_ctrl_dw2 |= MTK_RING_MAX_AGG_TIME;

	/* set max LRO AGG count */
	ring_ctrl_dw2 |= MTK_RING_MAX_AGG_CNT_L;
	ring_ctrl_dw3 |= MTK_RING_MAX_AGG_CNT_H;

	for (i = 1; i <= MTK_HW_LRO_RING_NUM; i++) {
		mtk_w32(eth, ring_ctrl_dw1, MTK_LRO_CTRL_DW1_CFG(i));
		mtk_w32(eth, ring_ctrl_dw2, MTK_LRO_CTRL_DW2_CFG(i));
		mtk_w32(eth, ring_ctrl_dw3, MTK_LRO_CTRL_DW3_CFG(i));
	}

	/* IPv4 checksum update enable */
	lro_ctrl_dw0 |= MTK_L3_CKS_UPD_EN;

	/* switch priority comparison to packet count mode */
	lro_ctrl_dw0 |= MTK_LRO_ALT_PKT_CNT_MODE;

	/* bandwidth threshold setting */
	mtk_w32(eth, MTK_HW_LRO_BW_THRE, MTK_PDMA_LRO_CTRL_DW2);

	/* auto-learn score delta setting */
	mtk_w32(eth, MTK_HW_LRO_REPLACE_DELTA, MTK_LRO_ALT_SCORE_DELTA);

	/* set refresh timer for altering flows to 1 sec. (unit: 20us) */
	mtk_w32(eth, (MTK_HW_LRO_TIMER_UNIT << 16) | MTK_HW_LRO_REFRESH_TIME,
		MTK_PDMA_LRO_ALT_REFRESH_TIMER);

	/* the minimal remaining room of SDL0 in RXD for lro aggregation */
	lro_ctrl_dw3 |= MTK_LRO_MIN_RXD_SDL;

	if (MTK_HAS_CAPS(eth->soc->caps, MTK_NETSYS_RX_V2)) {
		val = mtk_r32(eth, MTK_PDMA_RX_CFG);
		mtk_w32(eth, val | (MTK_PDMA_LRO_SDL << MTK_RX_CFG_SDL_OFFSET),
			MTK_PDMA_RX_CFG);

		lro_ctrl_dw0 |= MTK_PDMA_LRO_SDL << MTK_CTRL_DW0_SDL_OFFSET;
	} else {
		/* set HW LRO mode & the max aggregation count for rx packets */
		lro_ctrl_dw3 |= MTK_ADMA_MODE | (MTK_HW_LRO_MAX_AGG_CNT & 0xff);
	}

	/* enable HW LRO */
	lro_ctrl_dw0 |= MTK_LRO_EN;

	/* enable cpu reason black list */
	lro_ctrl_dw0 |= MTK_LRO_CRSN_BNW;

	mtk_w32(eth, lro_ctrl_dw3, MTK_PDMA_LRO_CTRL_DW3);
	mtk_w32(eth, lro_ctrl_dw0, MTK_PDMA_LRO_CTRL_DW0);

	/* no use PPE cpu reason */
	mtk_w32(eth, 0xffffffff, MTK_PDMA_LRO_CTRL_DW1);

	/* Set perLRO GRP INT */
	if (MTK_HAS_CAPS(eth->soc->caps, MTK_NETSYS_RX_V2) ||
	    MTK_HAS_CAPS(eth->soc->caps, MTK_NETSYS_V3)) {
		mtk_m32(eth, MTK_RX_DONE_INT(MTK_HW_LRO_RING(1)),
			MTK_RX_DONE_INT(MTK_HW_LRO_RING(1)), MTK_PDMA_INT_GRP1);
		mtk_m32(eth, MTK_RX_DONE_INT(MTK_HW_LRO_RING(2)),
			MTK_RX_DONE_INT(MTK_HW_LRO_RING(2)), MTK_PDMA_INT_GRP2);
		mtk_m32(eth, MTK_RX_DONE_INT(MTK_HW_LRO_RING(3)),
			MTK_RX_DONE_INT(MTK_HW_LRO_RING(3)), MTK_PDMA_INT_GRP3);
	}

	return 0;
}

static void mtk_hwlro_rx_uninit(struct mtk_eth *eth)
{
	int i;
	u32 val;

	/* relinquish lro rings, flush aggregated packets */
	mtk_w32(eth, MTK_LRO_RING_RELINGUISH_REQ, MTK_PDMA_LRO_CTRL_DW0);

	/* wait for relinquishments done */
	for (i = 0; i < 10; i++) {
		val = mtk_r32(eth, MTK_PDMA_LRO_CTRL_DW0);
		if (val & MTK_LRO_RING_RELINGUISH_DONE) {
			mdelay(20);
			continue;
		}
		break;
	}

	/* invalidate lro rings */
	for (i = 1; i <= MTK_HW_LRO_RING_NUM; i++)
		mtk_w32(eth, 0, MTK_LRO_CTRL_DW2_CFG(i));

	/* disable HW LRO */
	mtk_w32(eth, 0, MTK_PDMA_LRO_CTRL_DW0);
}

static void mtk_hwlro_val_ipaddr(struct mtk_eth *eth, int idx, __be32 ip)
{
	u32 reg_val;

	if (MTK_HAS_CAPS(eth->soc->caps, MTK_NETSYS_RX_V2))
		idx += 1;

	reg_val = mtk_r32(eth, MTK_LRO_CTRL_DW2_CFG(idx));

	/* invalidate the IP setting */
	mtk_w32(eth, (reg_val & ~MTK_RING_MYIP_VLD), MTK_LRO_CTRL_DW2_CFG(idx));

	mtk_w32(eth, ip, MTK_LRO_DIP_DW0_CFG(idx));

	/* validate the IP setting */
	mtk_w32(eth, (reg_val | MTK_RING_MYIP_VLD), MTK_LRO_CTRL_DW2_CFG(idx));
}

static void mtk_hwlro_inval_ipaddr(struct mtk_eth *eth, int idx)
{
	u32 reg_val;

	if (MTK_HAS_CAPS(eth->soc->caps, MTK_NETSYS_RX_V2))
		idx += 1;

	reg_val = mtk_r32(eth, MTK_LRO_CTRL_DW2_CFG(idx));

	/* invalidate the IP setting */
	mtk_w32(eth, (reg_val & ~MTK_RING_MYIP_VLD), MTK_LRO_CTRL_DW2_CFG(idx));

	mtk_w32(eth, 0, MTK_LRO_DIP_DW0_CFG(idx));
}

static int mtk_hwlro_get_ip_cnt(struct mtk_mac *mac)
{
	int cnt = 0;
	int i;

	for (i = 0; i < MTK_MAX_LRO_IP_CNT; i++) {
		if (mac->hwlro_ip[i])
			cnt++;
	}

	return cnt;
}

static int mtk_hwlro_add_ipaddr_idx(struct net_device *dev, u32 ip4dst)
{
	struct mtk_mac *mac = netdev_priv(dev);
	struct mtk_eth *eth = mac->hw;
	u32 reg_val;
	int i;

	/* check for duplicate IP address in the current DIP list */
	for (i = 1; i <= MTK_HW_LRO_RING_NUM; i++) {
		reg_val = mtk_r32(eth, MTK_LRO_DIP_DW0_CFG(i));
		if (reg_val == ip4dst)
			break;
	}

	if (i <= MTK_HW_LRO_RING_NUM) {
		netdev_warn(dev, "Duplicate IP address at DIP(%d)!\n", i);
		return -EEXIST;
	}

	/* find out available DIP index */
	for (i = 1; i <= MTK_HW_LRO_RING_NUM; i++) {
		reg_val = mtk_r32(eth, MTK_LRO_DIP_DW0_CFG(i));
		if (reg_val == 0UL)
			break;
	}

	if (i > MTK_HW_LRO_RING_NUM) {
		netdev_warn(dev, "DIP index is currently out of resource!\n");
		return -EBUSY;
	}

	if (MTK_HAS_CAPS(eth->soc->caps, MTK_NETSYS_RX_V2))
		i -= 1;

	return i;
}

static int mtk_hwlro_get_ipaddr_idx(struct net_device *dev, u32 ip4dst)
{
	struct mtk_mac *mac = netdev_priv(dev);
	struct mtk_eth *eth = mac->hw;
	u32 reg_val;
	int i;

	/* find out DIP index that matches the given IP address */
	for (i = 1; i <= MTK_HW_LRO_RING_NUM; i++) {
		reg_val = mtk_r32(eth, MTK_LRO_DIP_DW0_CFG(i));
		if (reg_val == ip4dst)
			break;
	}

	if (i > MTK_HW_LRO_RING_NUM) {
		netdev_warn(dev, "DIP address is not exist!\n");
		return -ENOENT;
	}

	if (MTK_HAS_CAPS(eth->soc->caps, MTK_NETSYS_RX_V2))
		i -= 1;

	return i;
}

static int mtk_hwlro_add_ipaddr(struct net_device *dev,
				struct ethtool_rxnfc *cmd)
{
	struct ethtool_rx_flow_spec *fsp =
		(struct ethtool_rx_flow_spec *)&cmd->fs;
	struct mtk_mac *mac = netdev_priv(dev);
	struct mtk_eth *eth = mac->hw;
	int hwlro_idx;
	u32 ip4dst;

	if ((fsp->flow_type != TCP_V4_FLOW) ||
	    (!fsp->h_u.tcp_ip4_spec.ip4dst) ||
	    (fsp->location > 1))
		return -EINVAL;

	ip4dst = htonl(fsp->h_u.tcp_ip4_spec.ip4dst);
	hwlro_idx = mtk_hwlro_add_ipaddr_idx(dev, ip4dst);
	if (hwlro_idx < 0)
		return hwlro_idx;

	mac->hwlro_ip[fsp->location] = ip4dst;
	mac->hwlro_ip_cnt = mtk_hwlro_get_ip_cnt(mac);

	mtk_hwlro_val_ipaddr(eth, hwlro_idx, mac->hwlro_ip[fsp->location]);

	return 0;
}

static int mtk_hwlro_del_ipaddr(struct net_device *dev,
				struct ethtool_rxnfc *cmd)
{
	struct ethtool_rx_flow_spec *fsp =
		(struct ethtool_rx_flow_spec *)&cmd->fs;
	struct mtk_mac *mac = netdev_priv(dev);
	struct mtk_eth *eth = mac->hw;
	int hwlro_idx;
	u32 ip4dst;

	if (fsp->location > 1)
		return -EINVAL;

	ip4dst = mac->hwlro_ip[fsp->location];
	hwlro_idx = mtk_hwlro_get_ipaddr_idx(dev, ip4dst);
	if (hwlro_idx < 0)
		return hwlro_idx;

	mac->hwlro_ip[fsp->location] = 0;
	mac->hwlro_ip_cnt = mtk_hwlro_get_ip_cnt(mac);

	mtk_hwlro_inval_ipaddr(eth, hwlro_idx);

	return 0;
}

static void mtk_hwlro_netdev_enable(struct net_device *dev)
{
	struct mtk_mac *mac = netdev_priv(dev);
	struct mtk_eth *eth = mac->hw;
	int i, hwlro_idx;

	for (i = 0; i < MTK_MAX_LRO_IP_CNT; i++) {
		if (mac->hwlro_ip[i] == 0)
			continue;

		hwlro_idx = mtk_hwlro_get_ipaddr_idx(dev, mac->hwlro_ip[i]);
		if (hwlro_idx < 0)
			continue;

		mtk_hwlro_val_ipaddr(eth, hwlro_idx, mac->hwlro_ip[i]);
	}
}

static void mtk_hwlro_netdev_disable(struct net_device *dev)
{
	struct mtk_mac *mac = netdev_priv(dev);
	struct mtk_eth *eth = mac->hw;
	int i, hwlro_idx;

	for (i = 0; i < MTK_MAX_LRO_IP_CNT; i++) {
		if (mac->hwlro_ip[i] == 0)
			continue;

		hwlro_idx = mtk_hwlro_get_ipaddr_idx(dev, mac->hwlro_ip[i]);
		if (hwlro_idx < 0)
			continue;

		mac->hwlro_ip[i] = 0;

		mtk_hwlro_inval_ipaddr(eth, hwlro_idx);
	}

	mac->hwlro_ip_cnt = 0;
}

static int mtk_hwlro_get_fdir_entry(struct net_device *dev,
				    struct ethtool_rxnfc *cmd)
{
	struct mtk_mac *mac = netdev_priv(dev);
	struct ethtool_rx_flow_spec *fsp =
		(struct ethtool_rx_flow_spec *)&cmd->fs;

	/* only tcp dst ipv4 is meaningful, others are meaningless */
	fsp->flow_type = TCP_V4_FLOW;
	fsp->h_u.tcp_ip4_spec.ip4dst = ntohl(mac->hwlro_ip[fsp->location]);
	fsp->m_u.tcp_ip4_spec.ip4dst = 0;

	fsp->h_u.tcp_ip4_spec.ip4src = 0;
	fsp->m_u.tcp_ip4_spec.ip4src = 0xffffffff;
	fsp->h_u.tcp_ip4_spec.psrc = 0;
	fsp->m_u.tcp_ip4_spec.psrc = 0xffff;
	fsp->h_u.tcp_ip4_spec.pdst = 0;
	fsp->m_u.tcp_ip4_spec.pdst = 0xffff;
	fsp->h_u.tcp_ip4_spec.tos = 0;
	fsp->m_u.tcp_ip4_spec.tos = 0xff;

	return 0;
}

static int mtk_hwlro_get_fdir_all(struct net_device *dev,
				  struct ethtool_rxnfc *cmd,
				  u32 *rule_locs)
{
	struct mtk_mac *mac = netdev_priv(dev);
	int cnt = 0;
	int i;

	for (i = 0; i < MTK_MAX_LRO_IP_CNT; i++) {
		if (mac->hwlro_ip[i]) {
			rule_locs[cnt] = i;
			cnt++;
		}
	}

	cmd->rule_cnt = cnt;

	return 0;
}

u32 mtk_rss_indr_table(struct mtk_rss_params *rss_params, int index)
{
	u32 val = 0;
	int i;

	for (i = 16 * index; i < 16 * index + 16; i++)
		val |= (rss_params->indirection_table[i] << (2 * (i % 16)));

	return val;
}

static int mtk_rss_init(struct mtk_eth *eth)
{
	struct mtk_rss_params *rss_params = &eth->rss_params;
	static u8 hash_key[MTK_RSS_HASH_KEYSIZE] = {
		0xfa, 0x01, 0xac, 0xbe, 0x3b, 0xb7, 0x42, 0x6a,
		0x0c, 0xf2, 0x30, 0x80, 0xa3, 0x2d, 0xcb, 0x77,
		0xb4, 0x30, 0x7b, 0xae, 0xcb, 0x2b, 0xca, 0xd0,
		0xb0, 0x8f, 0xa3, 0x43, 0x3d, 0x25, 0x67, 0x41,
		0xc2, 0x0e, 0x5b, 0x25, 0xda, 0x56, 0x5a, 0x6d};
	u32 val;
	int i;

	memcpy(rss_params->hash_key, hash_key, MTK_RSS_HASH_KEYSIZE);

	for (i = 0; i < MTK_RSS_MAX_INDIRECTION_TABLE; i++)
		rss_params->indirection_table[i] = i % eth->soc->rss_num;

	if (!MTK_HAS_CAPS(eth->soc->caps, MTK_NETSYS_RX_V2)) {
		/* Set RSS rings to PSE modes */
		val =  mtk_r32(eth, MTK_LRO_CTRL_DW2_CFG(1));
		val |= MTK_RING_PSE_MODE;
		mtk_w32(eth, val, MTK_LRO_CTRL_DW2_CFG(1));

		/* Enable non-lro multiple rx */
		val = mtk_r32(eth, MTK_PDMA_LRO_CTRL_DW0);
		val |= MTK_NON_LRO_MULTI_EN;
		mtk_w32(eth, val, MTK_PDMA_LRO_CTRL_DW0);

		/* Enable RSS dly int supoort */
		val |= MTK_LRO_DLY_INT_EN;
		mtk_w32(eth, val, MTK_PDMA_LRO_CTRL_DW0);
	}

	/* Hash Type */
	val = mtk_r32(eth, MTK_PDMA_RSS_GLO_CFG);
	val |= MTK_RSS_IPV4_STATIC_HASH;
	val |= MTK_RSS_IPV6_STATIC_HASH;
	mtk_w32(eth, val, MTK_PDMA_RSS_GLO_CFG);

	/* Hash Key */
	for (i = 0; i < MTK_RSS_HASH_KEYSIZE / sizeof(u32); i++)
		mtk_w32(eth, rss_params->hash_key[i], MTK_RSS_HASH_KEY_DW(i));

	/* Select the size of indirection table */
	for (i = 0; i < MTK_RSS_MAX_INDIRECTION_TABLE / 16; i++)
		mtk_w32(eth, mtk_rss_indr_table(rss_params, i),
			MTK_RSS_INDR_TABLE_DW(i));

	/* Pause */
	val |= MTK_RSS_CFG_REQ;
	mtk_w32(eth, val, MTK_PDMA_RSS_GLO_CFG);

	/* Enable RSS*/
	val |= MTK_RSS_EN;
	mtk_w32(eth, val, MTK_PDMA_RSS_GLO_CFG);

	/* Release pause */
	val &= ~(MTK_RSS_CFG_REQ);
	mtk_w32(eth, val, MTK_PDMA_RSS_GLO_CFG);

	/* Set perRSS GRP INT */
	mtk_m32(eth, MTK_RX_DONE_INT(MTK_RSS_RING(0)),
		MTK_RX_DONE_INT(MTK_RSS_RING(0)), MTK_PDMA_INT_GRP1);
	mtk_m32(eth, MTK_RX_DONE_INT(MTK_RSS_RING(1)),
		MTK_RX_DONE_INT(MTK_RSS_RING(1)), MTK_PDMA_INT_GRP2);
	mtk_m32(eth, MTK_RX_DONE_INT(MTK_RSS_RING(2)),
		MTK_RX_DONE_INT(MTK_RSS_RING(2)), MTK_PDMA_INT_GRP3);

	/* Set GRP INT */
	mtk_w32(eth, 0x210FFFF2, MTK_FE_INT_GRP);

	/* Enable RSS delay interrupt */
	if (!MTK_HAS_CAPS(eth->soc->caps, MTK_NETSYS_RX_V2)) {
		mtk_w32(eth, MTK_MAX_DELAY_INT, MTK_LRO_RX1_DLY_INT);
		mtk_w32(eth, MTK_MAX_DELAY_INT, MTK_LRO_RX2_DLY_INT);
		mtk_w32(eth, MTK_MAX_DELAY_INT, MTK_LRO_RX3_DLY_INT);
	} else
		mtk_w32(eth, MTK_MAX_DELAY_INT_V2, MTK_PDMA_RSS_DELAY_INT);

	return 0;
}

static void mtk_rss_uninit(struct mtk_eth *eth)
{
	u32 val;

	/* Pause */
	val = mtk_r32(eth, MTK_PDMA_RSS_GLO_CFG);
	val |= MTK_RSS_CFG_REQ;
	mtk_w32(eth, val, MTK_PDMA_RSS_GLO_CFG);

	/* Disable RSS*/
	val &= ~(MTK_RSS_EN);
	mtk_w32(eth, val, MTK_PDMA_RSS_GLO_CFG);

	/* Release pause */
	val &= ~(MTK_RSS_CFG_REQ);
	mtk_w32(eth, val, MTK_PDMA_RSS_GLO_CFG);
}

static netdev_features_t mtk_fix_features(struct net_device *dev,
					  netdev_features_t features)
{
	if (!(features & NETIF_F_LRO)) {
		struct mtk_mac *mac = netdev_priv(dev);
		int ip_cnt = mtk_hwlro_get_ip_cnt(mac);

		if (ip_cnt) {
			netdev_info(dev, "RX flow is programmed, LRO should keep on\n");

			features |= NETIF_F_LRO;
		}
	}

	if ((features & NETIF_F_HW_VLAN_CTAG_TX) && netdev_uses_dsa(dev)) {
		netdev_info(dev, "TX vlan offload cannot be enabled when dsa is attached.\n");

		features &= ~NETIF_F_HW_VLAN_CTAG_TX;
	}

	return features;
}

static int mtk_set_features(struct net_device *dev, netdev_features_t features)
{
	struct mtk_mac *mac = netdev_priv(dev);
	struct mtk_eth *eth = mac->hw;
	netdev_features_t lro;
	int err = 0;

	if (!((dev->features ^ features) & MTK_SET_FEATURES))
		return 0;

	lro = dev->features & NETIF_F_LRO;
	if (!(features & NETIF_F_LRO) && lro)
		mtk_hwlro_netdev_disable(dev);
	else if ((features & NETIF_F_LRO) && !lro)
		mtk_hwlro_netdev_enable(dev);

	if (!(features & NETIF_F_HW_VLAN_CTAG_RX))
		mtk_w32(eth, 0, MTK_CDMP_EG_CTRL);
	else
		mtk_w32(eth, 1, MTK_CDMP_EG_CTRL);

	return err;
}

/* wait for DMA to finish whatever it is doing before we start using it again */
static int mtk_dma_busy_wait(struct mtk_eth *eth)
{
	unsigned long t_start = jiffies;

	while (1) {
		if (MTK_HAS_CAPS(eth->soc->caps, MTK_QDMA)) {
			if (!(mtk_r32(eth, MTK_QDMA_GLO_CFG) &
			      (MTK_RX_DMA_BUSY | MTK_TX_DMA_BUSY)))
				return 0;
		} else {
			if (!(mtk_r32(eth, MTK_PDMA_GLO_CFG) &
			      (MTK_RX_DMA_BUSY | MTK_TX_DMA_BUSY)))
				return 0;
		}

		if (time_after(jiffies, t_start + MTK_DMA_BUSY_TIMEOUT))
			break;
	}

	dev_err(eth->dev, "DMA init timeout\n");
	return -1;
}

static int mtk_dma_init(struct mtk_eth *eth)
{
	int err;
	u32 i;

	if (mtk_dma_busy_wait(eth))
		return -EBUSY;

	if (MTK_HAS_CAPS(eth->soc->caps, MTK_QDMA)) {
		/* QDMA needs scratch memory for internal reordering of the
		 * descriptors
		 */
		err = mtk_init_fq_dma(eth);
		if (err)
			return err;
	}

	err = mtk_tx_alloc(eth);
	if (err)
		return err;

	if (MTK_HAS_CAPS(eth->soc->caps, MTK_QDMA)) {
		err = mtk_rx_alloc(eth, 0, MTK_RX_FLAGS_QDMA);
		if (err)
			return err;
	}

	err = mtk_rx_alloc(eth, 0, MTK_RX_FLAGS_NORMAL);
	if (err)
		return err;

	if (eth->hwlro) {
		for (i = 0; i < MTK_HW_LRO_RING_NUM; i++) {
			err = mtk_rx_alloc(eth, MTK_HW_LRO_RING(i), MTK_RX_FLAGS_HWLRO);
			if (err)
				return err;
		}
		err = mtk_hwlro_rx_init(eth);
		if (err)
			return err;
	}

	if (MTK_HAS_CAPS(eth->soc->caps, MTK_RSS)) {
		for (i = 0; i < MTK_RX_RSS_NUM; i++) {
			err = mtk_rx_alloc(eth, MTK_RSS_RING(i), MTK_RX_FLAGS_NORMAL);
			if (err)
				return err;
		}
		err = mtk_rss_init(eth);
		if (err)
                        return err;
	}

	if (MTK_HAS_CAPS(eth->soc->caps, MTK_QDMA)) {
		/* Enable random early drop and set drop threshold
		 * automatically
		 */
		mtk_w32(eth, FC_THRES_DROP_MODE | FC_THRES_DROP_EN |
			FC_THRES_MIN, eth->soc->reg_map->qdma.fc_th);
		mtk_w32(eth, 0x0, eth->soc->reg_map->qdma.hred2);
	}

	return 0;
}

static void mtk_dma_free(struct mtk_eth *eth)
{
	const struct mtk_soc_data *soc = eth->soc;
	int i;

	for (i = 0; i < MTK_MAC_COUNT; i++)
		if (eth->netdev[i])
			netdev_reset_queue(eth->netdev[i]);
	if ( !eth->soc->has_sram && eth->scratch_ring) {
		dma_free_coherent(eth->dma_dev,
				  soc->txrx.fq_dma_size * soc->txrx.txd_size,
				  eth->scratch_ring, eth->phy_scratch_ring);
		eth->scratch_ring = NULL;
		eth->phy_scratch_ring = 0;
	}
	mtk_tx_clean(eth);
	mtk_rx_clean(eth, &eth->rx_ring[0],eth->soc->has_sram);
	mtk_rx_clean(eth, &eth->rx_ring_qdma,0);

	if (eth->hwlro) {
		mtk_hwlro_rx_uninit(eth);

		for (i = 0; i < MTK_HW_LRO_RING_NUM; i++)
			mtk_rx_clean(eth, &eth->rx_ring[MTK_HW_LRO_RING(i)], 0);
	}

	if (MTK_HAS_CAPS(eth->soc->caps, MTK_RSS)) {
		mtk_rss_uninit(eth);

		for (i = 0; i < MTK_RX_RSS_NUM; i++)
			mtk_rx_clean(eth, &eth->rx_ring[MTK_RSS_RING(i)], 1);
	}

	for (i = 0; i < DIV_ROUND_UP(soc->txrx.fq_dma_size, MTK_FQ_DMA_LENGTH); i++) {
		kfree(eth->scratch_head[i]);
		eth->scratch_head[i] = NULL;
	}
}

static void mtk_tx_timeout(struct net_device *dev)
{
	struct mtk_mac *mac = netdev_priv(dev);
	struct mtk_eth *eth = mac->hw;

	eth->netdev[mac->id]->stats.tx_errors++;
	netif_err(eth, tx_err, dev,
		  "transmit timed out\n");


}

static irqreturn_t mtk_handle_irq_rx(int irq, void *priv)
{
	struct mtk_napi *rx_napi = priv;
	struct mtk_eth *eth = rx_napi->eth;
	struct mtk_rx_ring *ring = rx_napi->rx_ring;

	if (unlikely(!(mtk_r32(eth, eth->soc->reg_map->pdma.irq_status) &
		       mtk_r32(eth, eth->soc->reg_map->pdma.irq_mask) &
		       MTK_RX_DONE_INT(ring->ring_no))))
		return IRQ_NONE;

	if (likely(napi_schedule_prep(&rx_napi->napi))) {
		mtk_rx_irq_disable(eth, MTK_RX_DONE_INT(ring->ring_no));
		__napi_schedule(&rx_napi->napi);
	}

	return IRQ_HANDLED;
}

static irqreturn_t mtk_handle_irq_tx(int irq, void *_eth)
{
	struct mtk_eth *eth = _eth;

	if (likely(napi_schedule_prep(&eth->tx_napi))) {
		mtk_tx_irq_disable(eth, MTK_TX_DONE_INT(0));
		__napi_schedule(&eth->tx_napi);
	}

	return IRQ_HANDLED;
}

static irqreturn_t mtk_handle_irq(int irq, void *_eth)
{
	struct mtk_eth *eth = _eth;
	const struct mtk_reg_map *reg_map = eth->soc->reg_map;

	if (mtk_r32(eth, reg_map->pdma.irq_mask) & MTK_RX_DONE_INT(0)) {
		if (mtk_r32(eth, reg_map->pdma.irq_status) & MTK_RX_DONE_INT(0))
			mtk_handle_irq_rx(irq, &eth->rx_napi[0]);
	}

	if (MTK_HAS_CAPS(eth->soc->caps, MTK_QDMA)) {
		if (mtk_r32(eth, reg_map->tx_irq_mask) & MTK_TX_DONE_INT(0)) {
			if (mtk_r32(eth, reg_map->tx_irq_status) & MTK_TX_DONE_INT(0))
				mtk_handle_irq_tx(irq, _eth);
		}
	} else {
		if (mtk_r32(eth, reg_map->pdma.irq_mask) & MTK_TX_DONE_INT(0)) {
			if (mtk_r32(eth, reg_map->pdma.irq_status) & MTK_TX_DONE_INT(0))
				mtk_handle_irq_tx(irq, _eth);
		}
	}

	return IRQ_HANDLED;
}

static irqreturn_t mtk_handle_irq_fixed_link(int irq, void *_mac)
{
	struct mtk_mac *mac = _mac;
	struct mtk_eth *eth = mac->hw;
	struct mtk_phylink_priv *phylink_priv = &mac->phylink_priv;
	struct net_device *dev = phylink_priv->dev;
	int link_old, link_new;

	// clear interrupt status for gpy211
	_mtk_mdio_read(eth, phylink_priv->phyaddr, 0x1A);

	link_old = phylink_priv->link;
	link_new = _mtk_mdio_read(eth, phylink_priv->phyaddr, MII_BMSR) & BMSR_LSTATUS;

	if (link_old != link_new) {
		phylink_priv->link = link_new;
		if (link_new) {
			printk("phylink.%d %s: Link is Up\n", phylink_priv->id, dev->name);
			if (dev)
				netif_carrier_on(dev);
		} else {
			printk("phylink.%d %s: Link is Down\n", phylink_priv->id, dev->name);
			if (dev)
				netif_carrier_off(dev);
		}
	}

	return IRQ_HANDLED;
}

#ifdef CONFIG_NET_POLL_CONTROLLER
static void mtk_poll_controller(struct net_device *dev)
{
	struct mtk_mac *mac = netdev_priv(dev);
	struct mtk_eth *eth = mac->hw;

	mtk_tx_irq_disable(eth, MTK_TX_DONE_INT(0));
	mtk_rx_irq_disable(eth, MTK_RX_DONE_INT(0));
	mtk_handle_irq_rx(eth->irq_fe[2], &eth->rx_napi[0]);
	mtk_tx_irq_enable(eth, MTK_TX_DONE_INT(0));
	mtk_rx_irq_enable(eth, MTK_RX_DONE_INT(0));
}
#endif

static int mtk_start_dma(struct mtk_eth *eth)
{
	u32 rx_2b_offset = (NET_IP_ALIGN == 2) ? MTK_RX_2B_OFFSET : 0;
	const struct mtk_reg_map *reg_map = eth->soc->reg_map;
	int val, err;

	err = mtk_dma_init(eth);
	if (err) {
		mtk_dma_free(eth);
		return err;
	}

	if (MTK_HAS_CAPS(eth->soc->caps, MTK_QDMA)) {
		val = mtk_r32(eth, reg_map->qdma.glo_cfg);
		if (MTK_HAS_CAPS(eth->soc->caps, MTK_NETSYS_V2) ||
		    MTK_HAS_CAPS(eth->soc->caps, MTK_NETSYS_V3)) {
			val &= ~(MTK_RESV_BUF_MASK | MTK_DMA_SIZE_MASK);
			mtk_w32(eth,
				val | MTK_TX_DMA_EN | MTK_RX_DMA_EN |
				MTK_DMA_SIZE_32DWORDS | MTK_TX_WB_DDONE |
				MTK_NDP_CO_PRO | MTK_MUTLI_CNT |
				MTK_RESV_BUF | MTK_WCOMP_EN |
				MTK_DMAD_WR_WDONE | MTK_CHK_DDONE_EN |
				MTK_RX_2B_OFFSET, reg_map->qdma.glo_cfg);
		} else
			mtk_w32(eth,
				val | MTK_TX_DMA_EN |
				MTK_DMA_SIZE_32DWORDS | MTK_NDP_CO_PRO |
				MTK_RX_DMA_EN | MTK_RX_2B_OFFSET |
				MTK_RX_BT_32DWORDS,
				reg_map->qdma.glo_cfg);

		val = mtk_r32(eth, reg_map->pdma.glo_cfg);
		mtk_w32(eth,
			val | MTK_RX_DMA_EN | rx_2b_offset |
			MTK_RX_BT_32DWORDS | MTK_MULTI_EN,
			reg_map->pdma.glo_cfg);
	} else {
		mtk_w32(eth, MTK_TX_WB_DDONE | MTK_TX_DMA_EN | MTK_RX_DMA_EN |
			MTK_MULTI_EN | MTK_PDMA_SIZE_8DWORDS,
			reg_map->pdma.glo_cfg);
	}

	if (MTK_HAS_CAPS(eth->soc->caps, MTK_NETSYS_RX_V2) && eth->hwlro) {
		val = mtk_r32(eth, MTK_PDMA_GLO_CFG);
		mtk_w32(eth, val | MTK_RX_DMA_LRO_EN, MTK_PDMA_GLO_CFG);
	}

	return 0;
}

void mtk_gdm_config(struct mtk_eth *eth, u32 id, u32 config)
{
	u32 val;

	if (MTK_HAS_CAPS(eth->soc->caps, MTK_SOC_MT7628))
		return;

	val = mtk_r32(eth, MTK_GDMA_FWD_CFG(id));

	/* default setup the forward port to send frame to PDMA */
	val &= ~0xffff;

	/* Enable RX checksum */
	val |= MTK_GDMA_ICS_EN | MTK_GDMA_TCS_EN | MTK_GDMA_UCS_EN;

	val |= config;

	if (eth->netdev[id] && netdev_uses_dsa(eth->netdev[id]))
		val |= MTK_GDMA_SPECIAL_TAG;

	mtk_w32(eth, val, MTK_GDMA_FWD_CFG(id));
}

void mtk_set_pse_drop(u32 config)
{
	struct mtk_eth *eth = g_eth;

	if (eth)
		mtk_w32(eth, config, PSE_PPE0_DROP);
}
EXPORT_SYMBOL(mtk_set_pse_drop);

static int mtk_device_event(struct notifier_block *n, unsigned long event, void *ptr)
{
	struct mtk_mac *mac = container_of(n, struct mtk_mac, device_notifier);
	struct mtk_eth *eth = mac->hw;
	struct net_device *dev = netdev_notifier_info_to_dev(ptr);
	struct ethtool_link_ksettings s;
	struct net_device *ldev;
	struct list_head *iter;
	struct dsa_port *dp;
	unsigned int queue = 0;

	if (!eth->pppq_toggle)
		return NOTIFY_DONE;

	if (event != NETDEV_CHANGE)
		return NOTIFY_DONE;

	switch (mac->id) {
	case MTK_GMAC1_ID:
		netdev_for_each_lower_dev(dev, ldev, iter) {
			if (netdev_priv(ldev) == mac)
				goto dsa_set_speed;
		}
		break;
	case MTK_GMAC2_ID:
		if (strcmp(netdev_name(dev), "eth1"))
			break;

		queue = MTK_QDMA_GMAC2_QID;
		goto set_speed;
	case MTK_GMAC3_ID:
		if (strcmp(netdev_name(dev), "eth2"))
			break;

		queue = MTK_QDMA_GMAC3_QID;
		goto set_speed;
	default:
		pr_info("%s mac id invalid", __func__);
		break;
	}

	return NOTIFY_DONE;

set_speed:
	if (__ethtool_get_link_ksettings(dev, &s))
		return NOTIFY_DONE;

	if (s.base.speed == 0 || s.base.speed == ((__u32)-1))
		return NOTIFY_DONE;

	if (mac->speed > 0 && mac->speed < s.base.speed)
		s.base.speed = 0;

	mtk_set_queue_speed(eth, queue, s.base.speed);

	return NOTIFY_DONE;

dsa_set_speed:
	if (!dsa_slave_dev_check(dev))
		return NOTIFY_DONE;

	if (__ethtool_get_link_ksettings(dev, &s))
		return NOTIFY_DONE;

	if (s.base.speed == 0 || s.base.speed == ((__u32)-1))
		return NOTIFY_DONE;

	dp = dsa_port_from_netdev(dev);
	if (dp->index >= MTK_QDMA_TX_NUM)
		return NOTIFY_DONE;

	if (mac->speed > 0 && mac->speed <= s.base.speed)
		s.base.speed = 0;

	mtk_set_queue_speed(eth, dp->index, s.base.speed);

	return NOTIFY_DONE;
}

static int mtk_open(struct net_device *dev)
{
	struct mtk_mac *mac = netdev_priv(dev);
	struct mtk_eth *eth = mac->hw;
	struct mtk_phylink_priv *phylink_priv = &mac->phylink_priv;
	u32 id = mtk_mac2xgmii_id(eth, mac->id);
	int err, i;
	struct device_node *phy_node;

	err = phylink_of_phy_connect(mac->phylink, mac->of_node, 0);
	if (err) {
		netdev_err(dev, "%s: could not attach PHY: %d\n", __func__,
			   err);
		return err;
	}

	/* we run 2 netdevs on the same dma ring so we only bring it up once */
	if (!refcount_read(&eth->dma_refcnt)) {
		int err = mtk_start_dma(eth);

		if (err)
			return err;


		/* Indicates CDM to parse the MTK special tag from CPU */
		if (netdev_uses_dsa(dev)) {
			u32 val;
			val = mtk_r32(eth, MTK_CDMQ_IG_CTRL);
			mtk_w32(eth, val | MTK_CDMQ_STAG_EN, MTK_CDMQ_IG_CTRL);
			val = mtk_r32(eth, MTK_CDMP_IG_CTRL);
			mtk_w32(eth, val | MTK_CDMP_STAG_EN, MTK_CDMP_IG_CTRL);
		}

		napi_enable(&eth->tx_napi);
		napi_enable(&eth->rx_napi[0].napi);
		mtk_tx_irq_enable(eth, MTK_TX_DONE_INT(0));
		mtk_rx_irq_enable(eth, MTK_RX_DONE_INT(0));

		if (MTK_HAS_CAPS(eth->soc->caps, MTK_RSS)) {
			for (i = 0; i < MTK_RX_RSS_NUM; i++) {
				napi_enable(&eth->rx_napi[MTK_RSS_RING(i)].napi);
				mtk_rx_irq_enable(eth, MTK_RX_DONE_INT(MTK_RSS_RING(i)));
			}
		}

		if (MTK_HAS_CAPS(eth->soc->caps, MTK_HWLRO)) {
			for (i = 0; i < MTK_HW_LRO_RING_NUM; i++) {
				napi_enable(&eth->rx_napi[MTK_HW_LRO_RING(i)].napi);
				mtk_rx_irq_enable(eth, MTK_RX_DONE_INT(MTK_HW_LRO_RING(i)));
			}
		}

		refcount_set(&eth->dma_refcnt, 1);
	}
	else
		refcount_inc(&eth->dma_refcnt);

	if (phylink_priv->desc) {
		/*Notice: This programming sequence is only for GPY211 single PHY chip.
		  If single PHY chip is not GPY211, the following step you should do:
		  1. Contact your Single PHY chip vendor and get the details of
		    - how to enables link status change interrupt
		    - how to clears interrupt source
		 */

		// clear interrupt source for gpy211
		_mtk_mdio_read(eth, phylink_priv->phyaddr, 0x1A);

		// enable link status change interrupt for gpy211
		_mtk_mdio_write(eth, phylink_priv->phyaddr, 0x19, 0x0001);

		phylink_priv->dev = dev;

		// override dev pointer for single PHY chip 0
		if (phylink_priv->id == 0) {
			struct net_device *tmp;

			tmp = __dev_get_by_name(&init_net, phylink_priv->label);
			if (tmp)
				phylink_priv->dev = tmp;
			else
				phylink_priv->dev = NULL;
		}
	}

	phylink_start(mac->phylink);
	netif_tx_start_all_queues(dev);
	phy_node = of_parse_phandle(mac->of_node, "phy-handle", 0);
	if (!phy_node && eth->sgmii->pcs[id].regmap)
		regmap_write(eth->sgmii->pcs[id].regmap,
			     SGMSYS_QPHY_PWR_STATE_CTRL, 0);

	mtk_gdm_config(eth, mac->id, MTK_GDMA_TO_PDMA);

	return 0;
}

static void mtk_stop_dma(struct mtk_eth *eth, u32 glo_cfg)
{
	u32 val;
	int i;

	/* stop the dma engine */
	spin_lock_bh(&eth->page_lock);
	val = mtk_r32(eth, glo_cfg);
	mtk_w32(eth, val & ~(MTK_TX_WB_DDONE | MTK_RX_DMA_EN | MTK_TX_DMA_EN),
		glo_cfg);
	spin_unlock_bh(&eth->page_lock);

	/* wait for dma stop */
	for (i = 0; i < 10; i++) {
		val = mtk_r32(eth, glo_cfg);
		if (val & (MTK_TX_DMA_BUSY | MTK_RX_DMA_BUSY)) {
			mdelay(20);
			continue;
		}
		break;
	}
}

static int mtk_stop(struct net_device *dev)
{
	struct mtk_mac *mac = netdev_priv(dev);
	struct mtk_eth *eth = mac->hw;
	int i;
	u32 id = mtk_mac2xgmii_id(eth, mac->id);
	u32 val = 0;
	struct device_node *phy_node;

	mtk_gdm_config(eth, mac->id, MTK_GDMA_DROP_ALL);
	netif_tx_disable(dev);

	phy_node = of_parse_phandle(mac->of_node, "phy-handle", 0);
	if (!phy_node && eth->sgmii->pcs[id].regmap) {
		regmap_read(eth->sgmii->pcs[id].regmap,
			    SGMSYS_QPHY_PWR_STATE_CTRL, &val);
		val |= SGMII_PHYA_PWD;
		regmap_write(eth->sgmii->pcs[id].regmap,
			     SGMSYS_QPHY_PWR_STATE_CTRL, val);
	}

	//GMAC RX disable
	val = mtk_r32(eth, MTK_MAC_MCR(mac->id));
	mtk_w32(eth, val & ~(MAC_MCR_RX_EN), MTK_MAC_MCR(mac->id));

	phylink_stop(mac->phylink);

	phylink_disconnect_phy(mac->phylink);

	/* only shutdown DMA if this is the last user */
	if (!refcount_dec_and_test(&eth->dma_refcnt))
		return 0;


	mtk_tx_irq_disable(eth, MTK_TX_DONE_INT(0));
	mtk_rx_irq_disable(eth, MTK_RX_DONE_INT(0));
	napi_disable(&eth->tx_napi);
	napi_disable(&eth->rx_napi[0].napi);

	if (MTK_HAS_CAPS(eth->soc->caps, MTK_RSS)) {
		for (i = 0; i < MTK_RX_RSS_NUM; i++) {
			mtk_rx_irq_disable(eth, MTK_RX_DONE_INT(MTK_RSS_RING(i)));
			napi_disable(&eth->rx_napi[MTK_RSS_RING(i)].napi);
		}
	}

	if (MTK_HAS_CAPS(eth->soc->caps, MTK_HWLRO)) {
		for (i = 0; i < MTK_HW_LRO_RING_NUM; i++) {
			mtk_rx_irq_disable(eth, MTK_RX_DONE_INT(MTK_HW_LRO_RING(i)));
			napi_disable(&eth->rx_napi[MTK_HW_LRO_RING(i)].napi);
		}
	}

	if (MTK_HAS_CAPS(eth->soc->caps, MTK_QDMA))
		mtk_stop_dma(eth, eth->soc->reg_map->qdma.glo_cfg);
	mtk_stop_dma(eth, eth->soc->reg_map->pdma.glo_cfg);

	mtk_dma_free(eth);

	return 0;
}

void ethsys_reset(struct mtk_eth *eth, u32 reset_bits)
{
	u32 val = 0, i = 0;

	regmap_update_bits(eth->ethsys, ETHSYS_RSTCTRL,
			   reset_bits, reset_bits);

	while (i++ < 5000) {
		mdelay(1);
		regmap_read(eth->ethsys, ETHSYS_RSTCTRL, &val);

		if ((val & reset_bits) == reset_bits) {
			mtk_reset_event_update(eth, MTK_EVENT_COLD_CNT);
			regmap_update_bits(eth->ethsys, ETHSYS_RSTCTRL,
					   reset_bits, ~reset_bits);
			break;
		}
	}

	mdelay(10);
}

static void mtk_clk_disable(struct mtk_eth *eth)
{
	int clk;

	for (clk = MTK_CLK_MAX - 1; clk >= 0; clk--)
		clk_disable_unprepare(eth->clks[clk]);
}

static int mtk_clk_enable(struct mtk_eth *eth)
{
	int clk, ret;

	for (clk = 0; clk < MTK_CLK_MAX ; clk++) {
		ret = clk_prepare_enable(eth->clks[clk]);
		if (ret)
			goto err_disable_clks;
	}

	return 0;

err_disable_clks:
	while (--clk >= 0)
		clk_disable_unprepare(eth->clks[clk]);

	return ret;
}

static int mtk_napi_init(struct mtk_eth *eth)
{
	struct mtk_napi *rx_napi = &eth->rx_napi[0];
	int i;

	rx_napi->eth = eth;
	rx_napi->rx_ring = &eth->rx_ring[0];
	rx_napi->irq_grp_no = 2;

	if (MTK_HAS_CAPS(eth->soc->caps, MTK_RSS)) {
		for (i = 0; i < MTK_RX_RSS_NUM; i++) {
			rx_napi = &eth->rx_napi[MTK_RSS_RING(i)];
			rx_napi->eth = eth;
			rx_napi->rx_ring = &eth->rx_ring[MTK_RSS_RING(i)];
			rx_napi->irq_grp_no = 2 + i;
		}
	}

	if (MTK_HAS_CAPS(eth->soc->caps, MTK_HWLRO)) {
		for (i = 0; i < MTK_HW_LRO_RING_NUM; i++) {
			rx_napi = &eth->rx_napi[MTK_HW_LRO_RING(i)];
			rx_napi->eth = eth;
			rx_napi->rx_ring = &eth->rx_ring[MTK_HW_LRO_RING(i)];
			rx_napi->irq_grp_no = 2;
		}
	}

	return 0;
}

static int mtk_hw_init(struct mtk_eth *eth, u32 type)
{
	u32 dma_mask = ETHSYS_DMA_AG_MAP_PDMA | ETHSYS_DMA_AG_MAP_QDMA |
		       ETHSYS_DMA_AG_MAP_PPE;
	const struct mtk_reg_map *reg_map = eth->soc->reg_map;
	int i, ret = 0;
	u32 val;

	pr_info("[%s] reset_lock:%d, force:%d\n", __func__,
		atomic_read(&reset_lock), atomic_read(&force));

	if (atomic_read(&reset_lock) == 0) {
		if (test_and_set_bit(MTK_HW_INIT, &eth->state))
			return 0;

		pm_runtime_enable(eth->dev);
		pm_runtime_get_sync(eth->dev);

		ret = mtk_clk_enable(eth);
		if (ret)
			goto err_disable_pm;
	}

	if (eth->ethsys)
		regmap_update_bits(eth->ethsys, ETHSYS_DMA_AG_MAP, dma_mask,
				   of_dma_is_coherent(eth->dma_dev->of_node) *
				   dma_mask);

	if (MTK_HAS_CAPS(eth->soc->caps, MTK_SOC_MT7628)) {
		ret = device_reset(eth->dev);
		if (ret) {
			dev_err(eth->dev, "MAC reset failed!\n");
			goto err_disable_pm;
		}

		/* enable interrupt delay for RX */
		mtk_w32(eth, MTK_PDMA_DELAY_RX_DELAY, MTK_PDMA_DELAY_INT);

		/* disable delay and normal interrupt */
		mtk_tx_irq_disable(eth, ~0);
		mtk_rx_irq_disable(eth, ~0);

		return 0;
	}

	pr_info("[%s] execute fe %s reset\n", __func__,
		(type == MTK_TYPE_WARM_RESET) ? "warm" : "cold");

	if (type == MTK_TYPE_WARM_RESET)
		mtk_eth_warm_reset(eth);
	else
		mtk_eth_cold_reset(eth);

	if (!MTK_HAS_CAPS(eth->soc->caps, MTK_SOC_MT7628))
		mtk_mdc_init(eth);

	if (MTK_HAS_CAPS(eth->soc->caps, MTK_NETSYS_RX_V2)) {
		/* Set FE to PDMAv2 if necessary */
		mtk_w32(eth, mtk_r32(eth, MTK_FE_GLO_MISC) | MTK_PDMA_V2, MTK_FE_GLO_MISC);
	}

	if (eth->pctl) {
		/* Set GE2 driving and slew rate */
		regmap_write(eth->pctl, GPIO_DRV_SEL10, 0xa00);

		/* set GE2 TDSEL */
		regmap_write(eth->pctl, GPIO_OD33_CTRL8, 0x5);

		/* set GE2 TUNE */
		regmap_write(eth->pctl, GPIO_BIAS_CTRL, 0x0);
	}

	/* Set linkdown as the default for each GMAC. Its own MCR would be set
	 * up with the more appropriate value when mtk_mac_config call is being
	 * invoked.
	 */
	for (i = 0; i < MTK_MAC_COUNT; i++)
		mtk_w32(eth, MAC_MCR_FORCE_LINK_DOWN, MTK_MAC_MCR(i));

	/* Enable RX VLan Offloading */
	if (eth->soc->hw_features & NETIF_F_HW_VLAN_CTAG_RX)
		mtk_w32(eth, 1, MTK_CDMP_EG_CTRL);
	else
		mtk_w32(eth, 0, MTK_CDMP_EG_CTRL);

	/* enable interrupt delay for RX/TX */
	mtk_w32(eth, 0x8f0f8f0f, MTK_PDMA_DELAY_INT);
	mtk_w32(eth, 0x8f0f8f0f, MTK_QDMA_DELAY_INT);
	if (MTK_HAS_CAPS(eth->soc->caps, MTK_NETSYS_V3)) {
		mtk_w32(eth, 0x8f0f8f0f, MTK_PDMA_TX_DELAY_INT0);
		mtk_w32(eth, 0x8f0f8f0f, MTK_PDMA_TX_DELAY_INT1);
	}

	mtk_tx_irq_disable(eth, ~0);
	mtk_rx_irq_disable(eth, ~0);

	/* FE int grouping */
	if (MTK_HAS_CAPS(eth->soc->caps, MTK_QDMA))
		mtk_w32(eth, MTK_TX_DONE_INT(0), reg_map->qdma.int_grp);
	else
		mtk_w32(eth, MTK_TX_DONE_INT(0), reg_map->pdma.int_grp);
	mtk_w32(eth, MTK_RX_DONE_INT(0), reg_map->qdma.int_grp2);
	if (MTK_HAS_CAPS(eth->soc->caps, MTK_PDMA_INT)) {
		if (MTK_HAS_CAPS(eth->soc->caps, MTK_QDMA))
			mtk_w32(eth, 0x210FFFF2, MTK_FE_INT_GRP);
		else
			mtk_w32(eth, 0xFFFF1FF2, MTK_FE_INT_GRP);
	} else {
		mtk_w32(eth, MTK_RX_DONE_INT(0), reg_map->pdma.int_grp);
		mtk_w32(eth, 0x210F2FF3, MTK_FE_INT_GRP);
	}
	mtk_w32(eth, MTK_FE_INT_TSO_FAIL |
		MTK_FE_INT_TSO_ILLEGAL | MTK_FE_INT_TSO_ALIGN |
		MTK_FE_INT_RFIFO_OV | MTK_FE_INT_RFIFO_UF, MTK_FE_INT_ENABLE);

	if (MTK_HAS_CAPS(eth->soc->caps, MTK_NETSYS_V3)) {
		/* PSE dummy page mechanism */
		if (eth->soc->caps != MT7988_CAPS || eth->hwver != MTK_HWID_V1)
			mtk_w32(eth, PSE_DUMMY_WORK_GDM(1) |
				PSE_DUMMY_WORK_GDM(2) |	PSE_DUMMY_WORK_GDM(3) |
				DUMMY_PAGE_THR, PSE_DUMY_REQ);

		/* PSE should not drop port1, port8 and port9 packets */
		mtk_w32(eth, 0x00000302, PSE_NO_DROP_CFG);

		/* PSE should drop p8 and p9 packets when WDMA Rx ring full*/
		mtk_w32(eth, 0x00000300, PSE_PPE0_DROP);

		/* PSE free buffer drop threshold */
		mtk_w32(eth, 0x00600009, PSE_IQ_REV(8));

		/* GDM and CDM Threshold */
		mtk_w32(eth, 0x00000004, MTK_CDM2_THRES);
		mtk_w32(eth, 0x08000707, MTK_CDMW0_THRES);
		mtk_w32(eth, 0x00000077, MTK_CDMW1_THRES);

		/* Disable GDM1 RX CRC stripping */
		val = mtk_r32(eth, MTK_GDMA_FWD_CFG(0));
		val &= ~MTK_GDMA_STRP_CRC;
		mtk_w32(eth, val, MTK_GDMA_FWD_CFG(0));

		/* PSE GDM3 MIB counter has incorrect hw default values,
		 * so the driver ought to read clear the values beforehand
		 * in case ethtool retrieve wrong mib values.
		 */
		for (i = 0; i < MTK_STAT_OFFSET; i += 0x4)
			mtk_r32(eth,
				MTK_GDM1_TX_GBCNT + MTK_STAT_OFFSET * 2 + i);
	} else if (MTK_HAS_CAPS(eth->soc->caps, MTK_NETSYS_V2)) {
		/* PSE Free Queue Flow Control  */
		mtk_w32(eth, 0x01fa01f4, PSE_FQFC_CFG2);

		/* PSE should not drop port8 and port9 packets from WDMA Tx */
		mtk_w32(eth, 0x00000300, PSE_NO_DROP_CFG);

		/* PSE should drop p8 and p9 packets when WDMA Rx ring full*/
		mtk_w32(eth, 0x00000300, PSE_PPE0_DROP);

		/* PSE config input queue threshold */
		mtk_w32(eth, 0x001a000e, PSE_IQ_REV(1));
		mtk_w32(eth, 0x01ff001a, PSE_IQ_REV(2));
		mtk_w32(eth, 0x000e01ff, PSE_IQ_REV(3));
		mtk_w32(eth, 0x000e000e, PSE_IQ_REV(4));
		mtk_w32(eth, 0x000e000e, PSE_IQ_REV(5));
		mtk_w32(eth, 0x000e000e, PSE_IQ_REV(6));
		mtk_w32(eth, 0x000e000e, PSE_IQ_REV(7));
		mtk_w32(eth, 0x002a000e, PSE_IQ_REV(8));

		/* PSE config output queue threshold */
		mtk_w32(eth, 0x000f000a, PSE_OQ_TH(1));
		mtk_w32(eth, 0x001a000f, PSE_OQ_TH(2));
		mtk_w32(eth, 0x000f001a, PSE_OQ_TH(3));
		mtk_w32(eth, 0x01ff000f, PSE_OQ_TH(4));
		mtk_w32(eth, 0x000f000f, PSE_OQ_TH(5));
		mtk_w32(eth, 0x0006000f, PSE_OQ_TH(6));
		mtk_w32(eth, 0x00060006, PSE_OQ_TH(7));
		mtk_w32(eth, 0x00060006, PSE_OQ_TH(8));

		/* GDM and CDM Threshold */
		mtk_w32(eth, 0x00000004, MTK_GDM2_THRES);
                mtk_w32(eth, 0x00000004, MTK_CDMW0_THRES);
                mtk_w32(eth, 0x00000004, MTK_CDMW1_THRES);
                mtk_w32(eth, 0x00000004, MTK_CDME0_THRES);
                mtk_w32(eth, 0x00000004, MTK_CDME1_THRES);
                mtk_w32(eth, 0x00000004, MTK_CDMM_THRES);
	}

	return 0;

err_disable_pm:
	pm_runtime_put_sync(eth->dev);
	pm_runtime_disable(eth->dev);

	return ret;
}

static int mtk_hw_deinit(struct mtk_eth *eth)
{
	if (!test_and_clear_bit(MTK_HW_INIT, &eth->state))
		return 0;

	mtk_clk_disable(eth);

	pm_runtime_put_sync(eth->dev);
	pm_runtime_disable(eth->dev);

	return 0;
}

static int __init mtk_init(struct net_device *dev)
{
	struct mtk_mac *mac = netdev_priv(dev);
	struct mtk_eth *eth = mac->hw;
	const char *mac_addr;

	mac_addr = of_get_mac_address(mac->of_node);
	if (!IS_ERR(mac_addr))
		ether_addr_copy(dev->dev_addr, mac_addr);

	/* If the mac address is invalid, use random mac address  */
	if (!is_valid_ether_addr(dev->dev_addr)) {
		eth_hw_addr_random(dev);
		dev_err(eth->dev, "generated random MAC address %pM\n",
			dev->dev_addr);
	}

	return 0;
}

static void mtk_uninit(struct net_device *dev)
{
	struct mtk_mac *mac = netdev_priv(dev);
	struct mtk_eth *eth = mac->hw;

	phylink_disconnect_phy(mac->phylink);
	mtk_tx_irq_disable(eth, ~0);
	mtk_rx_irq_disable(eth, ~0);
}

static int mtk_do_ioctl(struct net_device *dev, struct ifreq *ifr, int cmd)
{
	struct mtk_mac *mac = netdev_priv(dev);

	switch (cmd) {
	case SIOCGMIIPHY:
	case SIOCGMIIREG:
	case SIOCSMIIREG:
		return phylink_mii_ioctl(mac->phylink, ifr, cmd);
	default:
		/* default invoke the mtk_eth_dbg handler */
		return mtk_do_priv_ioctl(dev, ifr, cmd);
		break;
	}

	return -EOPNOTSUPP;
}

int mtk_phy_config(struct mtk_eth *eth, int enable)
{
	struct device_node *mii_np = NULL;
	struct device_node *child = NULL;
	int addr = 0;
	u32 val = 0;

	mii_np = of_get_child_by_name(eth->dev->of_node, "mdio-bus");
	if (!mii_np) {
		dev_err(eth->dev, "no %s child node found", "mdio-bus");
		return -ENODEV;
	}

	if (!of_device_is_available(mii_np)) {
		dev_err(eth->dev, "device is not available\n");
		return -ENODEV;
	}

	for_each_available_child_of_node(mii_np, child) {
		addr = of_mdio_parse_addr(&eth->mii_bus->dev, child);
		if (addr < 0)
			continue;
		pr_info("%s %d addr:%d name:%s\n",
			__func__, __LINE__, addr, child->name);
		val = _mtk_mdio_read(eth, addr, mdiobus_c45_addr(0x1e, 0));
		if (enable)
			val &= ~BMCR_PDOWN;
		else
			val |= BMCR_PDOWN;
		_mtk_mdio_write(eth, addr, mdiobus_c45_addr(0x1e, 0), val);
	}

	return 0;
}

static void mtk_pending_work(struct work_struct *work)
{
	struct mtk_eth *eth = container_of(work, struct mtk_eth, pending_work);
	int err, i = 0;
	unsigned long restart = 0;
	u32 val = 0;

	atomic_inc(&reset_lock);
	val = mtk_r32(eth, MTK_FE_INT_STATUS);
	if (!mtk_check_reset_event(eth, val)) {
		atomic_dec(&reset_lock);
		pr_info("[%s] No need to do FE reset !\n", __func__);
		return;
	}

	rtnl_lock();

	while (test_and_set_bit_lock(MTK_RESETTING, &eth->state))
		cpu_relax();

	mtk_phy_config(eth, 0);

	/* Adjust PPE configurations to prepare for reset */
	mtk_prepare_reset_ppe(eth, 0);
	if (MTK_HAS_CAPS(eth->soc->caps, MTK_RSTCTRL_PPE1))
		mtk_prepare_reset_ppe(eth, 1);

	/* Adjust FE configurations to prepare for reset */
	mtk_prepare_reset_fe(eth);

	/* Trigger Wifi SER reset */
	for (i = 0; i < MTK_MAC_COUNT; i++) {
		if (!eth->netdev[i])
			continue;
		if (mtk_reset_flag == MTK_FE_STOP_TRAFFIC) {
			pr_info("send MTK_FE_STOP_TRAFFIC event\n");
			call_netdevice_notifiers(MTK_FE_STOP_TRAFFIC,
				eth->netdev[i]);
		} else {
			pr_info("send MTK_FE_START_RESET event\n");
			call_netdevice_notifiers(MTK_FE_START_RESET,
				eth->netdev[i]);
		}
		rtnl_unlock();
		if (!wait_for_completion_timeout(&wait_ser_done, 3000)) {
			if (MTK_HAS_CAPS(eth->soc->caps, MTK_NETSYS_V3) &&
				(mtk_stop_fail)) {
				pr_info("send MTK_FE_START_RESET stop\n");
				rtnl_lock();
				call_netdevice_notifiers(MTK_FE_START_RESET,
					eth->netdev[i]);
				rtnl_unlock();
				if (!wait_for_completion_timeout(&wait_ser_done,
					3000))
					pr_warn("wait for MTK_FE_START_RESET\n");
				}
			pr_warn("wait for MTK_FE_START_RESET\n");
		}
		rtnl_lock();
		break;
	}

	del_timer_sync(&eth->mtk_dma_monitor_timer);
	pr_info("[%s] mtk_stop starts !\n", __func__);
	/* stop all devices to make sure that dma is properly shut down */
	for (i = 0; i < MTK_MAC_COUNT; i++) {
		if (!eth->netdev[i] || !netif_running(eth->netdev[i]))
			continue;
		mtk_stop(eth->netdev[i]);
		__set_bit(i, &restart);
	}
	pr_info("[%s] mtk_stop ends !\n", __func__);
	mdelay(15);

	if (eth->dev->pins)
		pinctrl_select_state(eth->dev->pins->p,
				     eth->dev->pins->default_state);

	pr_info("[%s] mtk_hw_init starts !\n", __func__);
	mtk_hw_init(eth, MTK_TYPE_WARM_RESET);
	pr_info("[%s] mtk_hw_init ends !\n", __func__);

	/* restart DMA and enable IRQs */
	for (i = 0; i < MTK_MAC_COUNT; i++) {
		if (!test_bit(i, &restart) || !eth->netdev[i])
			continue;
		err = mtk_open(eth->netdev[i]);
		if (err) {
			netif_alert(eth, ifup, eth->netdev[i],
			      "Driver up/down cycle failed, closing device.\n");
			dev_close(eth->netdev[i]);
		}
	}

	for (i = 0; i < MTK_MAC_COUNT; i++) {
		if (!eth->netdev[i])
			continue;
		if (mtk_reset_flag == MTK_FE_STOP_TRAFFIC) {
			pr_info("send MTK_FE_START_TRAFFIC event\n");
			call_netdevice_notifiers(MTK_FE_START_TRAFFIC,
				eth->netdev[i]);
		} else {
			pr_info("send MTK_FE_RESET_DONE event\n");
			call_netdevice_notifiers(MTK_FE_RESET_DONE,
				eth->netdev[i]);
		}
		call_netdevice_notifiers(MTK_FE_RESET_NAT_DONE,
			eth->netdev[i]);
		break;
	}

	atomic_dec(&reset_lock);

	timer_setup(&eth->mtk_dma_monitor_timer, mtk_dma_monitor, 0);
	eth->mtk_dma_monitor_timer.expires = jiffies;
	add_timer(&eth->mtk_dma_monitor_timer);

	mtk_phy_config(eth, 1);
	mtk_reset_flag = 0;
	clear_bit_unlock(MTK_RESETTING, &eth->state);

	rtnl_unlock();
}

static int mtk_free_dev(struct mtk_eth *eth)
{
	int i;

	for (i = 0; i < MTK_MAC_COUNT; i++) {
		if (!eth->netdev[i])
			continue;
		free_netdev(eth->netdev[i]);
	}

	return 0;
}

static int mtk_unreg_dev(struct mtk_eth *eth)
{
	int i;

	for (i = 0; i < MTK_MAC_COUNT; i++) {
		struct mtk_mac *mac;
		if (!eth->netdev[i])
			continue;
		mac = netdev_priv(eth->netdev[i]);
		if (MTK_HAS_CAPS(eth->soc->caps, MTK_QDMA))
			unregister_netdevice_notifier(&mac->device_notifier);
		unregister_netdev(eth->netdev[i]);
	}

	return 0;
}

static int mtk_cleanup(struct mtk_eth *eth)
{
	mtk_unreg_dev(eth);
	mtk_free_dev(eth);
	cancel_work_sync(&eth->pending_work);

	return 0;
}

static int mtk_get_link_ksettings(struct net_device *ndev,
				  struct ethtool_link_ksettings *cmd)
{
	struct mtk_mac *mac = netdev_priv(ndev);

	if (unlikely(test_bit(MTK_RESETTING, &mac->hw->state)))
		return -EBUSY;

	return phylink_ethtool_ksettings_get(mac->phylink, cmd);
}

static int mtk_set_link_ksettings(struct net_device *ndev,
				  const struct ethtool_link_ksettings *cmd)
{
	struct mtk_mac *mac = netdev_priv(ndev);

	if (unlikely(test_bit(MTK_RESETTING, &mac->hw->state)))
		return -EBUSY;

	return phylink_ethtool_ksettings_set(mac->phylink, cmd);
}

static void mtk_get_drvinfo(struct net_device *dev,
			    struct ethtool_drvinfo *info)
{
	struct mtk_mac *mac = netdev_priv(dev);

	strlcpy(info->driver, mac->hw->dev->driver->name, sizeof(info->driver));
	strlcpy(info->bus_info, dev_name(mac->hw->dev), sizeof(info->bus_info));
	info->n_stats = ARRAY_SIZE(mtk_ethtool_stats);
}

static u32 mtk_get_msglevel(struct net_device *dev)
{
	struct mtk_mac *mac = netdev_priv(dev);

	return mac->hw->msg_enable;
}

static void mtk_set_msglevel(struct net_device *dev, u32 value)
{
	struct mtk_mac *mac = netdev_priv(dev);

	mac->hw->msg_enable = value;
}

static int mtk_nway_reset(struct net_device *dev)
{
	struct mtk_mac *mac = netdev_priv(dev);

	if (unlikely(test_bit(MTK_RESETTING, &mac->hw->state)))
		return -EBUSY;

	if (!mac->phylink)
		return -ENOTSUPP;

	return phylink_ethtool_nway_reset(mac->phylink);
}

static void mtk_get_strings(struct net_device *dev, u32 stringset, u8 *data)
{
	int i;

	switch (stringset) {
	case ETH_SS_STATS:
		for (i = 0; i < ARRAY_SIZE(mtk_ethtool_stats); i++) {
			memcpy(data, mtk_ethtool_stats[i].str, ETH_GSTRING_LEN);
			data += ETH_GSTRING_LEN;
		}
		break;
	}
}

static int mtk_get_sset_count(struct net_device *dev, int sset)
{
	switch (sset) {
	case ETH_SS_STATS:
		return ARRAY_SIZE(mtk_ethtool_stats);
	default:
		return -EOPNOTSUPP;
	}
}

static void mtk_get_ethtool_stats(struct net_device *dev,
				  struct ethtool_stats *stats, u64 *data)
{
	struct mtk_mac *mac = netdev_priv(dev);
	struct mtk_hw_stats *hwstats = mac->hw_stats;
	u64 *data_src, *data_dst;
	unsigned int start;
	int i;

	if (unlikely(test_bit(MTK_RESETTING, &mac->hw->state)))
		return;

	if (netif_running(dev) && netif_device_present(dev)) {
		if (spin_trylock_bh(&hwstats->stats_lock)) {
			mtk_stats_update_mac(mac);
			spin_unlock_bh(&hwstats->stats_lock);
		}
	}

	data_src = (u64 *)hwstats;

	do {
		data_dst = data;
		start = u64_stats_fetch_begin_irq(&hwstats->syncp);

		for (i = 0; i < ARRAY_SIZE(mtk_ethtool_stats); i++)
			*data_dst++ = *(data_src + mtk_ethtool_stats[i].offset);
	} while (u64_stats_fetch_retry_irq(&hwstats->syncp, start));
}

static int mtk_get_rxnfc(struct net_device *dev, struct ethtool_rxnfc *cmd,
			 u32 *rule_locs)
{
	struct mtk_mac *mac = netdev_priv(dev);
	struct mtk_eth *eth = mac->hw;
	int ret = -EOPNOTSUPP;

	switch (cmd->cmd) {
	case ETHTOOL_GRXRINGS:
		if (dev->hw_features & NETIF_F_LRO) {
			cmd->data = MTK_MAX_RX_RING_NUM;
			ret = 0;
		} else if (MTK_HAS_CAPS(eth->soc->caps, MTK_RSS)) {
			cmd->data = eth->soc->rss_num;
			ret = 0;
		}
		break;
	case ETHTOOL_GRXCLSRLCNT:
		if (dev->hw_features & NETIF_F_LRO) {
			cmd->rule_cnt = mac->hwlro_ip_cnt;
			ret = 0;
		}
		break;
	case ETHTOOL_GRXCLSRULE:
		if (dev->hw_features & NETIF_F_LRO)
			ret = mtk_hwlro_get_fdir_entry(dev, cmd);
		break;
	case ETHTOOL_GRXCLSRLALL:
		if (dev->hw_features & NETIF_F_LRO)
			ret = mtk_hwlro_get_fdir_all(dev, cmd,
						     rule_locs);
		break;
	default:
		break;
	}

	return ret;
}

static int mtk_set_rxnfc(struct net_device *dev, struct ethtool_rxnfc *cmd)
{
	int ret = -EOPNOTSUPP;

	switch (cmd->cmd) {
	case ETHTOOL_SRXCLSRLINS:
		if (dev->hw_features & NETIF_F_LRO)
			ret = mtk_hwlro_add_ipaddr(dev, cmd);
		break;
	case ETHTOOL_SRXCLSRLDEL:
		if (dev->hw_features & NETIF_F_LRO)
			ret = mtk_hwlro_del_ipaddr(dev, cmd);
		break;
	default:
		break;
	}

	return ret;
}

static u32 mtk_get_rxfh_key_size(struct net_device *dev)
{
	return MTK_RSS_HASH_KEYSIZE;
}

static u32 mtk_get_rxfh_indir_size(struct net_device *dev)
{
	return MTK_RSS_MAX_INDIRECTION_TABLE;
}

static int mtk_get_rxfh(struct net_device *dev, u32 *indir, u8 *key,
			u8 *hfunc)
{
	struct mtk_mac *mac = netdev_priv(dev);
	struct mtk_eth *eth = mac->hw;
	struct mtk_rss_params *rss_params = &eth->rss_params;
	int i;

	if (hfunc)
		*hfunc = ETH_RSS_HASH_TOP;	/* Toeplitz */

	if (key) {
		memcpy(key, rss_params->hash_key,
		       sizeof(rss_params->hash_key));
	}

	if (indir) {
		for (i = 0; i < MTK_RSS_MAX_INDIRECTION_TABLE; i++)
			indir[i] = rss_params->indirection_table[i];
	}

	return 0;
}

static int mtk_set_rxfh(struct net_device *dev, const u32 *indir,
			const u8 *key, const u8 hfunc)
{
	struct mtk_mac *mac = netdev_priv(dev);
	struct mtk_eth *eth = mac->hw;
	struct mtk_rss_params *rss_params = &eth->rss_params;
	int i;

	if (hfunc != ETH_RSS_HASH_NO_CHANGE &&
	    hfunc != ETH_RSS_HASH_TOP)
		return -EOPNOTSUPP;

	if (key) {
		memcpy(rss_params->hash_key, key,
		       sizeof(rss_params->hash_key));

		for (i = 0; i < MTK_RSS_HASH_KEYSIZE / sizeof(u32); i++)
			mtk_w32(eth, rss_params->hash_key[i],
				MTK_RSS_HASH_KEY_DW(i));
	}

	if (indir) {
		for (i = 0; i < MTK_RSS_MAX_INDIRECTION_TABLE; i++)
			rss_params->indirection_table[i] = indir[i];

		for (i = 0; i < MTK_RSS_MAX_INDIRECTION_TABLE / 16; i++)
			mtk_w32(eth, mtk_rss_indr_table(rss_params, i),
				MTK_RSS_INDR_TABLE_DW(i));
	}

	return 0;
}

static void mtk_get_pauseparam(struct net_device *dev, struct ethtool_pauseparam *pause)
{
	struct mtk_mac *mac = netdev_priv(dev);
	struct mtk_eth *eth = mac->hw;
	u32 val;

	pause->autoneg = 0;

	if (mac->type == MTK_GDM_TYPE) {
		val = mtk_r32(eth, MTK_MAC_MCR(mac->id));

		pause->rx_pause = !!(val & MAC_MCR_FORCE_RX_FC);
		pause->tx_pause = !!(val & MAC_MCR_FORCE_TX_FC);
	} else if (mac->type == MTK_XGDM_TYPE) {
		val = mtk_r32(eth, MTK_XMAC_MCR(mac->id));

		pause->rx_pause = !!(val & XMAC_MCR_FORCE_RX_FC);
		pause->tx_pause = !!(val & XMAC_MCR_FORCE_TX_FC);
	}
}

static int mtk_set_pauseparam(struct net_device *dev, struct ethtool_pauseparam *pause)
{
	struct mtk_mac *mac = netdev_priv(dev);

	return phylink_ethtool_set_pauseparam(mac->phylink, pause);
}

static int mtk_get_eee(struct net_device *dev, struct ethtool_eee *eee)
{
	struct mtk_mac *mac = netdev_priv(dev);
	struct mtk_eth *eth = mac->hw;
	u32 val;

	if (mac->type == MTK_GDM_TYPE) {
		val = mtk_r32(eth, MTK_MAC_EEE(mac->id));

		eee->tx_lpi_enabled = mac->tx_lpi_enabled;
		eee->tx_lpi_timer = FIELD_GET(MAC_EEE_LPI_TXIDLE_THD, val);
	}

	return phylink_ethtool_get_eee(mac->phylink, eee);
}

static int mtk_set_eee(struct net_device *dev, struct ethtool_eee *eee)
{
	struct mtk_mac *mac = netdev_priv(dev);

	if (mac->type == MTK_GDM_TYPE) {
		if (eee->tx_lpi_enabled && eee->tx_lpi_timer > 255)
			return -EINVAL;

		mac->tx_lpi_timer = eee->tx_lpi_timer;

		mtk_setup_eee(mac, eee->eee_enabled && eee->tx_lpi_timer);
	}

	return phylink_ethtool_set_eee(mac->phylink, eee);
}

static u16 mtk_select_queue(struct net_device *dev, struct sk_buff *skb,
			    struct net_device *sb_dev)
{
	struct mtk_mac *mac = netdev_priv(dev);
	struct mtk_eth *eth = mac->hw;
	unsigned int queue = 0;

	if (skb->mark > 0 && skb->mark < MTK_QDMA_TX_NUM)
		return skb->mark;

	if (eth->pppq_toggle) {
		switch (mac->id) {
		case MTK_GMAC1_ID:
			queue = skb_get_queue_mapping(skb);
			break;
		case MTK_GMAC2_ID:
			queue = MTK_QDMA_GMAC2_QID;
			break;
		case MTK_GMAC3_ID:
			queue = MTK_QDMA_GMAC3_QID;
			break;
		default:
			pr_info("%s mac id invalid", __func__);
			break;
		}
	} else
		queue = mac->id ? MTK_QDMA_GMAC2_QID : 0;

	if (queue >= MTK_QDMA_TX_NUM)
		queue = 0;

	return queue;
}

static const struct ethtool_ops mtk_ethtool_ops = {
	.get_link_ksettings	= mtk_get_link_ksettings,
	.set_link_ksettings	= mtk_set_link_ksettings,
	.get_drvinfo		= mtk_get_drvinfo,
	.get_msglevel		= mtk_get_msglevel,
	.set_msglevel		= mtk_set_msglevel,
	.nway_reset		= mtk_nway_reset,
	.get_link		= ethtool_op_get_link,
	.get_strings		= mtk_get_strings,
	.get_sset_count		= mtk_get_sset_count,
	.get_ethtool_stats	= mtk_get_ethtool_stats,
	.get_rxnfc		= mtk_get_rxnfc,
	.set_rxnfc              = mtk_set_rxnfc,
	.get_rxfh_key_size	= mtk_get_rxfh_key_size,
	.get_rxfh_indir_size	= mtk_get_rxfh_indir_size,
	.get_rxfh		= mtk_get_rxfh,
	.set_rxfh		= mtk_set_rxfh,
	.get_pauseparam		= mtk_get_pauseparam,
	.set_pauseparam		= mtk_set_pauseparam,
	.get_eee		= mtk_get_eee,
	.set_eee		= mtk_set_eee,
};

static const struct net_device_ops mtk_netdev_ops = {
	.ndo_init		= mtk_init,
	.ndo_uninit		= mtk_uninit,
	.ndo_open		= mtk_open,
	.ndo_stop		= mtk_stop,
	.ndo_start_xmit		= mtk_start_xmit,
	.ndo_select_queue       = mtk_select_queue,
	.ndo_set_mac_address	= mtk_set_mac_address,
	.ndo_validate_addr	= eth_validate_addr,
	.ndo_do_ioctl		= mtk_do_ioctl,
	.ndo_tx_timeout		= mtk_tx_timeout,
	.ndo_get_stats64        = mtk_get_stats64,
	.ndo_fix_features	= mtk_fix_features,
	.ndo_set_features	= mtk_set_features,
#ifdef CONFIG_NET_POLL_CONTROLLER
	.ndo_poll_controller	= mtk_poll_controller,
#endif
};

static void mux_poll(struct work_struct *work)
{
	struct mtk_mux *mux = container_of(work, struct mtk_mux, poll.work);
	struct mtk_mac *mac = mux->mac;
	struct mtk_eth *eth = mac->hw;
	struct net_device *dev = eth->netdev[mac->id];
	unsigned int channel;

	if (IS_ERR(mux->gpio[0]) || IS_ERR(mux->gpio[1]))
		goto exit;

	channel = gpiod_get_value_cansleep(mux->gpio[0]);
	if (mux->channel == channel || !netif_running(dev))
		goto exit;

	rtnl_lock();

	mtk_stop(dev);

	if (channel == 0 || channel == 1) {
		mac->of_node = mux->data[channel]->of_node;
		mac->phylink = mux->data[channel]->phylink;
	};

	dev_info(eth->dev, "ethernet mux: switch to channel%d\n", channel);

	gpiod_set_value_cansleep(mux->gpio[1], channel);

	mtk_open(dev);

	rtnl_unlock();

	mux->channel = channel;

exit:
	mod_delayed_work(system_wq, &mux->poll, msecs_to_jiffies(100));
}

static int mtk_add_mux_channel(struct mtk_mux *mux, struct device_node *np)
{
	const __be32 *_id = of_get_property(np, "reg", NULL);
	struct mtk_mac *mac = mux->mac;
	struct mtk_eth *eth = mac->hw;
	struct mtk_mux_data *data;
	struct phylink *phylink;
	int phy_mode, id;

	if (!_id) {
		dev_err(eth->dev, "missing mux channel id\n");
		return -EINVAL;
	}

	id = be32_to_cpup(_id);
	if (id < 0 || id > 1) {
		dev_err(eth->dev, "%d is not a valid mux channel id\n", id);
		return -EINVAL;
	}

	data = kmalloc(sizeof(*data), GFP_KERNEL);
	if (unlikely(!data)) {
		dev_err(eth->dev, "failed to create mux data structure\n");
		return -ENOMEM;
	}

	mux->data[id] = data;

	/* phylink create */
	phy_mode = of_get_phy_mode(np);
	if (phy_mode < 0) {
		dev_err(eth->dev, "incorrect phy-mode\n");
		return -EINVAL;
	}

	phylink = phylink_create(&mux->mac->phylink_config,
				 of_fwnode_handle(np),
				 phy_mode, &mtk_phylink_ops);
	if (IS_ERR(phylink)) {
		dev_err(eth->dev, "failed to create phylink structure\n");
		return PTR_ERR(phylink);
	}

	data->of_node = np;
	data->phylink = phylink;

	return 0;
}

static int mtk_add_mux(struct mtk_eth *eth, struct device_node *np)
{
	const __be32 *_id = of_get_property(np, "reg", NULL);
	struct device_node *child;
	struct mtk_mux *mux;
	int id, err;

	if (!_id) {
		dev_err(eth->dev, "missing attach mac id\n");
		return -EINVAL;
	}

	id = be32_to_cpup(_id);
	if (id < 0 || id >= MTK_MAX_DEVS) {
		dev_err(eth->dev, "%d is not a valid attach mac id\n", id);
		return -EINVAL;
	}

	mux = kmalloc(sizeof(struct mtk_mux), GFP_KERNEL);
	if (unlikely(!mux)) {
		dev_err(eth->dev, "failed to create mux structure\n");
		return -ENOMEM;
	}

	eth->mux[id] = mux;

	mux->mac = eth->mac[id];
	mux->channel = 0;

	mux->gpio[0] = fwnode_get_named_gpiod(of_fwnode_handle(np),
					      "mod-def0-gpios", 0,
					      GPIOD_IN, "?");
	if (IS_ERR(mux->gpio[0]))
		dev_err(eth->dev, "failed to requset gpio for mod-def0-gpios\n");

	mux->gpio[1] = fwnode_get_named_gpiod(of_fwnode_handle(np),
					      "chan-sel-gpios", 0,
					      GPIOD_OUT_LOW, "?");
	if (IS_ERR(mux->gpio[1]))
		dev_err(eth->dev, "failed to requset gpio for chan-sel-gpios\n");

	for_each_child_of_node(np, child) {
		err = mtk_add_mux_channel(mux, child);
		if (err) {
			dev_err(eth->dev, "failed to add mtk_mux\n");
			of_node_put(child);
			return -ECHILD;
		}
		of_node_put(child);
	}

	INIT_DELAYED_WORK(&mux->poll, mux_poll);
	mod_delayed_work(system_wq, &mux->poll, msecs_to_jiffies(3000));

	return 0;
}

static int mtk_add_mac(struct mtk_eth *eth, struct device_node *np)
{
	const __be32 *_id = of_get_property(np, "reg", NULL);
	const char *label;
	struct phylink *phylink;
	int mac_type, phy_mode, id, err;
	struct mtk_mac *mac;
	struct mtk_phylink_priv *phylink_priv;
	struct fwnode_handle *fixed_node;
	struct gpio_desc *desc;
	int txqs = 1;

	if (!_id) {
		dev_err(eth->dev, "missing mac id\n");
		return -EINVAL;
	}

	id = be32_to_cpup(_id);
	if (id < 0 || id >= MTK_MAC_COUNT) {
		dev_err(eth->dev, "%d is not a valid mac id\n", id);
		return -EINVAL;
	}

	if (eth->netdev[id]) {
		dev_err(eth->dev, "duplicate mac id found: %d\n", id);
		return -EINVAL;
	}

	if (MTK_HAS_CAPS(eth->soc->caps, MTK_QDMA))
		txqs = MTK_QDMA_TX_NUM;

	eth->netdev[id] = alloc_etherdev_mqs(sizeof(*mac), txqs, 1);
	if (!eth->netdev[id]) {
		dev_err(eth->dev, "alloc_etherdev failed\n");
		return -ENOMEM;
	}
	mac = netdev_priv(eth->netdev[id]);
	eth->mac[id] = mac;
	mac->id = id;
	mac->hw = eth;
	mac->of_node = np;

	memset(mac->hwlro_ip, 0, sizeof(mac->hwlro_ip));
	mac->hwlro_ip_cnt = 0;

	mac->hw_stats = devm_kzalloc(eth->dev,
				     sizeof(*mac->hw_stats),
				     GFP_KERNEL);
	if (!mac->hw_stats) {
		dev_err(eth->dev, "failed to allocate counter memory\n");
		err = -ENOMEM;
		goto free_netdev;
	}
	spin_lock_init(&mac->hw_stats->stats_lock);
	u64_stats_init(&mac->hw_stats->syncp);
	mac->hw_stats->reg_offset = id * MTK_STAT_OFFSET;

	/* phylink create */
	phy_mode = of_get_phy_mode(np);
	if (phy_mode < 0) {
		dev_err(eth->dev, "incorrect phy-mode\n");
		err = -EINVAL;
		goto free_netdev;
	}

	/* mac config is not set */
	mac->interface = PHY_INTERFACE_MODE_NA;
	mac->mode = MLO_AN_PHY;
	mac->speed = SPEED_UNKNOWN;

	mac->tx_lpi_timer = 1;

	mac->phylink_config.dev = &eth->netdev[id]->dev;
	mac->phylink_config.type = PHYLINK_NETDEV;

	mac->type = 0;
	if (!of_property_read_string(np, "mac-type", &label)) {
		for (mac_type = 0; mac_type < MTK_GDM_TYPE_MAX; mac_type++) {
			if (!strcasecmp(label, gdm_type(mac_type)))
				break;
		}

		switch (mac_type) {
		case 0:
			mac->type = MTK_GDM_TYPE;
			break;
		case 1:
			mac->type = MTK_XGDM_TYPE;
			break;
		default:
			dev_warn(eth->dev, "incorrect mac-type\n");
			break;
		};
	}

	phylink = phylink_create(&mac->phylink_config,
				 of_fwnode_handle(mac->of_node),
				 phy_mode, &mtk_phylink_ops);
	if (IS_ERR(phylink)) {
		err = PTR_ERR(phylink);
		goto free_netdev;
	}

	mac->phylink = phylink;

	fixed_node = fwnode_get_named_child_node(of_fwnode_handle(mac->of_node),
						 "fixed-link");
	if (fixed_node) {
		desc = fwnode_get_named_gpiod(fixed_node, "link-gpio",
					      0, GPIOD_IN, "?");
		if (!IS_ERR(desc)) {
			struct device_node *phy_np;
			const char *label;
			int irq, phyaddr;

			phylink_priv = &mac->phylink_priv;

			phylink_priv->desc = desc;
			phylink_priv->id = id;
			phylink_priv->link = -1;

			irq = gpiod_to_irq(desc);
			if (irq > 0) {
				devm_request_irq(eth->dev, irq, mtk_handle_irq_fixed_link,
						 IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
					         "ethernet:fixed link", mac);
			}

			if (!of_property_read_string(to_of_node(fixed_node),
						     "label", &label)) {
				if (strlen(label) < 16) {
					strncpy(phylink_priv->label, label,
						strlen(label));
				} else
					dev_err(eth->dev, "insufficient space for label!\n");
			}

			phy_np = of_parse_phandle(to_of_node(fixed_node), "phy-handle", 0);
			if (phy_np) {
				if (!of_property_read_u32(phy_np, "reg", &phyaddr))
					phylink_priv->phyaddr = phyaddr;
			}
		}
		fwnode_handle_put(fixed_node);
	}

	SET_NETDEV_DEV(eth->netdev[id], eth->dev);
	eth->netdev[id]->watchdog_timeo = 5 * HZ;
	eth->netdev[id]->netdev_ops = &mtk_netdev_ops;
	eth->netdev[id]->base_addr = (unsigned long)eth->base;

	eth->netdev[id]->hw_features = eth->soc->hw_features;
	if (eth->hwlro)
		eth->netdev[id]->hw_features |= NETIF_F_LRO;

	eth->netdev[id]->vlan_features = eth->soc->hw_features &
		~(NETIF_F_HW_VLAN_CTAG_TX | NETIF_F_HW_VLAN_CTAG_RX);
	eth->netdev[id]->features |= eth->soc->hw_features;
	eth->netdev[id]->ethtool_ops = &mtk_ethtool_ops;

	eth->netdev[id]->irq = eth->irq_fe[0];
	eth->netdev[id]->dev.of_node = np;

	if (MTK_HAS_CAPS(eth->soc->caps, MTK_QDMA)) {
		mac->device_notifier.notifier_call = mtk_device_event;
		register_netdevice_notifier(&mac->device_notifier);
	}

	return 0;

free_netdev:
	free_netdev(eth->netdev[id]);
	return err;
}

void mtk_eth_set_dma_device(struct mtk_eth *eth, struct device *dma_dev)
{
	struct net_device *dev, *tmp;
	LIST_HEAD(dev_list);
	int i;

	rtnl_lock();

	for (i = 0; i < MTK_MAC_COUNT; i++) {
		dev = eth->netdev[i];

		if (!dev || !(dev->flags & IFF_UP))
			continue;

		list_add_tail(&dev->close_list, &dev_list);
	}

	dev_close_many(&dev_list, false);

	eth->dma_dev = dma_dev;

	list_for_each_entry_safe(dev, tmp, &dev_list, close_list) {
		list_del_init(&dev->close_list);
		dev_open(dev, NULL);
	}

	rtnl_unlock();
}

static int mtk_probe(struct platform_device *pdev)
{
	struct device_node *mac_np, *mux_np;
	struct mtk_eth *eth;
	int err, i;

	eth = devm_kzalloc(&pdev->dev, sizeof(*eth), GFP_KERNEL);
	if (!eth)
		return -ENOMEM;

	eth->soc = of_device_get_match_data(&pdev->dev);
	if (!eth->soc)
		return -EINVAL;

	eth->dev = &pdev->dev;
	eth->dma_dev = &pdev->dev;
	eth->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(eth->base))
		return PTR_ERR(eth->base);

	if (MTK_HAS_CAPS(eth->soc->caps, MTK_NETSYS_V3)) {
		eth->sram_base = (void __force *)devm_platform_ioremap_resource(pdev, 1);
		if (IS_ERR(eth->sram_base))
			return PTR_ERR(eth->sram_base);
	} else {
		eth->sram_base = (void __force *)eth->base + MTK_ETH_SRAM_OFFSET;
	}

	if(eth->soc->has_sram) {
		struct resource *res;
		res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
		if (unlikely(!res))
			return -EINVAL;
		eth->phy_scratch_ring = res->start + MTK_ETH_SRAM_OFFSET;
	}

	mtk_get_hwver(eth);

	if (MTK_HAS_CAPS(eth->soc->caps, MTK_SOC_MT7628))
		eth->ip_align = NET_IP_ALIGN;

	if (MTK_HAS_CAPS(eth->soc->caps, MTK_8GB_ADDRESSING)) {
		err = dma_set_mask(&pdev->dev, DMA_BIT_MASK(36));
		if (!err) {
			err = dma_set_coherent_mask(&pdev->dev,
						    DMA_BIT_MASK(32));
			if (err) {
				dev_err(&pdev->dev, "Wrong DMA config\n");
				return -EINVAL;
			}
		}
	}

	spin_lock_init(&eth->page_lock);
	spin_lock_init(&eth->tx_irq_lock);
	spin_lock_init(&eth->rx_irq_lock);
	spin_lock_init(&eth->txrx_irq_lock);
	spin_lock_init(&eth->syscfg0_lock);

	if (!MTK_HAS_CAPS(eth->soc->caps, MTK_SOC_MT7628)) {
		eth->ethsys = syscon_regmap_lookup_by_phandle(pdev->dev.of_node,
							      "mediatek,ethsys");
		if (IS_ERR(eth->ethsys)) {
			dev_err(&pdev->dev, "no ethsys regmap found\n");
			return PTR_ERR(eth->ethsys);
		}
	}

	if (MTK_HAS_CAPS(eth->soc->caps, MTK_INFRA)) {
		eth->infra = syscon_regmap_lookup_by_phandle(pdev->dev.of_node,
							     "mediatek,infracfg");
		if (IS_ERR(eth->infra)) {
			dev_err(&pdev->dev, "no infracfg regmap found\n");
			return PTR_ERR(eth->infra);
		}
	}

	if (of_dma_is_coherent(pdev->dev.of_node)) {
		struct regmap *cci;

		cci = syscon_regmap_lookup_by_phandle(pdev->dev.of_node,
						      "cci-control-port");
		/* enable CPU/bus coherency */
		if (!IS_ERR(cci))
			regmap_write(cci, 0, 3);
	}

	if (MTK_HAS_CAPS(eth->soc->caps, MTK_SGMII)) {
		eth->sgmii = devm_kzalloc(eth->dev, sizeof(*eth->sgmii),
					  GFP_KERNEL);
		if (!eth->sgmii)
			return -ENOMEM;

		err = mtk_sgmii_init(eth, pdev->dev.of_node,
				     eth->soc->ana_rgc3);
		if (err)
			return err;
	}

	if (MTK_HAS_CAPS(eth->soc->caps, MTK_USXGMII)) {
		eth->usxgmii = devm_kzalloc(eth->dev, sizeof(*eth->usxgmii),
					    GFP_KERNEL);
		if (!eth->usxgmii)
			return -ENOMEM;

		err = mtk_usxgmii_init(eth, pdev->dev.of_node);
		if (err)
			return err;

		err = mtk_toprgu_init(eth, pdev->dev.of_node);
		if (err)
			return err;
	}

	if (eth->soc->required_pctl) {
		eth->pctl = syscon_regmap_lookup_by_phandle(pdev->dev.of_node,
							    "mediatek,pctl");
		if (IS_ERR(eth->pctl)) {
			dev_err(&pdev->dev, "no pctl regmap found\n");
			return PTR_ERR(eth->pctl);
		}
	}

	if (MTK_HAS_CAPS(eth->soc->caps, MTK_PDMA_INT)) {
		for (i = 0; i < MTK_PDMA_IRQ_NUM; i++)
			eth->irq_pdma[i] = platform_get_irq(pdev, i);
	}

	for (i = 0; i < MTK_FE_IRQ_NUM; i++) {
		if (MTK_HAS_CAPS(eth->soc->caps, MTK_SHARED_INT) && i > 0)
			eth->irq_fe[i] = eth->irq_fe[0];
		else if (MTK_HAS_CAPS(eth->soc->caps, MTK_PDMA_INT))
			eth->irq_fe[i] =
				platform_get_irq(pdev, i + MTK_PDMA_IRQ_NUM);
		else
			eth->irq_fe[i] = platform_get_irq(pdev, i);

		if (eth->irq_fe[i] < 0) {
			dev_err(&pdev->dev, "no IRQ%d resource found\n", i);
			return -ENXIO;
		}
	}

	for (i = 0; i < ARRAY_SIZE(eth->clks); i++) {
		eth->clks[i] = devm_clk_get(eth->dev,
					    mtk_clks_source_name[i]);
		if (IS_ERR(eth->clks[i])) {
			if (PTR_ERR(eth->clks[i]) == -EPROBE_DEFER)
				return -EPROBE_DEFER;
			if (eth->soc->required_clks & BIT(i)) {
				dev_err(&pdev->dev, "clock %s not found\n",
					mtk_clks_source_name[i]);
				return -EINVAL;
			}
			eth->clks[i] = NULL;
		}
	}

	eth->msg_enable = netif_msg_init(mtk_msg_level, MTK_DEFAULT_MSG_ENABLE);
	INIT_WORK(&eth->pending_work, mtk_pending_work);

	err = mtk_hw_init(eth, MTK_TYPE_COLD_RESET);
	if (err)
		return err;

	eth->hwlro = MTK_HAS_CAPS(eth->soc->caps, MTK_HWLRO);

	for_each_child_of_node(pdev->dev.of_node, mac_np) {
		if (!of_device_is_compatible(mac_np,
					     "mediatek,eth-mac"))
			continue;

		if (!of_device_is_available(mac_np))
			continue;

		err = mtk_add_mac(eth, mac_np);
		if (err) {
			of_node_put(mac_np);
			goto err_deinit_hw;
		}
	}

	mux_np = of_get_child_by_name(eth->dev->of_node, "mux-bus");
	if (mux_np) {
		struct device_node *child;

		for_each_available_child_of_node(mux_np, child) {
			if (!of_device_is_compatible(child,
						     "mediatek,eth-mux"))
				continue;

			if (!of_device_is_available(child))
				continue;

			err = mtk_add_mux(eth, child);
			if (err)
				dev_err(&pdev->dev, "failed to add mux\n");

			of_node_put(mux_np);
		};
	}

	err = mtk_napi_init(eth);
	if (err)
		goto err_free_dev;

	if (MTK_HAS_CAPS(eth->soc->caps, MTK_SHARED_INT)) {
		err = devm_request_irq(eth->dev, eth->irq_fe[0],
				       mtk_handle_irq, 0,
				       dev_name(eth->dev), eth);
	} else {
		err = devm_request_irq(eth->dev, eth->irq_fe[1],
				       mtk_handle_irq_tx, 0,
				       dev_name(eth->dev), eth);
		if (err)
			goto err_free_dev;

		if (MTK_HAS_CAPS(eth->soc->caps, MTK_PDMA_INT)) {
			err = devm_request_irq(eth->dev, eth->irq_fe[2],
					       mtk_handle_fe_irq, 0,
					       dev_name(eth->dev), eth);
			if (err)
				goto err_free_dev;

			err = devm_request_irq(eth->dev, eth->irq_pdma[0],
					       mtk_handle_irq_rx, IRQF_SHARED,
					       dev_name(eth->dev), &eth->rx_napi[0]);
			if (err)
				goto err_free_dev;

			if (MTK_HAS_CAPS(eth->soc->caps, MTK_RSS)) {
				for (i = 0; i < MTK_RX_RSS_NUM; i++) {
					err = devm_request_irq(eth->dev,
							       eth->irq_pdma[MTK_RSS_RING(i)],
							       mtk_handle_irq_rx, IRQF_SHARED,
							       dev_name(eth->dev),
							       &eth->rx_napi[MTK_RSS_RING(i)]);
					if (err)
						goto err_free_dev;
				}
			}

			if (MTK_HAS_CAPS(eth->soc->caps, MTK_HWLRO)) {
				i = (MTK_HAS_CAPS(eth->soc->caps, MTK_NETSYS_RX_V2) ||
				     MTK_HAS_CAPS(eth->soc->caps, MTK_NETSYS_V3)) ? 0 : 1;
				for (; i < MTK_HW_LRO_RING_NUM; i++) {
					err = devm_request_irq(eth->dev,
							       eth->irq_pdma[i],
							       mtk_handle_irq_rx, IRQF_SHARED,
							       dev_name(eth->dev),
							       &eth->rx_napi[MTK_HW_LRO_RING(i)]);
					if (err)
						goto err_free_dev;
				}
			}
		} else {
			err = devm_request_irq(eth->dev, eth->irq_fe[2],
					       mtk_handle_irq_rx, 0,
					       dev_name(eth->dev), &eth->rx_napi[0]);
			if (err)
				goto err_free_dev;

			if (MTK_FE_IRQ_NUM > 3) {
				err = devm_request_irq(eth->dev, eth->irq_fe[3],
						       mtk_handle_fe_irq, 0,
						       dev_name(eth->dev), eth);
				if (err)
					goto err_free_dev;
			}
		}
	}

	if (err)
		goto err_free_dev;

	/* No MT7628/88 support yet */
	if (!MTK_HAS_CAPS(eth->soc->caps, MTK_SOC_MT7628)) {
		err = mtk_mdio_init(eth);
		if (err)
			goto err_free_dev;
	}

	for (i = 0; i < MTK_MAX_DEVS; i++) {
		if (!eth->netdev[i])
			continue;

		err = register_netdev(eth->netdev[i]);
		if (err) {
			dev_err(eth->dev, "error bringing up device\n");
			goto err_deinit_mdio;
		} else
			netif_info(eth, probe, eth->netdev[i],
				   "mediatek frame engine at 0x%08lx, irq %d\n",
				   eth->netdev[i]->base_addr, eth->irq_fe[0]);
	}

	/* we run 2 devices on the same DMA ring so we need a dummy device
	 * for NAPI to work
	 */
	init_dummy_netdev(&eth->dummy_dev);
	netif_napi_add(&eth->dummy_dev, &eth->tx_napi, mtk_napi_tx,
		       MTK_NAPI_WEIGHT);
	netif_napi_add(&eth->dummy_dev, &eth->rx_napi[0].napi, mtk_napi_rx,
		       MTK_NAPI_WEIGHT);

	if (MTK_HAS_CAPS(eth->soc->caps, MTK_RSS)) {
		for (i = 0; i < MTK_RX_RSS_NUM; i++)
			netif_napi_add(&eth->dummy_dev, &eth->rx_napi[MTK_RSS_RING(i)].napi,
				       mtk_napi_rx, MTK_NAPI_WEIGHT);
	}

	if (MTK_HAS_CAPS(eth->soc->caps, MTK_HWLRO)) {
		for (i = 0; i < MTK_HW_LRO_RING_NUM; i++) {
			netif_napi_add(&eth->dummy_dev, &eth->rx_napi[MTK_HW_LRO_RING(i)].napi,
				       mtk_napi_rx, MTK_NAPI_WEIGHT);
		}
	}

	mtketh_debugfs_init(eth);
	debug_proc_init(eth);

	platform_set_drvdata(pdev, eth);

	register_netdevice_notifier(&mtk_eth_netdevice_nb);
#if defined(CONFIG_MEDIATEK_NETSYS_V2) || defined(CONFIG_MEDIATEK_NETSYS_V3)
	timer_setup(&eth->mtk_dma_monitor_timer, mtk_dma_monitor, 0);
	eth->mtk_dma_monitor_timer.expires = jiffies;
	add_timer(&eth->mtk_dma_monitor_timer);
#endif

	return 0;

err_deinit_mdio:
	mtk_mdio_cleanup(eth);
err_free_dev:
	mtk_free_dev(eth);
err_deinit_hw:
	mtk_hw_deinit(eth);

	return err;
}

static int mtk_remove(struct platform_device *pdev)
{
	struct mtk_eth *eth = platform_get_drvdata(pdev);
	struct mtk_mac *mac;
	int i;

	/* stop all devices to make sure that dma is properly shut down */
	for (i = 0; i < MTK_MAC_COUNT; i++) {
		if (!eth->netdev[i])
			continue;
		mtk_stop(eth->netdev[i]);
		mac = netdev_priv(eth->netdev[i]);
		phylink_disconnect_phy(mac->phylink);
	}

	mtk_hw_deinit(eth);

	netif_napi_del(&eth->tx_napi);
	netif_napi_del(&eth->rx_napi[0].napi);

	if (MTK_HAS_CAPS(eth->soc->caps, MTK_RSS)) {
		for (i = 1; i < MTK_RX_NAPI_NUM; i++)
			netif_napi_del(&eth->rx_napi[i].napi);
	}

	mtk_cleanup(eth);
	mtk_mdio_cleanup(eth);
	unregister_netdevice_notifier(&mtk_eth_netdevice_nb);
	del_timer_sync(&eth->mtk_dma_monitor_timer);

	return 0;
}

static const struct mtk_soc_data mt2701_data = {
	.reg_map = &mtk_reg_map,
	.caps = MT7623_CAPS | MTK_HWLRO,
	.hw_features = MTK_HW_FEATURES,
	.required_clks = MT7623_CLKS_BITMAP,
	.required_pctl = true,
	.has_sram = false,
	.rss_num = 0,
	.txrx = {
		.txd_size = sizeof(struct mtk_tx_dma),
		.rxd_size = sizeof(struct mtk_rx_dma),
		.tx_dma_size = MTK_DMA_SIZE(2K),
		.rx_dma_size = MTK_DMA_SIZE(2K),
		.fq_dma_size = MTK_DMA_SIZE(2K),
		.rx_dma_l4_valid = RX_DMA_L4_VALID,
		.dma_max_len = MTK_TX_DMA_BUF_LEN,
		.dma_len_offset = MTK_TX_DMA_BUF_SHIFT,
	},
};

static const struct mtk_soc_data mt7621_data = {
	.reg_map = &mtk_reg_map,
	.caps = MT7621_CAPS,
	.hw_features = MTK_HW_FEATURES,
	.required_clks = MT7621_CLKS_BITMAP,
	.required_pctl = false,
	.has_sram = false,
	.rss_num = 0,
	.txrx = {
		.txd_size = sizeof(struct mtk_tx_dma),
		.rx_dma_l4_valid = RX_DMA_L4_VALID,
		.tx_dma_size = MTK_DMA_SIZE(2K),
		.rx_dma_size = MTK_DMA_SIZE(2K),
		.fq_dma_size = MTK_DMA_SIZE(2K),
		.rxd_size = sizeof(struct mtk_rx_dma),
		.dma_max_len = MTK_TX_DMA_BUF_LEN,
		.dma_len_offset = MTK_TX_DMA_BUF_SHIFT,
	},
};

static const struct mtk_soc_data mt7622_data = {
	.reg_map = &mtk_reg_map,
	.ana_rgc3 = 0x2028,
	.caps = MT7622_CAPS | MTK_HWLRO,
	.hw_features = MTK_HW_FEATURES,
	.required_clks = MT7622_CLKS_BITMAP,
	.required_pctl = false,
	.has_sram = false,
	.rss_num = 0,
	.txrx = {
		.txd_size = sizeof(struct mtk_tx_dma),
		.rxd_size = sizeof(struct mtk_rx_dma),
		.tx_dma_size = MTK_DMA_SIZE(2K),
		.rx_dma_size = MTK_DMA_SIZE(2K),
		.fq_dma_size = MTK_DMA_SIZE(2K),
		.rx_dma_l4_valid = RX_DMA_L4_VALID,
		.dma_max_len = MTK_TX_DMA_BUF_LEN,
		.dma_len_offset = MTK_TX_DMA_BUF_SHIFT,
	},
};

static const struct mtk_soc_data mt7623_data = {
	.reg_map = &mtk_reg_map,
	.caps = MT7623_CAPS | MTK_HWLRO,
	.hw_features = MTK_HW_FEATURES,
	.required_clks = MT7623_CLKS_BITMAP,
	.required_pctl = true,
	.has_sram = false,
	.rss_num = 0,
	.txrx = {
		.txd_size = sizeof(struct mtk_tx_dma),
		.rxd_size = sizeof(struct mtk_rx_dma),
		.tx_dma_size = MTK_DMA_SIZE(2K),
		.rx_dma_size = MTK_DMA_SIZE(2K),
		.fq_dma_size = MTK_DMA_SIZE(2K),
		.rx_dma_l4_valid = RX_DMA_L4_VALID,
		.dma_max_len = MTK_TX_DMA_BUF_LEN,
		.dma_len_offset = MTK_TX_DMA_BUF_SHIFT,
	},
};

static const struct mtk_soc_data mt7629_data = {
	.reg_map = &mtk_reg_map,
	.ana_rgc3 = 0x128,
	.caps = MT7629_CAPS | MTK_HWLRO,
	.hw_features = MTK_HW_FEATURES,
	.required_clks = MT7629_CLKS_BITMAP,
	.required_pctl = false,
	.has_sram = false,
	.rss_num = 0,
	.txrx = {
		.txd_size = sizeof(struct mtk_tx_dma),
		.rxd_size = sizeof(struct mtk_rx_dma),
		.tx_dma_size = MTK_DMA_SIZE(2K),
		.rx_dma_size = MTK_DMA_SIZE(2K),
		.fq_dma_size = MTK_DMA_SIZE(2K),
		.rx_dma_l4_valid = RX_DMA_L4_VALID,
		.dma_max_len = MTK_TX_DMA_BUF_LEN,
		.dma_len_offset = MTK_TX_DMA_BUF_SHIFT,
	},
};

static const struct mtk_soc_data mt7986_data = {
	.reg_map = &mt7986_reg_map,
	.ana_rgc3 = 0x128,
	.caps = MT7986_CAPS,
	.hw_features = MTK_HW_FEATURES,
	.required_clks = MT7986_CLKS_BITMAP,
	.required_pctl = false,
	.has_sram = false,
	.rss_num = 4,
	.txrx = {
		.txd_size = sizeof(struct mtk_tx_dma_v2),
		.rxd_size = sizeof(struct mtk_rx_dma),
		.tx_dma_size = MTK_DMA_SIZE(4K),
		.rx_dma_size = MTK_DMA_SIZE(1K),
		.fq_dma_size = MTK_DMA_SIZE(2K),
		.rx_dma_l4_valid = RX_DMA_L4_VALID_V2,
		.dma_max_len = MTK_TX_DMA_BUF_LEN_V2,
		.dma_len_offset = MTK_TX_DMA_BUF_SHIFT_V2,
	},
};

static const struct mtk_soc_data mt7981_data = {
	.reg_map = &mt7986_reg_map,
	.ana_rgc3 = 0x128,
	.caps = MT7981_CAPS,
	.hw_features = MTK_HW_FEATURES,
	.required_clks = MT7981_CLKS_BITMAP,
	.required_pctl = false,
	.has_sram = false,
	.rss_num = 4,
	.txrx = {
		.txd_size = sizeof(struct mtk_tx_dma_v2),
		.rxd_size = sizeof(struct mtk_rx_dma),
		.tx_dma_size = MTK_DMA_SIZE(4K),
		.rx_dma_size = MTK_DMA_SIZE(1K),
		.fq_dma_size = MTK_DMA_SIZE(2K),
		.rx_dma_l4_valid = RX_DMA_L4_VALID_V2,
		.dma_max_len = MTK_TX_DMA_BUF_LEN_V2,
		.dma_len_offset = MTK_TX_DMA_BUF_SHIFT_V2,
	},
};

static const struct mtk_soc_data mt7988_data = {
	.reg_map = &mt7988_reg_map,
	.ana_rgc3 = 0x128,
	.caps = MT7988_CAPS | MTK_HWLRO,
	.hw_features = MTK_HW_FEATURES,
	.required_clks = MT7988_CLKS_BITMAP,
	.required_pctl = false,
	.has_sram = true,
	.rss_num = 4,
	.txrx = {
		.txd_size = sizeof(struct mtk_tx_dma_v2),
		.rxd_size = sizeof(struct mtk_rx_dma_v2),
		.tx_dma_size = MTK_DMA_SIZE(4K),
		.rx_dma_size = MTK_DMA_SIZE(1K),
		.fq_dma_size = MTK_DMA_SIZE(4K),
		.rx_dma_l4_valid = RX_DMA_L4_VALID_V2,
		.dma_max_len = MTK_TX_DMA_BUF_LEN_V2,
		.dma_len_offset = MTK_TX_DMA_BUF_SHIFT_V2,
	},
};

static const struct mtk_soc_data rt5350_data = {
	.reg_map = &mt7628_reg_map,
	.caps = MT7628_CAPS,
	.hw_features = MTK_HW_FEATURES_MT7628,
	.required_clks = MT7628_CLKS_BITMAP,
	.required_pctl = false,
	.has_sram = false,
	.rss_num = 0,
	.txrx = {
		.txd_size = sizeof(struct mtk_tx_dma),
		.rxd_size = sizeof(struct mtk_rx_dma),
		.tx_dma_size = MTK_DMA_SIZE(2K),
		.rx_dma_size = MTK_DMA_SIZE(2K),
		.fq_dma_size = MTK_DMA_SIZE(2K),
		.rx_dma_l4_valid = RX_DMA_L4_VALID_PDMA,
		.dma_max_len = MTK_TX_DMA_BUF_LEN,
		.dma_len_offset = MTK_TX_DMA_BUF_SHIFT,
	},
};

const struct of_device_id of_mtk_match[] = {
	{ .compatible = "mediatek,mt2701-eth", .data = &mt2701_data},
	{ .compatible = "mediatek,mt7621-eth", .data = &mt7621_data},
	{ .compatible = "mediatek,mt7622-eth", .data = &mt7622_data},
	{ .compatible = "mediatek,mt7623-eth", .data = &mt7623_data},
	{ .compatible = "mediatek,mt7629-eth", .data = &mt7629_data},
	{ .compatible = "mediatek,mt7986-eth", .data = &mt7986_data},
	{ .compatible = "mediatek,mt7981-eth", .data = &mt7981_data},
	{ .compatible = "mediatek,mt7988-eth", .data = &mt7988_data},
	{ .compatible = "ralink,rt5350-eth", .data = &rt5350_data},
	{},
};
MODULE_DEVICE_TABLE(of, of_mtk_match);

static struct platform_driver mtk_driver = {
	.probe = mtk_probe,
	.remove = mtk_remove,
	.driver = {
		.name = "mtk_soc_eth",
		.of_match_table = of_mtk_match,
	},
};

module_platform_driver(mtk_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("John Crispin <blogic@openwrt.org>");
MODULE_DESCRIPTION("Ethernet driver for MediaTek SoC");
