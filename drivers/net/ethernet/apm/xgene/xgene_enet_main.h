/* Applied Micro X-Gene SoC Ethernet Driver
 *
 * Copyright (c) 2014, Applied Micro Circuits Corporation
 * Authors: Iyappan Subramanian <isubramanian@apm.com>
 *	    Ravi Patel <rapatel@apm.com>
 *	    Keyur Chudgar <kchudgar@apm.com>
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef __XGENE_ENET_MAIN_H__
#define __XGENE_ENET_MAIN_H__

#include <linux/clk.h>
#include <linux/of_platform.h>
#include <linux/of_net.h>
#include <linux/of_mdio.h>
#include <linux/module.h>
#include <net/ip.h>
#include <linux/if_vlan.h>
#include <linux/phy.h>
#include "xgene_enet_hw.h"

#define XGENE_ENET_MAX_MTU	1536
#define SKB_BUFFER_SIZE		(XGENE_ENET_MAX_MTU - NET_IP_ALIGN)

#define NUM_PKT_BUF	64
#define NUM_BUFPOOL	32

/* software context of a descriptor ring */
struct xgene_enet_desc_ring {
	struct net_device *ndev;
	u16 id;
	u16 num;
	u16 head;
	u16 tail;
	u16 slots;
	u16 irq;
	u32 size;
	u32 state[NUM_RING_CONFIG];
	void __iomem *cmd_base;
	void __iomem *cmd;
	dma_addr_t dma;
	u16 dst_ring_num;
	u8 nbufpool;
	struct sk_buff *(*rx_skb);
	struct sk_buff *(*cp_skb);
	enum xgene_enet_ring_cfgsize cfgsize;
	struct xgene_enet_desc_ring *cp_ring;
	struct xgene_enet_desc_ring *buf_pool;
	struct napi_struct napi;
	struct xgene_enet_desc desc;
	union {
		void *desc_addr;
		struct xgene_enet_raw_desc *raw_desc;
		struct xgene_enet_raw_desc16 *raw_desc16;
	};
};

struct xgene_enet_rx_stats {
	u32 bytes;
	u32 pkts;
	u32 crc_err;
	u32 checksum_err;
	u32 dropped;
	u32 oversize_pkts;
	u32 undersize_pkts;
	u32 frm_len_err;
	u32 fifo_overrun;
};

struct xgene_enet_tx_stats {
	u32 byte_cntr;
	u32 pkt_cntr;
	u32 drop_frame_cntr;
	u32 fcs_error_cntr;
	u32 undersize_frame_cntr;
};

struct xgene_enet_stats {
	struct xgene_enet_rx_stats rx_stats;
	struct xgene_enet_tx_stats tx_stats;
};

/* ethernet private data */
struct xgene_enet_pdata {
	struct net_device *ndev;
	struct mii_bus *mdio_bus;
	struct phy_device *phy_dev;
	int phy_link;
	int phy_speed;
	struct clk *clk;
	struct platform_device *pdev;
	struct xgene_enet_desc_ring *tx_ring;
	struct xgene_enet_desc_ring *rx_ring;
	struct net_device_stats nstats;
	char *dev_name;
	u32 rx_buff_cnt;
	u32 tx_qcnt_hi;
	u32 cp_qcnt_hi;
	u32 cp_qcnt_low;
	u32 rx_irq;
	void __iomem *eth_csr_addr;
	void __iomem *eth_ring_if_addr;
	void __iomem *eth_diag_csr_addr;
	void __iomem *mcx_mac_addr;
	void __iomem *mcx_stats_addr;
	void __iomem *mcx_mac_csr_addr;
	void __iomem *base_addr;
	void __iomem *ring_csr_addr;
	void __iomem *ring_cmd_addr;
	u32 phy_addr;
	int phy_mode;
	u32 speed;
	u16 rm;
	struct xgene_enet_stats stats;
	spinlock_t stats_lock;
};

#endif /* __XGENE_ENET_MAIN_H__ */
