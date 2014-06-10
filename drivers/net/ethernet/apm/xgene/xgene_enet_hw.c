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

#include "xgene_enet_main.h"
#include "xgene_enet_hw.h"

u64 xgene_prepare_eth_work_msg(u8 l4hlen, u8 l3hlen, u8 ethhdr,
			       u8 csum_enable, u8 proto)
{
	u64 hopinfo;

	hopinfo = (l4hlen & TCPHDR_MASK) |
		  ((l3hlen << IPHDR_POS) & IPHDR_MASK) |
		  (ethhdr << ETHHDR_POS) |
		  ((csum_enable << EC_POS) & EC_MASK) |
		  ((proto << IS_POS) & IS_MASK) |
		  INSERT_CRC |
		  TYPE_ETH_WORK_MESSAGE;

	return hopinfo;
}

/* Tx descriptor raw write */
void xgene_set_tx_desc(struct xgene_enet_desc_ring *ring,
		       struct xgene_enet_raw_desc *raw_desc)
{
	raw_desc->m0 = ring->desc.userinfo;
	raw_desc->m1 = (ring->desc.dataaddr & DATAADDR_MASK) |
		       (((u64)ring->desc.bufdatalen << BUFDATALEN_POS) &
		       BUFDATALEN_MASK) | COHERENT_MASK;
	raw_desc->m3 = (((u64)ring->desc.henqnum << HENQNUM_POS) &
		       HENQNUM_MASK) |
		       ring->desc.hopinfo_lsb;
}

/* descriptor raw read */
void xgene_get_desc(struct xgene_enet_desc_ring *ring,
		    struct xgene_enet_raw_desc *raw_desc)
{
	struct xgene_enet_desc *desc = &ring->desc;

	desc->dataaddr = raw_desc->m1 & DATAADDR_MASK;
	desc->bufdatalen = (raw_desc->m1 & BUFDATALEN_MASK) >> BUFDATALEN_POS;
	desc->userinfo = raw_desc->m0 & USERINFO_MASK;
	desc->fpqnum = (raw_desc->m0 & FPQNUM_MASK) >> FPQNUM_POS;
	desc->status = (raw_desc->m0 & LERR_MASK) >> LERR_POS;
}

/* Bufpool descriptor raw write common fields */
void xgene_set_init_bufpool_desc(struct xgene_enet_desc_ring *ring,
				 struct xgene_enet_raw_desc16 *raw_desc)
{
	raw_desc->m0 = (ring->desc.userinfo) |
		       (((u64)ring->desc.fpqnum << FPQNUM_POS) & FPQNUM_MASK) |
		       STASHING_MASK;
}

/* Bufpool descriptor raw write */
void xgene_set_refill_bufpool_desc(struct xgene_enet_desc_ring *ring,
				   struct xgene_enet_raw_desc16 *raw_desc)
{
	raw_desc->m1 = (ring->desc.dataaddr & DATAADDR_MASK) |
		       (((u64)ring->desc.bufdatalen << BUFDATALEN_POS) &
		       BUFDATALEN_MASK) |
		       COHERENT_MASK;
}

/* Bufpool descriptor raw read */
void xgene_get_bufpool_desc(struct xgene_enet_desc_ring *ring,
			    struct xgene_enet_raw_desc16 *raw_desc)
{
	struct xgene_enet_desc *desc = &ring->desc;

	desc->userinfo = raw_desc->m0 & USERINFO_MASK;
}

static void xgene_enet_ring_init(struct xgene_enet_desc_ring *ring)
{
	u32 *ring_cfg = ring->state;
	u64 addr = ring->dma;
	enum xgene_enet_ring_cfgsize cfgsize = ring->cfgsize;

	ring_cfg[4] |= (1 << SELTHRSH_POS) &
			CREATE_MASK(SELTHRSH_POS, SELTHRSH_LEN);
	ring_cfg[3] |= ACCEPTLERR;
	ring_cfg[2] |= QCOHERENT;

	addr >>= 8;
	ring_cfg[2] |= (addr << RINGADDRL_POS) &
			CREATE_MASK_ULL(RINGADDRL_POS, RINGADDRL_LEN);
	addr >>= RINGADDRL_LEN;
	ring_cfg[3] |= addr & CREATE_MASK_ULL(RINGADDRH_POS, RINGADDRH_LEN);
	ring_cfg[3] |= ((u32) cfgsize << RINGSIZE_POS) &
			CREATE_MASK(RINGSIZE_POS, RINGSIZE_LEN);
}

static void xgene_enet_ring_set_type(struct xgene_enet_desc_ring *ring)
{
	u32 *ring_cfg = ring->state;
	bool is_bufpool;
	u32 val;

	is_bufpool = xgene_enet_is_bufpool(ring->id);
	val = (is_bufpool) ? RING_BUFPOOL : RING_REGULAR;
	ring_cfg[4] |= (val << RINGTYPE_POS) &
			CREATE_MASK(RINGTYPE_POS, RINGTYPE_LEN);

	if (is_bufpool) {
		ring_cfg[3] |= (BUFPOOL_MODE << RINGMODE_POS) &
				CREATE_MASK(RINGMODE_POS, RINGMODE_LEN);
	}
}

static void xgene_enet_ring_set_recombbuf(struct xgene_enet_desc_ring *ring)
{
	u32 *ring_cfg = ring->state;

	ring_cfg[3] |= RECOMBBUF;
	ring_cfg[3] |= (0xf << RECOMTIMEOUTL_POS) &
			CREATE_MASK(RECOMTIMEOUTL_POS, RECOMTIMEOUTL_LEN);
	ring_cfg[4] |= 0x7 & CREATE_MASK(RECOMTIMEOUTH_POS, RECOMTIMEOUTH_LEN);
}

static void xgene_enet_ring_wr32(struct xgene_enet_desc_ring *ring,
					u32 offset, u32 data)
{
	struct xgene_enet_pdata *pdata = netdev_priv(ring->ndev);

	iowrite32(data, pdata->ring_csr_addr + offset);
}

static void xgene_enet_ring_rd32(struct xgene_enet_desc_ring *ring,
					u32 offset, u32 *data)
{
	struct xgene_enet_pdata *pdata = netdev_priv(ring->ndev);

	*data = ioread32(pdata->ring_csr_addr + offset);
}

static void xgene_enet_write_ring_state(struct xgene_enet_desc_ring *ring)
{
	int i;

	xgene_enet_ring_wr32(ring, CSR_RING_CONFIG, ring->num);
	for (i = 0; i < NUM_RING_CONFIG; i++) {
		xgene_enet_ring_wr32(ring, CSR_RING_WR_BASE + (i * 4),
				     ring->state[i]);
	}
}

static void xgene_enet_clr_ring_state(struct xgene_enet_desc_ring *ring)
{
	memset(ring->state, 0, sizeof(u32) * NUM_RING_CONFIG);
	xgene_enet_write_ring_state(ring);
}

static void xgene_enet_set_ring_state(struct xgene_enet_desc_ring *ring)
{
	xgene_enet_ring_set_type(ring);

	if (xgene_enet_ring_owner(ring->id) == RING_OWNER_ETH0)
		xgene_enet_ring_set_recombbuf(ring);

	xgene_enet_ring_init(ring);
	xgene_enet_write_ring_state(ring);
}

static void xgene_enet_set_ring_id(struct xgene_enet_desc_ring *ring)
{
	u32 ring_id_val, ring_id_buf;
	bool is_bufpool;

	is_bufpool = xgene_enet_is_bufpool(ring->id);

	ring_id_val = ring->id & GENMASK(9, 0);
	ring_id_val |= OVERWRITE;

	ring_id_buf = (ring->num << 9) & GENMASK(18, 9);
	ring_id_buf |= PREFETCH_BUF_EN;
	if (is_bufpool)
		ring_id_buf |= IS_BUFFER_POOL;

	xgene_enet_ring_wr32(ring, CSR_RING_ID, ring_id_val);
	xgene_enet_ring_wr32(ring, CSR_RING_ID_BUF, ring_id_buf);
}

static void xgene_enet_clr_desc_ring_id(struct xgene_enet_desc_ring *ring)
{
	u32 ring_id;

	ring_id = ring->id | OVERWRITE;
	xgene_enet_ring_wr32(ring, CSR_RING_ID, ring_id);
	xgene_enet_ring_wr32(ring, CSR_RING_ID_BUF, 0);
}

struct xgene_enet_desc_ring *xgene_enet_setup_ring(
					struct xgene_enet_desc_ring *ring)
{
	u32 size = ring->size;
	u32 i, data;
	u64 *desc;
	bool is_bufpool;

	xgene_enet_clr_ring_state(ring);
	xgene_enet_set_ring_state(ring);
	xgene_enet_set_ring_id(ring);

	ring->slots = xgene_enet_get_numslots(ring->id, size);

	is_bufpool = xgene_enet_is_bufpool(ring->id);
	if (is_bufpool || xgene_enet_ring_owner(ring->id) != RING_OWNER_CPU)
		return ring;

	for (i = 0; i < ring->slots; i++) {
		desc = (u64 *)&ring->raw_desc[i];
		desc[EMPTY_SLOT_INDEX] = EMPTY_SLOT;
	}

	xgene_enet_ring_rd32(ring, CSR_RING_NE_INT_MODE, &data);
	data |= BIT(31 - xgene_enet_ring_bufnum(ring->id));
	xgene_enet_ring_wr32(ring, CSR_RING_NE_INT_MODE, data);

	return ring;
}

void xgene_enet_clear_ring(struct xgene_enet_desc_ring *ring)
{
	u32 data;
	bool is_bufpool;

	is_bufpool = xgene_enet_is_bufpool(ring->id);
	if (is_bufpool || xgene_enet_ring_owner(ring->id) != RING_OWNER_CPU)
		goto out;

	xgene_enet_ring_rd32(ring, CSR_RING_NE_INT_MODE, &data);
	data &= ~BIT(31 - xgene_enet_ring_bufnum(ring->id));
	xgene_enet_ring_wr32(ring, CSR_RING_NE_INT_MODE, data);

out:
	xgene_enet_clr_desc_ring_id(ring);
	xgene_enet_clr_ring_state(ring);
}

void xgene_enet_parse_error(struct xgene_enet_desc_ring *ring,
			    struct xgene_enet_pdata *pdata,
			    enum xgene_enet_err_code status)
{
	struct net_device *ndev = ring->ndev;
	struct xgene_enet_rx_stats *rx_stats = &pdata->stats.rx_stats;

	netdev_err(ndev, "LERR: %d ring_num: %d ", status, ring->num);
	switch (status) {
	case HBF_READ_DATA:
		netdev_err(ndev, "HBF read data error\n");
		break;
	case HBF_LL_READ:
		netdev_err(ndev, "HBF link list read error\n");
		break;
	case BAD_WORK_MSG:
		netdev_err(ndev, "Bad work message\n");
		break;
	case BUFPOOL_TIMEOUT:
		netdev_err(ndev, "Bufpool buffer timeout error\n");
		break;
	case INGRESS_CRC:
		netdev_err(ndev, "Ingress packet had CRC error\n");
		rx_stats->crc_err++;
		break;
	case INGRESS_CHECKSUM:
		netdev_err(ndev, "Ingress packet had checksum error\n");
		rx_stats->checksum_err++;
		break;
	case INGRESS_TRUNC_FRAME:
		netdev_err(ndev, "Ingress packet was truncated\n");
		rx_stats->oversize_pkts++;
		break;
	case INGRESS_PKT_LEN:
		netdev_err(ndev, "Ingress packet length error\n");
		rx_stats->frm_len_err++;
		break;
	case INGRESS_PKT_UNDER:
		netdev_err(ndev, "Ingress packet size was less than 64 bytes\n");
		rx_stats->undersize_pkts++;
		break;
	case INGRESS_FIFO_OVERRUN:
		netdev_err(ndev, "Ingress data FIFO overrun error\n");
		rx_stats->fifo_overrun++;
		break;
	case INGRESS_CHECKSUM_COMPUTE:
		netdev_err(ndev, "Could not compute checksum on ingress pkt\n");
		rx_stats->checksum_err++;
		break;
	default:
		netdev_err(ndev, "unexpected error\n");
		break;
	}
}

static void xgene_enet_wr_csr(struct xgene_enet_pdata *pdata,
			      u32 offset, u32 val)
{
	void *addr = pdata->eth_csr_addr + offset;

	iowrite32(val, addr);
}

static void xgene_enet_wr_ring_if(struct xgene_enet_pdata *pdata,
				  u32 offset, u32 val)
{
	void *addr = pdata->eth_ring_if_addr + offset;

	iowrite32(val, addr);
}

static void xgene_enet_wr_diag_csr(struct xgene_enet_pdata *pdata,
				   u32 offset, u32 val)
{
	void *addr = pdata->eth_diag_csr_addr + offset;

	iowrite32(val, addr);
}

static void xgene_enet_wr_mcx_csr(struct xgene_enet_pdata *pdata,
				  u32 offset, u32 val)
{
	void *addr = pdata->mcx_mac_csr_addr + offset;

	iowrite32(val, addr);
}

static bool xgene_enet_wr_indirect(void *addr, void *wr, void *cmd,
				  void *cmd_done, u32 wr_addr, u32 wr_data)
{
	u32 done;
	u8 wait = 10;

	iowrite32(wr_addr, addr);
	iowrite32(wr_data, wr);
	iowrite32(XGENE_ENET_WR_CMD, cmd);

	/* wait for write command to complete */
	while (!(done = ioread32(cmd_done)) && wait--)
		udelay(1);

	if (!done)
		return false;

	iowrite32(0, cmd);

	return true;
}

static void xgene_enet_wr_mcx_mac(struct xgene_enet_pdata *pdata,
				  u32 wr_addr, u32 wr_data)
{
	void *addr, *wr, *cmd, *cmd_done;
	bool ret;

	addr = pdata->mcx_mac_addr + MAC_ADDR_REG_OFFSET;
	wr = pdata->mcx_mac_addr + MAC_WRITE_REG_OFFSET;
	cmd = pdata->mcx_mac_addr + MAC_COMMAND_REG_OFFSET;
	cmd_done = pdata->mcx_mac_addr + MAC_COMMAND_DONE_REG_OFFSET;

	ret = xgene_enet_wr_indirect(addr, wr, cmd, cmd_done, wr_addr, wr_data);
	if (!ret)
		netdev_err(pdata->ndev, "MCX mac write failed, addr: %04x\n",
			   wr_addr);
}

static void xgene_enet_rd_csr(struct xgene_enet_pdata *pdata,
			      u32 offset, u32 *val)
{
	void *addr = pdata->eth_csr_addr + offset;

	*val = ioread32(addr);
}

static void xgene_enet_rd_diag_csr(struct xgene_enet_pdata *pdata,
				   u32 offset, u32 *val)
{
	void *addr = pdata->eth_diag_csr_addr + offset;

	*val = ioread32(addr);
}

static void xgene_enet_rd_mcx_csr(struct xgene_enet_pdata *pdata,
				  u32 offset, u32 *val)
{
	void *addr = pdata->mcx_mac_csr_addr + offset;

	*val = ioread32(addr);
}

static bool xgene_enet_rd_indirect(void *addr, void *rd, void *cmd,
				   void *cmd_done, u32 rd_addr, u32 *rd_data)
{
	u32 done;
	u8 wait = 10;

	iowrite32(rd_addr, addr);
	iowrite32(XGENE_ENET_RD_CMD, cmd);

	/* wait for read command to complete */
	while (!(done = ioread32(cmd_done)) && wait--)
		udelay(1);

	if (!done)
		return false;

	*rd_data = ioread32(rd);
	iowrite32(0, cmd);

	return true;
}

static void xgene_enet_rd_mcx_mac(struct xgene_enet_pdata *pdata,
				  u32 rd_addr, u32 *rd_data)
{
	void *addr, *rd, *cmd, *cmd_done;
	bool ret;

	addr = pdata->mcx_mac_addr + MAC_ADDR_REG_OFFSET;
	rd = pdata->mcx_mac_addr + MAC_READ_REG_OFFSET;
	cmd = pdata->mcx_mac_addr + MAC_COMMAND_REG_OFFSET;
	cmd_done = pdata->mcx_mac_addr + MAC_COMMAND_DONE_REG_OFFSET;

	ret = xgene_enet_rd_indirect(addr, rd, cmd, cmd_done, rd_addr, rd_data);
	if (!ret)
		netdev_err(pdata->ndev, "MCX mac read failed, addr: %04x\n",
			   rd_addr);
}

static void xgene_enet_rd_mcx_stats(struct xgene_enet_pdata *pdata,
				    u32 rd_addr, u32 *rd_data)
{
	void *addr, *rd, *cmd, *cmd_done;
	bool ret;

	addr = pdata->mcx_stats_addr + STAT_ADDR_REG_OFFSET;
	rd = pdata->mcx_stats_addr + STAT_READ_REG_OFFSET;
	cmd = pdata->mcx_stats_addr + STAT_COMMAND_REG_OFFSET;
	cmd_done = pdata->mcx_stats_addr + STAT_COMMAND_DONE_REG_OFFSET;

	ret = xgene_enet_rd_indirect(addr, rd, cmd, cmd_done, rd_addr, rd_data);
	if (!ret)
		netdev_err(pdata->ndev, "MCX stats read failed, addr: %04x\n",
			   rd_addr);
}

static int xgene_mii_phy_write(struct xgene_enet_pdata *pdata, int phy_id,
			       u32 reg, u16 data)
{
	u32 addr = 0, wr_data = 0;
	u32 done;
	u8 wait = 10;

	PHY_ADDR_SET(&addr, phy_id);
	REG_ADDR_SET(&addr, reg);
	xgene_enet_wr_mcx_mac(pdata, MII_MGMT_ADDRESS_ADDR, addr);

	PHY_CONTROL_SET(&wr_data, data);
	xgene_enet_wr_mcx_mac(pdata, MII_MGMT_CONTROL_ADDR, wr_data);
	do {
		usleep_range(5, 10);
		xgene_enet_rd_mcx_mac(pdata, MII_MGMT_INDICATORS_ADDR, &done);
	} while ((done & BUSY_MASK) && wait--);

	if (done & BUSY_MASK) {
		netdev_err(pdata->ndev, "MII_MGMT write failed\n");
		return -1;
	}

	return 0;
}

static int xgene_mii_phy_read(struct xgene_enet_pdata *pdata,
			      u8 phy_id, u32 reg)
{
	u32 addr = 0;
	u32 data, done;
	u8 wait = 10;

	PHY_ADDR_SET(&addr, phy_id);
	REG_ADDR_SET(&addr, reg);
	xgene_enet_wr_mcx_mac(pdata, MII_MGMT_ADDRESS_ADDR, addr);
	xgene_enet_wr_mcx_mac(pdata, MII_MGMT_COMMAND_ADDR, READ_CYCLE_MASK);
	do {
		usleep_range(5, 10);
		xgene_enet_rd_mcx_mac(pdata, MII_MGMT_INDICATORS_ADDR, &done);
	} while ((done & BUSY_MASK) && wait--);

	if (done & BUSY_MASK) {
		netdev_err(pdata->ndev, "MII_MGMT read failed\n");
		return -1;
	}

	xgene_enet_rd_mcx_mac(pdata, MII_MGMT_STATUS_ADDR, &data);
	xgene_enet_wr_mcx_mac(pdata, MII_MGMT_COMMAND_ADDR, 0);

	return data;
}

void xgene_gmac_set_mac_addr(struct xgene_enet_pdata *pdata)
{
	u32 addr0, addr1;
	u8 *dev_addr = pdata->ndev->dev_addr;

	addr0 = (dev_addr[3] << 24) | (dev_addr[2] << 16) |
		(dev_addr[1] << 8) | dev_addr[0];
	addr1 = (dev_addr[5] << 24) | (dev_addr[4] << 16);
	addr1 |= pdata->phy_addr & 0xFFFF;

	xgene_enet_wr_mcx_mac(pdata, STATION_ADDR0_ADDR, addr0);
	xgene_enet_wr_mcx_mac(pdata, STATION_ADDR1_ADDR, addr1);
}

static int xgene_enet_ecc_init(struct xgene_enet_pdata *pdata)
{
	struct net_device *ndev = pdata->ndev;
	u32 data;
	u8 wait = 10;

	xgene_enet_wr_diag_csr(pdata, ENET_CFG_MEM_RAM_SHUTDOWN_ADDR, 0x0);
	do {
		usleep_range(100, 110);
		xgene_enet_rd_diag_csr(pdata, ENET_BLOCK_MEM_RDY_ADDR, &data);
	} while ((data != 0xffffffff) && wait--);

	if (data != 0xffffffff) {
		netdev_err(ndev, "Failed to release memory from shutdown\n");
		return -ENODEV;
	}

	return 0;
}

static void xgene_gmac_phy_enable_scan_cycle(struct xgene_enet_pdata *pdata)
{
	u32 val;

	xgene_enet_rd_mcx_mac(pdata, MII_MGMT_COMMAND_ADDR, &val);
	SCAN_CYCLE_MASK_SET(&val, 1);
	xgene_enet_wr_mcx_mac(pdata, MII_MGMT_COMMAND_ADDR, val);

	/* Program phy address start scan from 0 and register at address 0x1 */
	xgene_enet_rd_mcx_mac(pdata, MII_MGMT_ADDRESS_ADDR, &val);
	PHY_ADDR_SET(&val, pdata->phy_dev->addr);
	REG_ADDR_SET(&val, MII_BMSR);
	xgene_enet_wr_mcx_mac(pdata, MII_MGMT_ADDRESS_ADDR, val);
}

void xgene_gmac_reset(struct xgene_enet_pdata *pdata)
{
	xgene_enet_wr_mcx_mac(pdata, MAC_CONFIG_1_ADDR, SOFT_RESET1);
	xgene_enet_wr_mcx_mac(pdata, MAC_CONFIG_1_ADDR, 0);
}

void xgene_gmac_init(struct xgene_enet_pdata *pdata, int speed)
{
	u32 value, mc2;
	u32 intf_ctl, rgmii;
	u32 icm0, icm2;

	xgene_gmac_reset(pdata);

	xgene_enet_rd_mcx_csr(pdata, ICM_CONFIG0_REG_0_ADDR, &icm0);
	xgene_enet_rd_mcx_csr(pdata, ICM_CONFIG2_REG_0_ADDR, &icm2);
	xgene_enet_rd_mcx_mac(pdata, MAC_CONFIG_2_ADDR, &mc2);
	xgene_enet_rd_mcx_mac(pdata, INTERFACE_CONTROL_ADDR, &intf_ctl);
	xgene_enet_rd_csr(pdata, RGMII_REG_0_ADDR, &rgmii);

	switch (speed) {
	case SPEED_10:
		ENET_INTERFACE_MODE2_SET(&mc2, 1);
		CFG_MACMODE_SET(&icm0, 0);
		CFG_WAITASYNCRD_SET(&icm2, 500);
		rgmii &= ~CFG_SPEED_1250;
		break;
	case SPEED_100:
		ENET_INTERFACE_MODE2_SET(&mc2, 1);
		intf_ctl |= ENET_LHD_MODE;
		CFG_MACMODE_SET(&icm0, 1);
		CFG_WAITASYNCRD_SET(&icm2, 80);
		rgmii &= ~CFG_SPEED_1250;
		break;
	default:
		ENET_INTERFACE_MODE2_SET(&mc2, 2);
		intf_ctl |= ENET_GHD_MODE;
		CFG_TXCLK_MUXSEL0_SET(&rgmii, 4);
		xgene_enet_rd_csr(pdata, DEBUG_REG_ADDR, &value);
		value |= CFG_BYPASS_UNISEC_TX | CFG_BYPASS_UNISEC_RX;
		xgene_enet_wr_csr(pdata, DEBUG_REG_ADDR, value);
		break;
	}

	mc2 |= FULL_DUPLEX2;
	xgene_enet_wr_mcx_mac(pdata, MAC_CONFIG_2_ADDR, mc2);
	xgene_enet_wr_mcx_mac(pdata, INTERFACE_CONTROL_ADDR, intf_ctl);

	xgene_gmac_set_mac_addr(pdata);

	/* Adjust MDC clock frequency */
	xgene_enet_rd_mcx_mac(pdata, MII_MGMT_CONFIG_ADDR, &value);
	MGMT_CLOCK_SEL_SET(&value, 7);
	xgene_enet_wr_mcx_mac(pdata, MII_MGMT_CONFIG_ADDR, value);

	/* Enable drop if bufpool not available */
	xgene_enet_rd_csr(pdata, RSIF_CONFIG_REG_ADDR, &value);
	value |= CFG_RSIF_FPBUFF_TIMEOUT_EN;
	xgene_enet_wr_csr(pdata, RSIF_CONFIG_REG_ADDR, value);

	/* Rtype should be copied from FP */
	xgene_enet_wr_csr(pdata, RSIF_RAM_DBG_REG0_ADDR, 0);
	xgene_enet_wr_csr(pdata, RGMII_REG_0_ADDR, rgmii);

	/* Rx-Tx traffic resume */
	xgene_enet_wr_csr(pdata, CFG_LINK_AGGR_RESUME_0_ADDR, TX_PORT0);

	xgene_enet_wr_mcx_csr(pdata, ICM_CONFIG0_REG_0_ADDR, icm0);
	xgene_enet_wr_mcx_csr(pdata, ICM_CONFIG2_REG_0_ADDR, icm2);

	xgene_enet_rd_mcx_csr(pdata, RX_DV_GATE_REG_0_ADDR, &value);
	value &= ~TX_DV_GATE_EN0;
	value &= ~RX_DV_GATE_EN0;
	value |= RESUME_RX0;
	xgene_enet_wr_mcx_csr(pdata, RX_DV_GATE_REG_0_ADDR, value);

	xgene_enet_wr_csr(pdata, CFG_BYPASS_ADDR, RESUME_TX);
}

void xgene_gmac_get_tx_stats(struct xgene_enet_pdata *pdata)
{
	struct xgene_enet_tx_stats *tx_stats = &pdata->stats.tx_stats;

	xgene_enet_rd_mcx_stats(pdata, TBYT_ADDR, &tx_stats->byte_cntr);
	xgene_enet_rd_mcx_stats(pdata, TPKT_ADDR, &tx_stats->pkt_cntr);
	xgene_enet_rd_mcx_stats(pdata, TDRP_ADDR, &tx_stats->drop_frame_cntr);
	xgene_enet_rd_mcx_stats(pdata, TFCS_ADDR, &tx_stats->fcs_error_cntr);
	xgene_enet_rd_mcx_stats(pdata, TUND_ADDR,
				&tx_stats->undersize_frame_cntr);
}

static void xgene_enet_config_ring_if_assoc(struct xgene_enet_pdata *pdata)
{
	u32 val = 0xffffffff;

	xgene_enet_wr_ring_if(pdata, ENET_CFGSSQMIWQASSOC_ADDR, val);
	xgene_enet_wr_ring_if(pdata, ENET_CFGSSQMIFPQASSOC_ADDR, val);
	xgene_enet_wr_ring_if(pdata, ENET_CFGSSQMIQMLITEWQASSOC_ADDR, val);
	xgene_enet_wr_ring_if(pdata, ENET_CFGSSQMIQMLITEFPQASSOC_ADDR, val);
}

void xgene_enet_cle_bypass(struct xgene_enet_pdata *pdata,
			   u32 dst_ring_num, u16 bufpool_id)
{
	u32 cb;
	u32 fpsel;

	fpsel = xgene_enet_ring_bufnum(bufpool_id) - 0x20;

	xgene_enet_rd_csr(pdata, CLE_BYPASS_REG0_0_ADDR, &cb);
	cb |= CFG_CLE_BYPASS_EN0;
	CFG_CLE_IP_PROTOCOL0_SET(&cb, 3);
	xgene_enet_wr_csr(pdata, CLE_BYPASS_REG0_0_ADDR, cb);

	xgene_enet_rd_csr(pdata, CLE_BYPASS_REG1_0_ADDR, &cb);
	CFG_CLE_DSTQID0_SET(&cb, dst_ring_num);
	CFG_CLE_FPSEL0_SET(&cb, fpsel);
	xgene_enet_wr_csr(pdata, CLE_BYPASS_REG1_0_ADDR, cb);
}

void xgene_gmac_rx_enable(struct xgene_enet_pdata *pdata)
{
	u32 data;

	xgene_enet_rd_mcx_mac(pdata, MAC_CONFIG_1_ADDR, &data);
	xgene_enet_wr_mcx_mac(pdata, MAC_CONFIG_1_ADDR, data | RX_EN);
}

void xgene_gmac_tx_enable(struct xgene_enet_pdata *pdata)
{
	u32 data;

	xgene_enet_rd_mcx_mac(pdata, MAC_CONFIG_1_ADDR, &data);
	xgene_enet_wr_mcx_mac(pdata, MAC_CONFIG_1_ADDR, data | TX_EN);
}

void xgene_gmac_rx_disable(struct xgene_enet_pdata *pdata)
{
	u32 data;

	xgene_enet_rd_mcx_mac(pdata, MAC_CONFIG_1_ADDR, &data);
	xgene_enet_wr_mcx_mac(pdata, MAC_CONFIG_1_ADDR, data & ~RX_EN);
}

void xgene_gmac_tx_disable(struct xgene_enet_pdata *pdata)
{
	u32 data;

	xgene_enet_rd_mcx_mac(pdata, MAC_CONFIG_1_ADDR, &data);
	xgene_enet_wr_mcx_mac(pdata, MAC_CONFIG_1_ADDR, data & ~TX_EN);
}

void xgene_enet_reset(struct xgene_enet_pdata *pdata)
{
	u32 val;

	clk_prepare_enable(pdata->clk);
	clk_disable_unprepare(pdata->clk);
	clk_prepare_enable(pdata->clk);
	xgene_enet_ecc_init(pdata);
	xgene_enet_config_ring_if_assoc(pdata);

	/* Enable auto-incr for scanning */
	xgene_enet_rd_mcx_mac(pdata, MII_MGMT_CONFIG_ADDR, &val);
	val |= SCAN_AUTO_INCR;
	MGMT_CLOCK_SEL_SET(&val, 1);
	xgene_enet_wr_mcx_mac(pdata, MII_MGMT_CONFIG_ADDR, val);
}

void xgene_gport_shutdown(struct xgene_enet_pdata *pdata)
{
	clk_disable_unprepare(pdata->clk);
}

static int xgene_enet_mdio_read(struct mii_bus *bus, int mii_id, int regnum)
{
	struct xgene_enet_pdata *pdata = bus->priv;
	u32 val;

	val = xgene_mii_phy_read(pdata, mii_id, regnum);
	netdev_dbg(pdata->ndev, "mdio_rd: bus=%d reg=%d val=%x\n",
		   mii_id, regnum, val);

	return val;
}

static int xgene_enet_mdio_write(struct mii_bus *bus, int mii_id, int regnum,
				 u16 val)
{
	struct xgene_enet_pdata *pdata = bus->priv;
	int ret;

	netdev_dbg(pdata->ndev, "mdio_wr: bus=%d reg=%d val=%x\n",
		   mii_id, regnum, val);
	ret = xgene_mii_phy_write(pdata, mii_id, regnum, val);

	return ret;
}

static void xgene_enet_adjust_link(struct net_device *ndev)
{
	struct xgene_enet_pdata *pdata = netdev_priv(ndev);
	struct phy_device *phydev = pdata->phy_dev;
	bool status_change = false;

	if (phydev->link && pdata->phy_speed != phydev->speed) {
		xgene_gmac_init(pdata, phydev->speed);
		pdata->phy_speed = phydev->speed;
		status_change = true;
	}

	if (pdata->phy_link != phydev->link) {
		if (!phydev->link)
			pdata->phy_speed = 0;
		pdata->phy_link = phydev->link;
		status_change = true;
	}

	if (!status_change)
		return;

	if (phydev->link) {
		xgene_gmac_rx_enable(pdata);
		xgene_gmac_tx_enable(pdata);
	} else {
		xgene_gmac_rx_disable(pdata);
		xgene_gmac_tx_disable(pdata);
	}
	phy_print_status(phydev);
}

static int xgene_enet_phy_connect(struct net_device *ndev)
{
	struct xgene_enet_pdata *pdata = netdev_priv(ndev);
	struct device_node *phy_np;
	struct phy_device *phy_dev;
	struct device *dev = &pdata->pdev->dev;

	phy_np = of_parse_phandle(dev->of_node, "phy-handle", 0);
	if (!phy_np) {
		netdev_dbg(ndev, "No phy-handle found\n");
		return -ENODEV;
	}

	phy_dev = of_phy_connect(ndev, phy_np, &xgene_enet_adjust_link,
				 0, pdata->phy_mode);
	if (!phy_dev) {
		netdev_err(ndev, "Could not connect to PHY\n");
		return  -ENODEV;
	}

	pdata->phy_link = 0;
	pdata->phy_speed = 0;
	pdata->phy_dev = phy_dev;

	return 0;
}

int xgene_enet_mdio_config(struct xgene_enet_pdata *pdata)
{
	struct net_device *ndev = pdata->ndev;
	struct device *dev = &pdata->pdev->dev;
	struct device_node *child_np;
	struct device_node *mdio_np = NULL;
	struct mii_bus *mdio_bus;
	int ret;

	for_each_child_of_node(dev->of_node, child_np) {
		if (of_device_is_compatible(child_np, "apm,xgene-mdio")) {
			mdio_np = child_np;
			break;
		}
	}

	if (!mdio_np) {
		netdev_dbg(ndev, "No mdio node in the dts\n");
		return -1;
	}

	mdio_bus = mdiobus_alloc();
	if (!mdio_bus)
		return -ENOMEM;

	mdio_bus->name = "APM X-Gene MDIO bus";
	mdio_bus->read = xgene_enet_mdio_read;
	mdio_bus->write = xgene_enet_mdio_write;
	snprintf(mdio_bus->id, MII_BUS_ID_SIZE, "%s-%s", "xgene-mii",
		 ndev->name);

	mdio_bus->irq = devm_kcalloc(dev, PHY_MAX_ADDR, sizeof(int),
				     GFP_KERNEL);
	if (!mdio_bus->irq) {
		ret = -ENOMEM;
		goto err;
	}

	mdio_bus->priv = pdata;
	mdio_bus->parent = &ndev->dev;

	ret = of_mdiobus_register(mdio_bus, mdio_np);
	if (ret) {
		netdev_err(ndev, "Failed to register MDIO bus\n");
		goto err;
	}
	pdata->mdio_bus = mdio_bus;

	ret = xgene_enet_phy_connect(ndev);
	if (ret)
		goto err;
	xgene_gmac_phy_enable_scan_cycle(pdata);

	return ret;

err:
	if (mdio_bus->irq)
		devm_kfree(dev, mdio_bus->irq);
	mdiobus_free(mdio_bus);

	return ret;
}

int xgene_enet_mdio_remove(struct xgene_enet_pdata *pdata)
{
	struct mii_bus *mdio_bus;

	mdio_bus = pdata->mdio_bus;
	mdiobus_unregister(mdio_bus);
	mdiobus_free(mdio_bus);
	pdata->mdio_bus = NULL;

	return 0;
}
