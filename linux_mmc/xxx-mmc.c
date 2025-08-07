// SPDX-License-Identifier: GPL-2.0-only

/*
 * based on sunplus-mmc.c
 */

#include <linux/bitfield.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <linux/iopoll.h>
#include <linux/mmc/core.h>
#include <linux/mmc/host.h>
#include <linux/mmc/mmc.h>
#include <linux/mmc/sdio.h>
#include <linux/mmc/slot-gpio.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pm.h>
#include <linux/pm_runtime.h>
#include <linux/reset.h>
#include <linux/dma-map-ops.h>

#define SDIO_MIN_CLK			400000
#define SDIO_MAX_CLK			5000000
#define SDIO_MAX_BLK_COUNT		65536
#define SDIO_TIMEOUT_US			500000
#define SDIO_POLL_DELAY_US		20

#define SDIO_DMA_TRANSF_MAXLEN	((1024-64)*1024)

#define SDIO_DMA_BASE_OFFSET	0x1000

#define SDIO_DMA_MODE	0
#define SDIO_PIO_MODE	1

#define DMA_TX_CHANNEL	0
#define DMA_RX_CHANNEL	1

/* Registers */
#define SDIO_RX_SADDR				0x0
#define SDIO_RX_SIZE				0x4
#define SDIO_RX_CFG					0x8
#define SDIO_CR						0xC
#define SDIO_TX_SADDR				0x10
#define SDIO_TX_SIZE				0x14
#define SDIO_TX_CFG					0x18
#define SDIO_VERSION				0x1c
#define SDIO_CMD_OP					0x20
#define SDIO_CMD_ARG				0x24
#define SDIO_DATA_SETUP				0x28
#define SDIO_START					0x2C
#define SDIO_RSP0					0x30
#define SDIO_RSP1					0x34
#define SDIO_RSP2					0x38
#define SDIO_RSP3					0x3C
#define SDIO_CLK_DIV				0x40
#define SDIO_STATUS					0x44
#define SDIO_STOP_CMD_OP			0x48
#define SDIO_STOP_CMD_ARG			0x4C
#define SDIO_DATA_TIMEOUT_CNT		0x50
#define SDIO_CMD_POWERUP_CNT		0x54
#define SDIO_CMD_WAIT_RSP_CNT		0x58
#define SDIO_CMD_WAIT_EOT_CNT		0x5C
#define SDIO_TX_DATA				0x60
#define SDIO_RX_DATA				0x64
#define SDIO_TX_MARK				0x68
#define SDIO_RX_MARK				0x6C
#define SDIO_IP						0x70
#define SDIO_IE						0x74
#define SDIO_SAMPLE_DDR				0x78
#define SDIO_RX_SADDR_HI			0x7C
#define SDIO_TX_SADDR_HI			0x80
#define SDIO_DATA_TX_DELAY_CNT		0x84
#define SDIO_DATA_CRC_TOKEN_CNT		0x88
#define SDIO_CRC_VALUE				0x8C
#define SDIO_STATUS1				0x90
#define SDIO_STOP					0x94
#define SDIO_PAD_CTRL				0x98
#define SDIO_STOP_RSP0				0xA0
#define SDIO_STOP_RSP1				0xA4
#define SDIO_STOP_RSP2				0xA8
#define SDIO_STOP_RSP3				0xAC

#define SDIO_DMA_INTR_EN			0xC00
#define SDIO_DMA_INTR_STAT			0xC04
#define SDIO_DMA_INTR_CLR			0xC08
#define SDIO_DMA_LLA_RX_LO_OFF		0xE00
#define SDIO_DMA_LLA_RX_HI_OFF		0xE04
#define SDIO_DMA_LLA_TX_LO_OFF		0xF00
#define SDIO_DMA_LLA_TX_HI_OFF		0xF04

/* Bits field */
#define SDIO_DMA_INT_EN_RX_FTRANS			BIT(0)
#define SDIO_DMA_INT_EN_RX_HFTRANS			BIT(1)
#define SDIO_DMA_INT_EN_RX_RSP_ERR			BIT(2)
#define SDIO_DMA_INT_EN_TX_FTRANS			BIT(3)
#define SDIO_DMA_INT_EN_TX_HFTRANS			BIT(4)
#define SDIO_DMA_INT_EN_TX_RSP_ERR			BIT(5)
#define SDIO_DMA_INT_EN_RX_LLA_FTRANS		BIT(9)
#define SDIO_DMA_INT_EN_RX_LLA_REG_ERR		BIT(10)
#define SDIO_DMA_INT_EN_TX_LLA_FTRANS		BIT(11)
#define SDIO_DMA_INT_EN_TX_LLA_REG_ERR		BIT(12)

#define SDIO_DMA_INT_STAT_RX_FTRANS			BIT(0)
#define SDIO_DMA_INT_STAT_RX_HFTRANS		BIT(1)
#define SDIO_DMA_INT_STAT_RX_RSP_ERR		BIT(2)
#define SDIO_DMA_INT_STAT_TX_FTRANS			BIT(3)
#define SDIO_DMA_INT_STAT_TX_HFTRANS		BIT(4)
#define SDIO_DMA_INT_STAT_TX_RSP_ERR		BIT(5)
#define SDIO_DMA_INT_STAT_RX_LLA_FTRANS		BIT(9)
#define SDIO_DMA_INT_STAT_RX_LLA_REG_ERR	BIT(10)
#define SDIO_DMA_INT_STAT_TX_LLA_FTRANS		BIT(11)
#define SDIO_DMA_INT_STAT_TX_LLA_REG_ERR	BIT(12)

#define SDIO_DMA_INT_CLR_RX_FTRANS			BIT(0)
#define SDIO_DMA_INT_CLR_RX_HFTRANS			BIT(1)
#define SDIO_DMA_INT_CLR_RX_RSP_ERR			BIT(2)
#define SDIO_DMA_INT_CLR_TX_FTRANS			BIT(3)
#define SDIO_DMA_INT_CLR_TX_HFTRANS			BIT(4)
#define SDIO_DMA_INT_CLR_TX_RSP_ERR			BIT(5)
#define SDIO_DMA_INT_CLR_RX_LLA_FTRANS		BIT(9)
#define SDIO_DMA_INT_CLR_RX_LLA_REG_ERR		BIT(10)
#define SDIO_DMA_INT_CLR_TX_LLA_FTRANS		BIT(11)
#define SDIO_DMA_INT_CLR_TX_LLA_REG_ERR		BIT(12)

#define SDIO_RXFIFO_EMPTY				BIT(2)
#define SDIO_TXFIFO_FULL				BIT(3)

#define SDIO_DMA_TX_RX_EN				BIT(4)
#define SDIO_DMA_DATASIZE_MASK			GENMASK(2,1)
#define SDIO_DMA_DATASIZE_BYTE			(0x0U & GENMASK(2,1))
#define SDIO_DMA_DATASIZE_HALF_WORD		(BIT(1) & GENMASK(2,1))
#define SDIO_DMA_DATASIZE_WORD			(BIT(2) & GENMASK(2,1))

#define SDIO_CR_DMA							BIT(0)
#define SDIO_CR_DDR							BIT(1)
#define SDIO_CR_AUTO_CMD12					BIT(2)
#define SDIO_CR_BLOCK						BIT(3)
#define SDIO_CR_CLK_STOP					BIT(5)
#define SDIO_CR_HIGH_WIDTH_MODE				BIT(6)
#define SDIO_CR_DATA_CRC_CHECK				BIT(7)
#define SDIO_CR_DATA_BUS_SPEED				BIT(8)
#define SDIO_CR_IRQ_PERIOD_CHECK			BIT(9)
#define SDIO_CR_STREAM_MODE					BIT(10)
#define SDIO_CR_SLEEP_MODE					BIT(11)
#define SDIO_CR_VOLTAGE_SWITCH_MODE_ENABLE	BIT(20)
#define SDIO_CR_POWER_MODE_MASK				GENMASK(22,21)
#define SDIO_CR_POWER_ON					((BIT(22) | BIT(21)) & SDIO_CR_POWER_MODE_MASK)

#define SDIO_IE_TX_WM				BIT(0)
#define SDIO_IE_RX_WM				BIT(1)
#define SDIO_IE_EOT					BIT(2)
#define SDIO_IE_ERR					BIT(3)
#define SDIO_IE_TX_UDF				BIT(4)
#define SDIO_IE_TX_OVF				BIT(5)
#define SDIO_IE_RX_UDF				BIT(6)
#define SDIO_IE_RX_OVF				BIT(7)

#define SDIO_IP_TX_WM				BIT(0)
#define SDIO_IP_RX_WM				BIT(1)

#define SDIO_STATUS_EOT				BIT(0)
#define SDIO_STATUS_ERR				BIT(1)
#define SDIO_STATUS_TXUDR_ERR		BIT(2)
#define SDIO_STATUS_TXOVF_ERR		BIT(3)
#define SDIO_STATUS_RXUDR_ERR		BIT(4)
#define SDIO_STATUS_RXOVF_ERR		BIT(5)
#define SDIO_STATUS_BUSY			BIT(6)
#define SDIO_STATUS_CLR_FIFO		BIT(7)

#define SDIO_STATUS_CMDERR_RSP_TO	BIT(16)
#define SDIO_STATUS_CMDERR_WrongDir	BIT(17)
#define SDIO_STATUS_CMDERR_BUSY_TO	GENMASK(17,16)
#define SDIO_STATUS_CMDERR_CRC		BIT(19)

#define SDIO_STATUS_DATAERR_RSP_TO	BIT(24)
#define SDIO_STATUS_DATAERR_BUSY_TO	BIT(25)
#define SDIO_STATUS_DATAERR_TO		GENMASK(25,24)
#define SDIO_STATUS_DATAERR_CRC		BIT(26)

#define SDIO_CMD_OP_RSP_EN			BIT(0)
#define SDIO_CMD_OP_RSP_LEN_136BIT	BIT(1)
#define SDIO_CMD_OP_CRC_EN			BIT(2)
#define SDIO_CMD_OP_BUSY_CHECK		BIT(3)
#define SDIO_CMD_OP_POWER_EN		BIT(4)
#define SDIO_CMD_OP_CRC_CHECK_EN	BIT(5)
#define SDIO_CMD_OP_STOP_CMD		BIT(6)
#define SDIO_CMD_OP_CMD_EN			BIT(7)
#define SDIO_CMD_OP_CMD_CODE		GENMASK(13,8)

#define SDIO_DATA_SETUP_EN			BIT(0)
#define SDIO_DATA_SETUP_RD			BIT(1)
#define SDIO_DATA_SETUP_WR			(0UL)

#define SDIO_DATA_SETUP_MODE(width)  (SDIO_MMC_BUS_WIDTH_##width & GENMASK(3, 2))
#define SDIO_MMC_BUS_WIDTH(width)	  SDIO_DATA_SETUP_MODE(width)
#define SDIO_MMC_BUS_WIDTH_0  0x0U
#define SDIO_MMC_BUS_WIDTH_2  BIT(2)
#define SDIO_MMC_BUS_WIDTH_3  BIT(3)

#define SDIO_DATA_SETUP_BLK_NUM		GENMASK(19, 4)
#define SDIO_MMC_BLOCK_NUM(regval)	(SDIO_DATA_SETUP_BLK_NUM & ((uint32_t)(regval - 1) << 4))

#define SDIO_DATA_SETUP_BLK_SIZE	GENMASK(31, 20)
#define SDIO_MMC_BLOCK_SIZE(regval)	(SDIO_DATA_SETUP_BLK_SIZE & ((uint32_t)(regval - 1) << 20))

/* FIFO depth is 64(unit:word), 64*4 = 256 bytes*/
#define NUCLEI_SDIO_FIFO_DEPTH			64
#define NUCLEI_SDIO_RX_MARK_THRESHOLD	(NUCLEI_SDIO_FIFO_DEPTH - 16)

#define NUCLEI_SDIO_INTR_TYPE_DATA	1
#define NUCLEI_SDIO_INTR_TYPE_EOT	2

struct nclmmc_lla_desc {
	u32 addr_lo;
	u32 block_size;
	u32 data_size;
	u32 lla_addr_lo;
	u32 lla_addr_hi;
	u32 addr_hi;
};

struct nclmmc_host {
	void __iomem *base;
	void __iomem *dma_base;
	struct clk *clk;
	struct reset_control *rstc;
	struct mmc_host *mmc;
	struct mmc_request *mrq; /* current mrq */
	int irq;
	int bus_width;
	int cur_clk;
	uint32_t dmapio_mode;
	uint32_t total_bytes_left;
	//uint32_t transfer_blks;
	//uint32_t blksz;
	uint32_t status;
	uint32_t intr_type;
	struct scatterlist *cur_sg;
	uint32_t sg_offset;

    struct nclmmc_lla_desc *current_desc_chain;
    dma_addr_t current_desc_dma;
};

static inline int nclmmc_wait_finish(struct nclmmc_host *host, u32 *status)
{
	u32 state;
	int ret;

	ret = readl_poll_timeout(host->base + SDIO_STATUS, state, (state & SDIO_STATUS_EOT), \
							SDIO_POLL_DELAY_US, SDIO_TIMEOUT_US);

	if (!ret) {
		if (status)
			*status = state;
		writel(state, host->base + SDIO_STATUS);
	} else {
		printk(KERN_ERR"pollret:%x time out:%x\n", ret, state);
	}

	return ret;
}

static void nclmmc_get_rsp(struct nclmmc_host *host, struct mmc_command *cmd)
{
	if (!(cmd->flags & MMC_RSP_PRESENT)) {
		return;
	}

	if (cmd->flags & MMC_RSP_136) {
		cmd->resp[0] = readl(host->base + SDIO_RSP3);
		cmd->resp[1] = readl(host->base + SDIO_RSP2);
		cmd->resp[2] = readl(host->base + SDIO_RSP1);
		cmd->resp[3] = readl(host->base + SDIO_RSP0);
	} else {
		cmd->resp[0] = readl(host->base + SDIO_RSP0);
	}
}

static void nclmmc_close_data_channel(struct nclmmc_host *host)
{
	u32 tmp_data_setup_val = readl(host->base + SDIO_DATA_SETUP);

	tmp_data_setup_val &= ~SDIO_DATA_SETUP_EN;
	writel(tmp_data_setup_val, host->base + SDIO_DATA_SETUP);
}

static void nclmmc_set_bus_clk(struct nclmmc_host *host, int clk)
{
	unsigned int clk_div;
	u32 ker_clk;

	if (clk > 0 && host->cur_clk != clk) {
		ker_clk = clk_get_rate(host->clk);
		clk_div = (ker_clk + 2 * clk - 1) /  (2 * clk) - 1;
		writel(clk_div, host->base + SDIO_CLK_DIV);
		host->cur_clk = clk;
	} else {
		/* do nothing */
	}
}

static void nclmmc_set_bus_width(struct nclmmc_host *host, int width)
{
	switch (width) {
	case MMC_BUS_WIDTH_8:
		host->bus_width = SDIO_MMC_BUS_WIDTH(MMC_BUS_WIDTH_8);
		break;
	case MMC_BUS_WIDTH_4:
		host->bus_width = SDIO_MMC_BUS_WIDTH(MMC_BUS_WIDTH_4);
		break;
	default:
		host->bus_width = SDIO_MMC_BUS_WIDTH(MMC_BUS_WIDTH_1);
		break;
	}
}

static void nclmmc_prepare_cmd(struct nclmmc_host *host, struct mmc_command *cmd)
{
	u32 tmp_op_val = readl(host->base + SDIO_CMD_OP);

	tmp_op_val &= ~(SDIO_CMD_OP_RSP_EN | SDIO_CMD_OP_RSP_LEN_136BIT | SDIO_CMD_OP_CRC_EN | \
			 SDIO_CMD_OP_BUSY_CHECK | SDIO_CMD_OP_POWER_EN | SDIO_CMD_OP_CRC_CHECK_EN | \
			 SDIO_CMD_OP_STOP_CMD | SDIO_CMD_OP_CMD_EN | SDIO_CMD_OP_CMD_CODE );

	if(cmd->opcode == MMC_GO_IDLE_STATE) {
		tmp_op_val |= SDIO_CMD_OP_POWER_EN;
	} else if (cmd->opcode == MMC_STOP_TRANSMISSION) {
		tmp_op_val |= SDIO_CMD_OP_STOP_CMD;
	} else {
		/* do nothing */
	}

	tmp_op_val |= cmd->opcode << 8;
	tmp_op_val |= mmc_resp_type(cmd) & 0xf;

	tmp_op_val |= (SDIO_CMD_OP_CMD_EN);
	writel(tmp_op_val, host->base + SDIO_CMD_OP);
	writel(cmd->arg, host->base + SDIO_CMD_ARG);

	if(!host->mrq->data) {
		nclmmc_close_data_channel(host);
	}
}

static void nclmmc_prepare_data(struct nclmmc_host *host, struct mmc_data *data)
{
	u32 tmp_data_setup_val = 0;
	int blksz_bits = ffs(data->blksz) - 1;

	if (!data){
		return;
	}

	BUG_ON(1 << blksz_bits != data->blksz);

	tmp_data_setup_val = SDIO_DATA_SETUP_EN;

	if (data->flags & MMC_DATA_READ) {
		tmp_data_setup_val |= SDIO_DATA_SETUP_RD;
	} else {
		tmp_data_setup_val |= SDIO_DATA_SETUP_WR;
	}

	tmp_data_setup_val |= host->bus_width;
	tmp_data_setup_val |= SDIO_MMC_BLOCK_NUM(data->blocks);
	tmp_data_setup_val |= SDIO_MMC_BLOCK_SIZE(data->blksz);
	writel(tmp_data_setup_val, host->base + SDIO_DATA_SETUP);

	/* enable stop clk when transfer data */
	tmp_data_setup_val = readl(host->base + SDIO_CR);
	tmp_data_setup_val |= SDIO_CR_CLK_STOP;
	writel(tmp_data_setup_val, host->base + SDIO_CR);

	host->total_bytes_left = data->blocks * data->blksz;
	host->cur_sg = data->sg;
	host->sg_offset = 0;
}

static inline void nclmmc_trigger_transaction(struct nclmmc_host *host)
{
	writel(1, host->base + SDIO_START);
}

static void nclmmc_send_stop_cmd(struct nclmmc_host *host)
{
	struct mmc_command stop_cmd = {};

	stop_cmd.opcode = MMC_STOP_TRANSMISSION;
	stop_cmd.arg = 0;
	stop_cmd.flags = MMC_RSP_R1B;
	nclmmc_prepare_cmd(host, &stop_cmd);
	nclmmc_trigger_transaction(host);
	nclmmc_wait_finish(host, NULL);
}

static int nclmmc_check_error(struct nclmmc_host *host, struct mmc_request *mrq)
{
	int ret = 0;
	struct mmc_command *cmd = mrq->cmd;
	struct mmc_data *data = mrq->data;

	u32 status = readl(host->base + SDIO_STATUS);

	if (status & SDIO_STATUS_ERR) {
		printk(KERN_ERR"cmd:	%d args:0x%x\n",cmd->opcode, readl(host->base + SDIO_CMD_ARG));
		printk(KERN_ERR"cmd_op: %x err_status:0x%x data_setup:0x%x\n", readl(host->base + SDIO_CMD_OP),	\
						status, readl(host->base + SDIO_DATA_SETUP));

		ret = -ETIMEDOUT;
		if(status & SDIO_STATUS_CMDERR_BUSY_TO) {
			printk(KERN_ERR "card cmd response busy\n");
		} else if(status & SDIO_STATUS_CMDERR_RSP_TO) {
			printk(KERN_ERR "card cmd response time out\n");
		} else if (status & SDIO_STATUS_CMDERR_WrongDir) {
			printk(KERN_ERR "sdio cmd wrong direction\n");
		} else if (status & SDIO_STATUS_CMDERR_CRC) {
			printk(KERN_ERR "sdio response crc err\n");
		} else if(status & SDIO_STATUS_DATAERR_TO) {
			printk(KERN_ERR "sdio tx crc sta time out\n");
		} else if(status & SDIO_STATUS_DATAERR_RSP_TO) {
			printk(KERN_ERR "card data response time out\n");
		} else if (status & SDIO_STATUS_DATAERR_BUSY_TO) {
			printk(KERN_ERR "sdio data busy timeout\n");
		} else if (status & SDIO_STATUS_DATAERR_CRC) {
			printk(KERN_ERR "sdio data crc err\n");
		}

		cmd->error = ret;
		if (data) {
			data->error = ret;
			data->bytes_xfered = 0;
		}
	} else if (status & (SDIO_STATUS_TXUDR_ERR | SDIO_STATUS_TXOVF_ERR | \
						SDIO_STATUS_RXUDR_ERR | SDIO_STATUS_RXOVF_ERR)) {
		printk(KERN_ERR"cmd: %d tx/rx fifo OVF/UDF err status:0x%x\n",cmd->opcode, status);
		data->error = -ECOMM;
	} else if (data) {
		data->error = 0;
		data->bytes_xfered = data->blocks * data->blksz;
	} else
		cmd->error = 0;

	return ret;
}

void dump_data(char *buf, int len)
{
#if 0
	int i;

	for(i = 0; i < len; i++) {
		printk(KERN_CONT "%02x ", buf[i]);
		if ((i+1)%16 ==0)
			printk("\n");
	}
#endif
}

static void nclmmc_controller_init(struct nclmmc_host *host)
{
	int ret = reset_control_assert(host->rstc);

	if (!ret) {
		usleep_range(1000, 1250);
		ret = reset_control_deassert(host->rstc);
	}

	writel(0x49, host->base + SDIO_CMD_POWERUP_CNT);
	writel(0x7ffff, host->base + SDIO_DATA_TIMEOUT_CNT);
	writel(0x3f, host->base + SDIO_CMD_WAIT_RSP_CNT);
	writel(0xffffff, host->base + SDIO_CMD_WAIT_EOT_CNT);
	writel(0x7, host->base + SDIO_DATA_TX_DELAY_CNT);

	u32 tmp_cr_val = readl(host->base + SDIO_CR);

	tmp_cr_val &= ~(SDIO_CR_AUTO_CMD12 | SDIO_CR_CLK_STOP |	\
				SDIO_CR_HIGH_WIDTH_MODE | SDIO_CR_DATA_CRC_CHECK);

	tmp_cr_val |= (SDIO_CR_HIGH_WIDTH_MODE | SDIO_CR_DATA_CRC_CHECK | SDIO_CR_POWER_ON);

	writel(tmp_cr_val, host->base + SDIO_CR);

	writel(0, host->base + SDIO_IE);
}

/*
 * 1. unmap scatterlist if needed;
 * 2. get response & check error conditions;
 * 3. notify mmc layer the request is done
 */
static void nclmmc_finish_request(struct nclmmc_host *host, struct mmc_request *mrq)
{
	struct mmc_command *cmd;
	struct mmc_data *data;

	if (!mrq)
		return;

	/*
	 * check interrupt type
	 * 1. if EOT type, get rsp, check status error,
	 * 2. if Data type, push or pull data from sdio hw
	 */
	cmd = mrq->cmd;
	data = mrq->data;

	if (host->intr_type == NUCLEI_SDIO_INTR_TYPE_EOT) {
		nclmmc_get_rsp(host, cmd);
		nclmmc_check_error(host, mrq);
		if (mrq->stop) {
			/* stop cmd use poll mode, avoid recursion*/
			nclmmc_send_stop_cmd(host);
		}
		host->mrq = NULL;
		mmc_request_done(host->mmc, mrq);
	} else if (host->intr_type == NUCLEI_SDIO_INTR_TYPE_DATA) {
		int i;
		u32 len;
		u32 *virt_addr;
		u32 cur_sg_len;
		/* read data */
		if (data->flags & MMC_DATA_READ) {
			u32 rx_watermark = readl(host->base + SDIO_RX_MARK);

			if (rx_watermark == 0) {
				len = host->total_bytes_left;
			} else {
				len = rx_watermark * 4;
			}

			while(len) {
				virt_addr = sg_virt(host->cur_sg) + host->sg_offset;
				cur_sg_len = host->cur_sg->length - host->sg_offset;

				if (cur_sg_len & 0x3) {
					printk(KERN_ERR"rx sg is not 4 byte aliged");
				}

				if (len > cur_sg_len) {
					for(i = 0; i < cur_sg_len; i+=4) {
						if (rx_watermark == 0) {
							while(readl(host->base + SDIO_IP) & SDIO_RXFIFO_EMPTY);
						}
						*virt_addr++ = readl(host->base + SDIO_RX_DATA);
					}
					host->cur_sg = sg_next(host->cur_sg);
					host->sg_offset = 0;
					len -= cur_sg_len;
				} else {
					for(i = 0; i < len; i+=4) {
						if (rx_watermark == 0) {
							while(readl(host->base + SDIO_IP) & SDIO_RXFIFO_EMPTY);
						}
						*virt_addr++ = readl(host->base + SDIO_RX_DATA);
					}
					host->sg_offset +=len;
					if (host->sg_offset == host->cur_sg->length) {
						host->cur_sg = sg_next(host->cur_sg);
						host->sg_offset = 0;
					}
					len = 0;
				}
			}

			if (rx_watermark == 0) {
				/* wait eot irq occur */
				host->total_bytes_left = 0;
				writel(SDIO_IE_EOT, host->base + SDIO_IE);
			} else {
				host->total_bytes_left -= rx_watermark * 4;
				/* change rx wm to 0 if rx data len is less than rx threshold */
				if (host->total_bytes_left <= rx_watermark * 4) {
					writel(0, host->base + SDIO_RX_MARK);
				}
				writel(SDIO_IE_RX_WM, host->base + SDIO_IE);
			}
		} else {
			/* write data from sglist to fifo */
			if (host->total_bytes_left > NUCLEI_SDIO_FIFO_DEPTH * 4)
				len = NUCLEI_SDIO_FIFO_DEPTH * 4;
			else {
				len = host->total_bytes_left;
			}

			host->total_bytes_left -= len;

			while(len) {
				virt_addr = sg_virt(host->cur_sg) + host->sg_offset;
				cur_sg_len = host->cur_sg->length - host->sg_offset;

				if (cur_sg_len & 0x3) {
					printk(KERN_ERR"tx sg is not 4 byte aliged");
				}

				if (len > cur_sg_len) {
					for(i = 0; i < cur_sg_len; i+=4){
						writel(*virt_addr++, host->base + SDIO_TX_DATA);
					}
					len -= cur_sg_len;
					host->cur_sg = sg_next(host->cur_sg);
					host->sg_offset = 0;
				} else {
					for(i = 0; i < len; i+=4) {
						writel(*virt_addr++, host->base + SDIO_TX_DATA);
					}
					host->sg_offset +=len;
					if (host->sg_offset == host->cur_sg->length) {
						host->cur_sg = sg_next(host->cur_sg);
						host->sg_offset = 0;
					}
					len = 0;
				}
			}

			if (host->total_bytes_left > 0)
				writel(SDIO_IE_TX_WM, host->base + SDIO_IE);
			else {
				/* disable tx watermark */
				writel(0, host->base + SDIO_TX_MARK);
				writel(SDIO_IE_EOT, host->base + SDIO_IE);
			}
		}
	}
}

/* Interrupt Service Routine */
static irqreturn_t nclmmc_irq(int irq, void *dev_id)
{
	struct nclmmc_host * host = (struct nclmmc_host *)dev_id;
	struct mmc_request *req = host->mrq;
	u32 status;
	u32 pending;

	pending = readl(host->base + SDIO_IP);
	status = readl(host->base + SDIO_STATUS);
	writel(0, host->base + SDIO_IE);
	if (req->data) {
		if (host->dmapio_mode == SDIO_PIO_MODE) {
			if (pending & (SDIO_IP_RX_WM | SDIO_IP_TX_WM)) {
				host->intr_type = NUCLEI_SDIO_INTR_TYPE_DATA;
				return IRQ_WAKE_THREAD;
			}
		}
	}

	if (status & SDIO_STATUS_EOT) {
		host->intr_type = NUCLEI_SDIO_INTR_TYPE_EOT;
		writel(SDIO_STATUS_EOT, host->base + SDIO_STATUS);
		return IRQ_WAKE_THREAD;
	}

	if (status & SDIO_STATUS_ERR) {
		printk(KERN_ERR "sdio err\n");
	} else if (status & SDIO_STATUS_TXOVF_ERR) {
		printk(KERN_ERR "sdio tx overflow\n");
	} else if (status & SDIO_STATUS_TXUDR_ERR) {
		printk(KERN_ERR "sdio tx underflow\n");
	} else if (status & SDIO_STATUS_RXOVF_ERR) {
		printk(KERN_ERR "sdio rx overflow\n");
	} else if (status & SDIO_STATUS_RXUDR_ERR) {
		printk(KERN_ERR "sdio rx underflow\n");
	} else {
		printk(KERN_ERR "sdio err irq:%d\n", irq);
	}

	return IRQ_HANDLED;
}

void nclmmc_prepare_dma_lla(struct nclmmc_host *host, dma_addr_t desc_dma, int sg_count, struct mmc_data *data)
{
	u32 tmp_cr_val;

	if (sg_count == 1) {
		if (data->flags & MMC_DATA_READ) {
			writel(0, host->dma_base + SDIO_DMA_LLA_RX_LO_OFF);
			writel(0, host->dma_base + SDIO_DMA_LLA_RX_HI_OFF);
		} else {
			writel(0, host->dma_base + SDIO_DMA_LLA_TX_LO_OFF);
			writel(0, host->dma_base + SDIO_DMA_LLA_TX_HI_OFF);
		}
	} else {
		if (data->flags & MMC_DATA_READ) {
			writel(lower_32_bits(desc_dma), host->dma_base + SDIO_DMA_LLA_RX_LO_OFF);
			writel(upper_32_bits(desc_dma), host->dma_base + SDIO_DMA_LLA_RX_HI_OFF);
		} else {
			writel(lower_32_bits(desc_dma), host->dma_base + SDIO_DMA_LLA_TX_LO_OFF);
			writel(upper_32_bits(desc_dma), host->dma_base + SDIO_DMA_LLA_TX_HI_OFF);
		}
	}

	struct scatterlist *first_sg = data->sg;
	dma_addr_t first_addr = sg_dma_address(first_sg);
	u32 first_size = sg_dma_len(first_sg);

	if (data->flags & MMC_DATA_READ) {
		writel(lower_32_bits(first_addr), host->base + SDIO_RX_SADDR);
		writel(upper_32_bits(first_addr), host->base + SDIO_RX_SADDR_HI);
		writel(first_size, host->base + SDIO_RX_SIZE);
		writel((SDIO_DMA_TX_RX_EN | SDIO_DMA_DATASIZE_WORD), host->base + SDIO_RX_CFG);
	} else {
		writel(lower_32_bits(first_addr), host->base + SDIO_TX_SADDR);
		writel(upper_32_bits(first_addr), host->base + SDIO_TX_SADDR_HI);
		writel(first_size, host->base + SDIO_TX_SIZE);
		writel((SDIO_DMA_TX_RX_EN | SDIO_DMA_DATASIZE_WORD), host->base + SDIO_TX_CFG);
	}

	tmp_cr_val = readl(host->base + SDIO_CR);
	tmp_cr_val |= SDIO_CR_DMA;
	writel(tmp_cr_val, host->base + SDIO_CR);
	mb();
}

struct nclmmc_lla_desc* nclmmc_prepare_dma_lla_desc(struct device *dev, struct mmc_data *data, int sg_count, dma_addr_t *desc_dma_handle)
{
	int i;
	struct scatterlist *sg;
	struct nclmmc_lla_desc *desc_chain;
	dma_addr_t next_desc_dma;

	desc_chain = dma_alloc_coherent(dev, sizeof(struct nclmmc_lla_desc) * sg_count, desc_dma_handle, GFP_KERNEL);
	if (!desc_chain) {
		dev_err(dev, "failed to allocate DMA descriptor memory\n");
		return NULL;
	}

	for_each_sg(data->sg, sg, sg_count, i) {
		dma_addr_t dma_data_addr = sg_dma_address(sg);
		u32 dma_data_size = data->blksz;

		desc_chain[i].addr_lo = lower_32_bits(dma_data_addr);
		desc_chain[i].addr_hi = upper_32_bits(dma_data_addr);
		desc_chain[i].block_size = dma_data_size;
		desc_chain[i].data_size = SDIO_DMA_DATASIZE_WORD;

        if (i < sg_count - 1) {
            next_desc_dma = *desc_dma_handle + (i + 1) * sizeof(struct nclmmc_lla_desc);
        } else {
            next_desc_dma = 0;
        }

		desc_chain[i].lla_addr_lo = lower_32_bits(next_desc_dma);
		desc_chain[i].lla_addr_hi = upper_32_bits(next_desc_dma);
	}

	return desc_chain;
}

static void nclmmc_request(struct mmc_host *mmc, struct mmc_request *mrq)
{
	struct nclmmc_host *host = mmc_priv(mmc);
	struct mmc_command *cmd = mrq->cmd;
	struct mmc_data *data = mrq->data;
	u32 interrupt_en = 0;
	host->mrq = mrq;
	int sg_count = 1;

	nclmmc_prepare_cmd(host, cmd);

	if (data) {
		nclmmc_prepare_data(host, data);

		if (host->dmapio_mode == SDIO_PIO_MODE) {
			if (data->flags & MMC_DATA_READ) {
				if (host->total_bytes_left >	NUCLEI_SDIO_RX_MARK_THRESHOLD * 4) {
					writel(NUCLEI_SDIO_RX_MARK_THRESHOLD, host->base + SDIO_RX_MARK);
				} else {
					writel(0, host->base + SDIO_RX_MARK);
				}
				interrupt_en = SDIO_IE_RX_WM;
			} else if (data->flags & MMC_DATA_WRITE) {
				writel(1, host->base + SDIO_TX_MARK);
				interrupt_en = SDIO_IE_TX_WM;
			}
		} else if (host->dmapio_mode == SDIO_DMA_MODE) {
			struct device *dev = host->mmc->parent;
			dma_addr_t desc_dma;
			struct nclmmc_lla_desc *desc_chain;

			sg_count = dma_map_sg(dev, data->sg, data->sg_len, mmc_get_dma_dir(data));
			if (!sg_count) {
				data->error = -EINVAL;
				return;
			}

			desc_chain = nclmmc_prepare_dma_lla_desc(dev, data, sg_count, &desc_dma);
			if (!desc_chain) {
				printk(KERN_ERR "dma lla descriptors prepare err\n");
				dma_unmap_sg(dev, data->sg, data->sg_len, mmc_get_dma_dir(data));
				return;
			}

			nclmmc_prepare_dma_lla(host, desc_dma, sg_count, data);

			host->current_desc_chain = desc_chain;
			host->current_desc_dma = desc_dma;

			interrupt_en = SDIO_IE_EOT;
		}
	} else {
		interrupt_en = SDIO_IE_EOT;
	}

	writel(interrupt_en, host->base + SDIO_IE);

	nclmmc_trigger_transaction(host);
}

static void nclmmc_set_ios(struct mmc_host *mmc, struct mmc_ios *ios)
{
	struct nclmmc_host *host = (struct nclmmc_host *)mmc_priv(mmc);

	nclmmc_set_bus_clk(host, ios->clock);
	nclmmc_set_bus_width(host, ios->bus_width);
}

static const struct mmc_host_ops nclmmc_ops = {
	.request = nclmmc_request,
	.set_ios = nclmmc_set_ios,
};

static irqreturn_t nclmmc_func_irq_worker(int irq, void *dev_id)
{
	struct nclmmc_host *host = dev_id;

	nclmmc_finish_request(host, host->mrq);

	return IRQ_HANDLED;
}

static int nclmmc_drv_probe(struct platform_device *pdev)
{
	struct mmc_host *mmc;
	struct resource *res;
	struct nclmmc_host *host;
	int ret = 0;

	mmc = devm_mmc_alloc_host(&pdev->dev, sizeof(struct nclmmc_host));
	if (!mmc)
		return -ENOMEM;

	host = mmc_priv(mmc);
	host->mmc = mmc;

	/* default support dma mode */
	host->dmapio_mode = SDIO_DMA_MODE;

	host->base = devm_platform_get_and_ioremap_resource(pdev, 0, &res);
	if (IS_ERR(host->base))
		return PTR_ERR(host->base);

	host->dma_base = host->base + SDIO_DMA_BASE_OFFSET;

	host->clk = devm_clk_get(&pdev->dev, "sdio_data_clk");
	if (IS_ERR(host->clk))
		return dev_err_probe(&pdev->dev, PTR_ERR(host->clk), "clk get fail\n");

	host->rstc = devm_reset_control_get_exclusive(&pdev->dev, NULL);
	if (IS_ERR(host->rstc))
		return dev_err_probe(&pdev->dev, PTR_ERR(host->rstc), "rst get fail\n");

	host->irq = platform_get_irq(pdev, 0);
	if (host->irq < 0)
		return host->irq;

	ret = devm_request_threaded_irq(&pdev->dev, host->irq,
					nclmmc_irq, nclmmc_func_irq_worker, IRQF_SHARED,
			NULL, host);
	if (ret)
		return ret;

	ret = clk_prepare_enable(host->clk);
	if (ret)
		return dev_err_probe(&pdev->dev, ret, "failed to enable clk\n");

	ret = mmc_of_parse(mmc);
	if (ret)
		goto clk_disable;

	mmc->ops = &nclmmc_ops;
	mmc->f_min = SDIO_MIN_CLK;
	if (mmc->f_max > SDIO_MAX_CLK)
		mmc->f_max = SDIO_MAX_CLK;

	if (!mmc->ocr_avail)
		mmc->ocr_avail = MMC_VDD_32_33 | MMC_VDD_33_34;
	mmc->max_req_size = SDIO_MAX_BLK_COUNT * 512;
	mmc->max_blk_size = 512;
	mmc->max_blk_count = SDIO_MAX_BLK_COUNT;
	mmc->max_seg_size = SDIO_DMA_TRANSF_MAXLEN;
	dev_set_drvdata(&pdev->dev, host);
	nclmmc_controller_init(host);

	ret = mmc_add_host(mmc);

	if (ret)
		goto clk_disable;

	return 0;

clk_disable:
	clk_disable_unprepare(host->clk);
	return ret;
}

static void nclmmc_drv_remove(struct platform_device *dev)
{
	struct nclmmc_host *host = platform_get_drvdata(dev);

	mmc_remove_host(host->mmc);
	clk_disable_unprepare(host->clk);
}

static const struct of_device_id nclmmc_of_table[] = {
	{
		.compatible = "nuclei,mmc",
	},
	{/* sentinel */}
};
MODULE_DEVICE_TABLE(of, nclmmc_of_table);

static struct platform_driver nclmmc_driver = {
	.probe = nclmmc_drv_probe,
	.remove_new = nclmmc_drv_remove,
	.driver = {
		.name = "nuclei-mmc",
		.of_match_table = nclmmc_of_table,
	},
};
module_platform_driver(nclmmc_driver);

MODULE_DESCRIPTION("Nuclei MMC controller driver");
MODULE_LICENSE("GPL");
