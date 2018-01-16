#include <platform_def.h>
#include <mmio.h>
#include <console_printf.h>


#define CAD_SPI_SSIENR		0x8
#define CAD_SPI_SSIEN(x)	((x & 1) >> 0)

#define CAD_SPI_FRF_MOTOROLA	0
#define CAD_SPI_FRF_TEXAS	1
#define CAD_SPI_FRF_NATIONAL	2
#define CAD_SPI_FRF_RESERVED	3

#define CAD_SPI_TMOD_BOTH		0
#define CAD_SPI_TMOD_XMIT		1
#define CAD_SPI_TMOD_RCV		2
#define CAD_SPI_TMOD_EEPROM_READ	3
#define CAD_SPI_TMOD_READ		2
#define CAD_SPI_TMOD_WRITE		1

#define CAD_SPI_BAUDR		0x14
#define CAD_SPI_SCKDV(x)	((x & 0xffff) << 0)

#define CAD_SPI_CTRLR0			0x0
#define CAD_SPI_CTRLR0_CFS(x)		((x & 0xf) << 12)
#define CAD_SPI_CTRLR0_CFS_MSK		(0xf << 12)
#define CAD_SPI_CTRLR0_DFS32(x)		((x & 0x1f) << 16)
#define CAD_SPI_CTRLR0_DFS32_MSK	(0xf << 0)
#define CAD_SPI_CTRLR0_SPI_FRF(x)	((x & 0x3) << 21)
#define CAD_SPI_CTRLR0_SPI_FRF_MSK	(0x3 << 21)
#define CAD_SPI_CTRLR0_FRF(x)		((x & 0x3) << 4)
#define CAD_SPI_CTRLR0_FRF_MSK		(0x3 << 4)
#define CAD_SPI_CTRLR0_SCPH(x)		((x & 1) << 6)
#define CAD_SPI_CTRLR0_SCPH_MSK		(1 << 6)
#define CAD_SPI_CTRLR0_SCPOL(x)		((x & 1) << 7)
#define CAD_SPI_CTRLR0_SCPOL_MSK	(1 << 7)
#define CAD_SPI_CTRLR0_TMOD(x)		((x & 3) << 8)
#define CAD_SPI_CTRLR0_TMOD_MSK		(3 << 8)
#define CAD_SPI_CTRLR0_SRL(x)		((x & 1) << 11)

#define CAD_SPI_CTRLR1		0x4
#define CAD_SPI_CTRLR1_NDF(x)	(((x) & 0xffff) << 0)

#define CAD_SPI_DR		0x60

#define CAD_SPI_RXFLR		0x24
#define CAD_SPI_RXFLR_RXTFL(x)	((x & 0xff) << 0)
#define CAD_SPI_SER		0x10
#define CAD_SPI_TXFLR		0x20
#define CAD_SPI_TXFLR_TXTFL(x)	((x & 0xff) << 0)

#define CAD_SPI_ISR		0x30
#define CAD_SPI_ISR_RXFIS(x)		((x << 4) & 1)
#define CAD_SPI_ISR_TXEIS(x)		((x << 1) & 1)

#define CAD_SPI_TMOD_RESERVED	0x3
#define CAD_SPI_TMOD_RX		~1
#define CAD_SPI_TMOD_TX		~2

uint32_t base = 0x0;


int cad_spim_enabled() {
	return CAD_SPI_SSIEN(mmio_read_32(base + CAD_SPI_SSIENR));
}

void cad_spim_enable(int enable) {
	if (!enable) {
		mmio_write_32(base + CAD_SPI_SSIENR,
			mmio_read_32(base + CAD_SPI_SSIENR) &
			~CAD_SPI_SSIEN(0));
	}
	else {
		mmio_write_32(base + CAD_SPI_SSIENR,
			mmio_read_32(base + CAD_SPI_SSIENR) |
			CAD_SPI_SSIEN(1));
	}
}

void cad_spim_set_data_frame_size(int dfs) {
	mmio_write_32(base + CAD_SPI_CTRLR0, CAD_SPI_CTRLR0_DFS32(dfs) |
		(mmio_read_32(base + CAD_SPI_CTRLR0) & ~CAD_SPI_CTRLR0_DFS32_MSK));
}

int cad_spim_xfer(int size, uint32_t slave_number, int rx_len, uint16_t *rx_buf,
			int tx_len, uint16_t *tx_buf) {
	uint32_t ctrlr0 = mmio_read_32(base + CAD_SPI_CTRLR0);
	int tmod = 0x3;

	if(rx_len > 0)
		tmod &= CAD_SPI_TMOD_RX;

	if(tx_len > 0)
		tmod &= CAD_SPI_TMOD_TX;

	ctrlr0 &= ~CAD_SPI_CTRLR0_TMOD_MSK | CAD_SPI_CTRLR0_TMOD(tmod);

	if(cad_spim_enabled())
		cad_spim_enable(0);

	mmio_write_32(base + CAD_SPI_CTRLR0, ctrlr0);

	mmio_write_32(base + CAD_SPI_SER, 1 << slave_number);

	cad_spim_set_data_frame_size(size);

	cad_spim_enable(1);

	while(tx_len > 0) {
		if(CAD_SPI_TXFLR_TXTFL(mmio_read_32(base +
				CAD_SPI_TXFLR)) < 1) {
			mmio_write_32(base + CAD_SPI_DR, *tx_buf);
			tx_buf++;
			tx_len--;
		}
	}

	if(rx_len > 0) {
		mmio_write_32(base + CAD_SPI_DR, 0);

		mmio_write_32(base + CAD_SPI_CTRLR1,
			CAD_SPI_CTRLR1_NDF(rx_len - 1));
	}

	while(rx_len > 0) {
		if(CAD_SPI_RXFLR_RXTFL(mmio_read_32(base +
				CAD_SPI_RXFLR)) > 0) {
			*rx_buf = mmio_read_32(base + CAD_SPI_DR);
			rx_buf++;
			rx_len--;
		}
	}

	return 0;
}

void cad_spim_set_control_frame_size(int cfs) {
	mmio_write_32(base + CAD_SPI_CTRLR0, CAD_SPI_CTRLR0_CFS(cfs) |
		(mmio_read_32(base + CAD_SPI_CTRLR0) & ~CAD_SPI_CTRLR0_CFS_MSK));
}

void cad_spim_set_spi_frame_format(int format) {
	mmio_write_32(base + CAD_SPI_CTRLR0, CAD_SPI_CTRLR0_SPI_FRF(format) |
		(mmio_read_32(base + CAD_SPI_CTRLR0) &
		~CAD_SPI_CTRLR0_SPI_FRF_MSK));
}

void cad_spim_set_frame_format(int format) {
	mmio_write_32(base + CAD_SPI_CTRLR0, CAD_SPI_CTRLR0_FRF(format) |
		(mmio_read_32(base + CAD_SPI_CTRLR0) & ~CAD_SPI_CTRLR0_FRF_MSK));
}

void cad_spim_set_clk(int scph, int scpol) {
	int ctrl0 = mmio_read_32(base + CAD_SPI_CTRLR0);

	mmio_write_32(base + CAD_SPI_CTRLR0,
		(CAD_SPI_CTRLR0_SCPH(scph) |
			(ctrl0 & ~CAD_SPI_CTRLR0_SCPH_MSK)));

	mmio_write_32(base + CAD_SPI_CTRLR0,
		CAD_SPI_CTRLR0_SCPOL(scpol) |
			(ctrl0 & ~CAD_SPI_CTRLR0_SCPOL_MSK));
}

void cad_spim_set_baud_div(int baudr_div) {
	mmio_write_32(base + CAD_SPI_BAUDR, CAD_SPI_SCKDV(baudr_div));
}

void cad_spim_set_txflr(int txflr) {
	mmio_write_32(base + CAD_SPI_TXFLR, txflr);
}

void cad_spim_set_rxflr(int rxflr) {
	mmio_write_32(base + CAD_SPI_RXFLR, rxflr);
}

void cad_spim_set_srl(int srl) {
	mmio_write_32(base + CAD_SPI_CTRLR0,
		mmio_read_32(base + CAD_SPI_CTRLR0) | CAD_SPI_CTRLR0_SRL(1));
}

void cad_spim_init(uint32_t spi_base, int scph, int scpol) {

	base = spi_base;

	cad_spim_enable(0);
	cad_spim_set_control_frame_size(0);
	cad_spim_set_frame_format(0);
	cad_spim_set_spi_frame_format(3);
	cad_spim_set_clk(scph, scpol);
}

