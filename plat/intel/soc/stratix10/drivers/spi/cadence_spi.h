int cad_spim_init(uint32_t spi_base, int scph, int scpol);
void cad_spim_set_baud_div(int bauddiv);
int cad_spim_xfer(int size, int slave, int rx_len, uint16_t *rx_buf, int tx_len,
			uint16_t *tx_buf);
int cad_spim_set_srl(int enable);

