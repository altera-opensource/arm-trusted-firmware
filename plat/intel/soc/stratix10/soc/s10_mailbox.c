#include <Base.h>
#include <mmio.h>
#include "s10_mailbox.h"

static int fill_mailbox_circular_buffer(uint32_t header_cmd, uint32_t *args,
					int len) {
	uint32_t cmd_free_offset;
	int i;

	cmd_free_offset = mmio_read_32(MBOX_OFFSET + MBOX_COUT);

	if(cmd_free_offset >= MBOX_CMD_BUFFER_SIZE) {
		console_printf("Insufficient buffer in mailbox\n");
		return MBOX_INSUFFICIENT_BUFFER;
	}

	mmio_write_32(MBOX_OFFSET + MBOX_CMD_BUFFER + (cmd_free_offset++ * 4),
			header_cmd);

	for (i = 0; i < len; i++) {
		cmd_free_offset %= MBOX_CMD_BUFFER_SIZE;
		mmio_write_32(MBOX_OFFSET + MBOX_CMD_BUFFER +
				(cmd_free_offset++ * 4), args[i]);
	}

	cmd_free_offset %= MBOX_CMD_BUFFER_SIZE;
	mmio_write_32(MBOX_OFFSET + MBOX_CIN, cmd_free_offset);

	return 0;
}

void mailbox_init(void)
{
	mailbox_set_int(MBOX_INT_FLAG_COE | MBOX_INT_FLAG_RIE);
	mailbox_send_cmd(0, MBOX_CMD_RESTART, 0, 0, 1, 0);
	mailbox_set_int(MBOX_INT_FLAG_COE | MBOX_INT_FLAG_RIE);
}

int mailbox_poll_response(int job_id, int urgent, uint32_t *response) {
	int timeout = 80000;
	int rin = 0;
	int rout = 0;
	int response_length = 0;
	int resp = 0;
	int total_resp_len = 0;

	mmio_write_32(MBOX_OFFSET + MBOX_DOORBELL_TO_SDM, 1);

	while(1) {
		while (timeout > 0 &&
			mmio_read_32(MBOX_OFFSET +
				MBOX_DOORBELL_FROM_SDM) != 1) {
			timeout--;
		}

		if (mmio_read_32(MBOX_OFFSET + MBOX_DOORBELL_FROM_SDM) != 1) {
			console_printf("Timed out waiting for SDM");
			return MBOX_TIMEOUT;
		}

		mmio_write_32(MBOX_OFFSET + MBOX_DOORBELL_FROM_SDM, 0);

		if(urgent) {
			mmio_write_32(MBOX_OFFSET + MBOX_URG, 0);
			if((mmio_read_32(MBOX_OFFSET + MBOX_STATUS) &
				MBOX_STATUS_UA_MASK)) {
				return 0;
			}
			else {
				console_printf("Error: Mailbox did not get UA");
				return -1;
			}
		}

		rin = mmio_read_32(MBOX_OFFSET + MBOX_RIN);
		rout = mmio_read_32(MBOX_OFFSET + MBOX_ROUT);

		if (rout!=rin)
		{
			resp = mmio_read_32(MBOX_OFFSET +
					    MBOX_RESP_BUFFER + ((rout++)*4));

			rout %= MBOX_RESP_BUFFER_SIZE;
			mmio_write_32(MBOX_OFFSET + MBOX_ROUT, rout);

			if(MBOX_RESP_CLIENT_ID(resp) != MBOX_ATF_CLIENT_ID ||
			   MBOX_RESP_JOB_ID(resp) != job_id)
				continue;

			if (MBOX_RESP_ERR(resp) > 0)
			{
				console_printf("Error in response: %x\n",resp);
				return -MBOX_RESP_ERR(resp);
			}
			response_length = MBOX_RESP_LEN(resp);

			while (response_length)
			{

				response_length--;
				resp = mmio_read_32(MBOX_OFFSET +
							MBOX_RESP_BUFFER +
							(rout)*4);
				if (response) {
					*(response + total_resp_len) = resp;
					total_resp_len++;
				}
				rout++;
				rout %= MBOX_RESP_BUFFER_SIZE;
				mmio_write_32(MBOX_OFFSET + MBOX_ROUT,rout);
			}
			return total_resp_len;
		}
	}
}

int mailbox_send_cmd(int job_id, unsigned int cmd, uint32_t *args,
			  int len, int urgent, uint32_t *response)
{
	if(urgent) {
		mmio_write_32(MBOX_OFFSET + MBOX_URG, 1);
	}

	fill_mailbox_circular_buffer(MBOX_CLIENT_ID_CMD(MBOX_ATF_CLIENT_ID) +
					MBOX_JOB_ID_CMD(job_id) +
					MBOX_CMD_LEN_CMD(len) +
					cmd, args, len);

	return mailbox_poll_response(job_id, urgent, response);
}

void mailbox_set_int(int interrupt)
{

	mmio_write_32(MBOX_OFFSET+MBOX_INT, MBOX_COE_BIT(interrupt) |
			MBOX_UAE_BIT(interrupt));
}


void mailbox_set_qspi_open()
{
	mailbox_set_int(MBOX_INT_FLAG_COE | MBOX_INT_FLAG_RIE);
	mailbox_send_cmd(MBOX_JOB_ID, MBOX_CMD_QSPI_OPEN, 0, 0, 0, 0);
}

void mailbox_set_qspi_direct() {
	mailbox_send_cmd(MBOX_JOB_ID, MBOX_CMD_QSPI_DIRECT, 0, 0, 0, 0);
}

void mailbox_set_qspi_close()
{
	mailbox_set_int(MBOX_INT_FLAG_COE | MBOX_INT_FLAG_RIE);
	mailbox_send_cmd(MBOX_JOB_ID, MBOX_CMD_QSPI_CLOSE, 0, 0, 0, 0);
}

int mailbox_get_qspi_clock()
{
	mailbox_set_int(MBOX_INT_FLAG_COE | MBOX_INT_FLAG_RIE);
	return mailbox_send_cmd(MBOX_JOB_ID, MBOX_CMD_QSPI_DIRECT, 0, 0, 0, 0);
}

void mailbox_qspi_set_cs(int device_select) {
	uint32_t cs_setting = device_select;
	cs_setting = (cs_setting << 28); // QSPI device selected settings at 31:28
	mailbox_set_int(MBOX_INT_FLAG_COE | MBOX_INT_FLAG_RIE);
	mailbox_send_cmd(MBOX_JOB_ID, MBOX_CMD_QSPI_SET_CS, &cs_setting, 1, 0, 0);
}

void mailbox_reset_cold() {
	mailbox_set_int(MBOX_INT_FLAG_COE | MBOX_INT_FLAG_RIE);
	mailbox_send_cmd(MBOX_JOB_ID, MBOX_CMD_REBOOT_HPS, 0, 0, 0, 0);
}

