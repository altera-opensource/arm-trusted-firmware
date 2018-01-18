#ifndef __MBOX__
#define __MBOX__

#define MBOX_OFFSET		0xffa30000

#define MBOX_ATF_CLIENT_ID 	0x1
#define MBOX_JOB_ID 		0x1

/* Mailbox interrupt flags and masks */
#define MBOX_INT_FLAG_COE	0x1
#define MBOX_INT_FLAG_RIE	0x2
#define MBOX_INT_FLAG_UAE	0x100
#define MBOX_COE_BIT(INTERRUPT)	(INTERRUPT & 0x3)
#define MBOX_UAE_BIT(INTERRUPT)	((INTERRUPT & (1<<4)))

/* Mailbox response and status */
#define MBOX_RESP_BUFFER_SIZE 	16
#define MBOX_RESP_ERR(BUFFER) 	(BUFFER & 0x00000fff)
#define MBOX_RESP_LEN(BUFFER) 	((BUFFER & 0x007ff000) >> 12)
#define MBOX_RESP_CLIENT_ID(BUFFER)	((BUFFER & 0xf0000000) >> 28)
#define MBOX_RESP_JOB_ID(BUFFER) 	((BUFFER & 0x0f000000) >> 24)
#define MBOX_STATUS_UA_MASK	(1<<8)

/* Mailbox command and response */
#define MBOX_CMD_FREE_OFFSET	0x14
#define MBOX_CMD_BUFFER_SIZE	32
#define MBOX_CLIENT_ID_CMD(CLIENT_ID)	(CLIENT_ID<<28)
#define MBOX_JOB_ID_CMD(JOB_ID)	(JOB_ID<<24)
#define MBOX_CMD_LEN_CMD(CMD_LEN)	(CMD_LEN<<12)
#define MBOX_INSUFFICIENT_BUFFER	-1
#define MBOX_CIN			0x00
#define MBOX_ROUT			0x04
#define MBOX_URG			0x08
#define MBOX_INT			0x0C
#define MBOX_COUT			0x20
#define MBOX_RIN			0x24
#define MBOX_STATUS			0x2C
#define MBOX_CMD_BUFFER			0x40
#define MBOX_RESP_BUFFER		0xC0

#define MBOX_RESP_BUFFER_SIZE		16
#define MBOX_RESP_OK			0
#define MBOX_RESP_INVALID_CMD		1
#define MBOX_RESP_UNKNOWN_BR		2
#define MBOX_RESP_UNKNOWN		3
#define MBOX_RESP_NOT_CONFIGURED 	256

/* Mailbox SDM doorbell */
#define MBOX_DOORBELL_TO_SDM		0x400
#define MBOX_DOORBELL_FROM_SDM		0x480

/* Mailbox QSPI commands */
#define MBOX_CMD_RESTART 		2
#define MBOX_CMD_QSPI_OPEN 		50
#define MBOX_CMD_QSPI_CLOSE 		51
#define MBOX_CMD_QSPI_DIRECT 		59
#define MBOX_CMD_GET_IDCODE		16
#define MBOX_CMD_QSPI_SET_CS		52

/* Mailbox REBOOT commands */
#define MBOX_CMD_REBOOT_HPS		71

/* Generic error handling */
#define MBOX_TIMEOUT			-1

void mailbox_set_int(int interrupt_input);
void mailbox_init(void);
void mailbox_set_qspi_close();
void mailbox_set_qspi_open();
void mailbox_set_qspi_direct();
int mailbox_send_cmd(int job_id, unsigned int cmd, uint32_t *args,
				int len, int urgent, uint32_t *response);
int mailbox_get_qspi_clock();
void mailbox_reset_cold(void);
#endif
