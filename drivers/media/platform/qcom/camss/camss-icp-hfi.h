/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Qualcomm CAMSS ICP HFI Protocol Definitions
 *
 * Copyright (c) 2025 Linaro Ltd.
 */

#ifndef __CAMSS_ICP_HFI_H__
#define __CAMSS_ICP_HFI_H__

#include <linux/types.h>

/* Queue Types */
#define HFI_QUEUE_CMD			1
#define HFI_QUEUE_MSG			2
#define HFI_QUEUE_DBG			3

#define HFI_QTBL_VERSION		0xFFFFFFFF

/* System Commands */
#define HFI_CMD_SYS_INIT		0x10001
#define HFI_CMD_SYS_PC_PREP		0x10002
#define HFI_CMD_SYS_SET_PROPERTY	0x10003
#define HFI_CMD_SYS_PING		0x10005

/* System Messages */
#define HFI_MSG_SYS_INIT_DONE		0x10001
#define HFI_MSG_SYS_PC_PREP_DONE	0x10002
#define HFI_MSG_SYS_PING_ACK		0x10003
#define HFI_MSG_SYS_DEBUG		0x10004
#define HFI_MSG_EVENT_NOTIFY		0x10005

/* IPE/BPS Commands */
#define HFI_CMD_IPEBPS_CREATE_HANDLE		0x20001
#define HFI_CMD_IPEBPS_ASYNC_COMMAND_DIRECT	0x20002
#define HFI_CMD_IPEBPS_ASYNC_COMMAND_INDIRECT	0x20003

/* IPE/BPS Messages */
#define HFI_MSG_IPEBPS_CREATE_HANDLE_ACK	0x20001
#define HFI_MSG_IPEBPS_ASYNC_DIRECT_ACK		0x20002
#define HFI_MSG_IPEBPS_ASYNC_INDIRECT_ACK	0x20003

/* Opcodes */
#define HFI_IPEBPS_CMD_CONFIG_IO		0x01
#define HFI_IPEBPS_CMD_FRAME_PROCESS		0x06
#define HFI_IPEBPS_CMD_ABORT			0x07
#define HFI_IPEBPS_CMD_DESTROY			0x08

/* Device Types */
#define HFI_DEV_TYPE_IPE		3
#define HFI_DEV_TYPE_BPS		4

/* Events */
#define HFI_EVENT_SYS_ERROR		0x01

/* Property Types */
#define HFI_PROP_SYS_UBWC_CONFIG	0x01

/* Memory Sizes */
#define HFI_QTBL_SIZE			SZ_1M
#define HFI_CMD_Q_SIZE			SZ_1M
#define HFI_MSG_Q_SIZE			SZ_1M
#define HFI_DBG_Q_SIZE			SZ_1M
#define HFI_SFR_SIZE			SZ_8K
#define HFI_SHMEM_SIZE			SZ_8M

#define HFI_CMD_Q_DATA_SIZE		4096
#define HFI_MSG_Q_DATA_SIZE		4096
#define HFI_DBG_Q_DATA_SIZE		102400

/*
 * HFI Queue Header - CAMX compatible
 * Each field is padded to 64 bytes (16 u32s) for cache line alignment.
 * The firmware expects this exact layout.
 */
struct hfi_queue_header {
	u32 dummy0[15];
	u32 status;
	u32 dummy1[15];
	u32 start_addr;
	u32 dummy2[15];
	u32 type;
	u32 dummy3[15];
	u32 q_size;
	u32 dummy4[15];
	u32 pkt_size;
	u32 dummy5[15];
	u32 pkt_drop_cnt;
	u32 dummy6[15];
	u32 rx_wm;
	u32 dummy7[15];
	u32 tx_wm;
	u32 dummy8[15];
	u32 rx_req;
	u32 dummy9[15];
	u32 tx_req;
	u32 dummy10[15];
	u32 rx_irq_status;
	u32 dummy11[15];
	u32 tx_irq_status;
	u32 dummy12[15];
	u32 read_idx;
	u32 dummy13[15];
	u32 write_idx;
	u32 dummy14[16];  /* 16 elements to pad structure to 960 bytes */
};

struct hfi_queue_table {
	u32 version;
	u32 size;
	u32 qhdr0_offset;
	u32 qhdr_size;
	u32 num_queues;
	u32 num_active;
	struct hfi_queue_header q[];
};

struct hfi_pkt_hdr {
	u32 size;
	u32 pkt_type;
} __packed;

struct hfi_msg_init_done {
	u32 size;
	u32 pkt_type;
	u32 error;
} __packed;

struct hfi_cmd_ping {
	u32 size;
	u32 pkt_type;
	u64 user_data;
} __packed;

struct hfi_msg_ping_ack {
	u32 size;
	u32 pkt_type;
	u64 user_data;
} __packed;

struct hfi_msg_event {
	u32 size;
	u32 pkt_type;
	u32 session_id;
	u32 event_id;
	u32 data1;
	u32 data2;
} __packed;

struct hfi_cmd_ubwc_cfg {
	u32 size;
	u32 pkt_type;
	u32 num_params;
	u32 prop_type;
	u32 ipe_fetch;
	u32 ipe_write;
	u32 bps_fetch;
	u32 bps_write;
} __packed;

struct hfi_cmd_create_handle {
	u32 size;
	u32 pkt_type;
	u32 dev_type;
	u32 user_data;
} __packed;

struct hfi_msg_create_handle_ack {
	u32 size;
	u32 pkt_type;
	u32 error;
	u32 fw_handle;
} __packed;

struct hfi_cmd_async {
	u32 size;
	u32 pkt_type;
	u32 opcode;
	u32 num_handles;
	u32 fw_handle;
	u64 user_data1;
	u64 user_data2;
	u32 payload[];
} __packed;

struct hfi_msg_async_ack {
	u32 size;
	u32 pkt_type;
	u32 opcode;
	u64 user_data1;
	u64 user_data2;
	u32 error;
} __packed;

static inline u32 hfi_pkt_size(void *pkt)
{
	return ((u32 *)pkt)[0];
}

static inline u32 hfi_pkt_type(void *pkt)
{
	return ((u32 *)pkt)[1];
}

static inline bool hfi_queue_empty(struct hfi_queue_header *q)
{
	return READ_ONCE(q->read_idx) == READ_ONCE(q->write_idx);
}

static inline u32 hfi_queue_free(struct hfi_queue_header *q)
{
	u32 ri = READ_ONCE(q->read_idx);
	u32 wi = READ_ONCE(q->write_idx);
	u32 used = (wi >= ri) ? (wi - ri) : (q->q_size - ri + wi);

	return q->q_size - used - 1;
}

#endif /* __CAMSS_ICP_HFI_H__ */
