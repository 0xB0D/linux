/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Qualcomm CAMSS ICP HFI Protocol Definitions
 *
 * Copyright (c) 2025 Linaro Ltd.
 */

#ifndef __CAMSS_ICP_HFI_H__
#define __CAMSS_ICP_HFI_H__

#include <linux/types.h>

#define HFI_QTBL_VERSION			0xFFFFFFFF

/* ICP firmware boot command/response */
#define ICP_INIT_REQUEST_RESET			0x0
#define ICP_INIT_REQUEST_SET			0x01

#define ICP_INIT_RESP_RESET			0x00
#define ICP_INIT_RESP_SUCCESS			0x01
#define	ICP_INIT_RESP_FAILED			0x02

/* Command / Response groups */
#define HFI_CMD_GRP_ICP				0
#define HFI_CMD_GRP_IPE_BPS			BIT(24)
#define HFI_CMD_GRP_CDM				BIT(25)
#define HFI_CMD_GRP_DBG				(BIT(24) | BIT(25))

/* Command / Response offsets */
#define HFI_CMD_BASE				BIT(16)
#define HFI_RSP_BASE				BIT(17)

/* System Commands */
#define HFI_CMD_SYS_INIT			0x10001
#define HFI_CMD_SYS_PC_PREP			0x10002
#define HFI_CMD_SYS_SET_PROPERTY		0x10003
#define HFI_CMD_SYS_GET_PROPERTY		0x10004
#define HFI_CMD_SYS_PING			0x10005
#define HFI_CMD_SYS_RESET			0x10006

/* ICP Firmware result codes */
#define CAMERAICP_OK				0x00
#define CAMERAICP_EFAILED			0x01
#define CAMERAICP_ENOMEMORY			0x02
#define CAMERAICP_EBADSTATE			0x03
#define CAMERAICP_EBADPARM			0x04
#define CAMERAICP_EBADITEM			0x05
#define CAMERAICP_EINVALIDFORMAT		0x06
#define CAMERAICP_EUNSUPPORTED			0x07
#define CAMERAICP_EOUTOFBOUND			0x08
#define CAMERAICP_ETIMEDOUT			0x09
#define CAMERAICP_EABORTED			0x0a
#define CAMERAICP_EHWVIOLATION			0x0b
#define CAMERAICP_ECDMERROR			0x0c

/* System Messages */
#define HFI_MSG_ICP_COMMON_START		0x20000
#define HFI_MSG_SYS_INIT_DONE			0x20001
#define HFI_MSG_SYS_PC_PREP_DONE		0x20002
#define HFI_MSG_SYS_DEBUG			0x20003
#define HFI_MSG_SYS_IDLE			0x20004
#define HFI_MSG_SYS_PROPERTY_INFO		0x20005
#define HFI_MSG_SYS_PING_ACK			0x20006
#define HFI_MSG_SYS_RESET_ACK			0x20007
#define HFI_MSG_EVENT_NOTIFY			0x20008

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
#define HFI_DEV_TYPE_IPE			3
#define HFI_DEV_TYPE_BPS			4

/* Events */
#define HFI_EVENT_SYS_ERROR			0x01
#define HFI_EVENT_ICP_ERROR			0x02
#define HFI_EVENT_IPE_BPS_ERROR			0x03
#define HFI_EVENT_CDM_ERROR			0x04
#define HFI_EVENT_DBG_ERROR			0x05

/* Property Types */
#define HFI_PROP_SYS_DEBUG_CFG			0x01
#define HFI_PROP_SYS_UBWC_CFG			0x02
#define HFI_PROP_SYS_IMAGE_VER			0x03
#define HFI_PROP_SYS_SUPPORTED			0x04
#define HFI_PROP_SYS_IPEBPS_PC			0x05

/*
 * Debug levels
 */
#define HFI_DEBUG_MSG_LOW			BIT(0)
#define HFI_DEBUG_MSG_MEDIUM			BIT(1)
#define HFI_DEBUG_MSG_HIGH			BIT(2)
#define HFI_DEBUG_MSG_ERROR			BIT(3)
#define HFI_DEBUG_MSG_FATAL			BIT(4)
#define HFI_DEBUG_MSG_PERF			BIT(5)

/*
 * Debug output modes
 */
#define HFI_DEBUG_MODE_QUEUE			0x00000001
#define HFI_DEBUG_MODE_QDSS			0x00000002

/*
 * Handle types for CREATE_HANDLE
 */
#define HFI_HANDLE_TYPE_BPS			1
#define HFI_HANDLE_TYPE_IPE			2

/*
 * Queue indexes
 */
enum {
	HFI_Q_CMD_TYPE = 0,
	HFI_Q_MSG_TYPE,
	HFI_Q_DBG_TYPE,
	HFI_Q_MAX,
};

/* Memory Sizes */
#define HFI_QTBL_SIZE				SZ_1M
#define HFI_CMD_Q_SIZE				SZ_1M
#define HFI_MSG_Q_SIZE				SZ_1M
#define HFI_DBG_Q_SIZE				SZ_1M
#define HFI_SFR_LOG_SIZE			SZ_4K

#define HFI_CMD_Q_DATA_SIZE			SZ_4K
#define HFI_MSG_Q_DATA_SIZE			SZ_4K
#define HFI_DBG_Q_DATA_SIZE			102400

#define HFI_SHMEM_SIZE				SZ_1M
#define HFI_FWUNCACHED_SIZE			(7 * SZ_1M)
#define HFI_QDSS_SIZE				SZ_1M
#define HFI_QTBL_SIZE				SZ_1M
#define HFI_Q_SIZE				SZ_1M
#define HFI_SFR_SIZE				SZ_8K
#define HFI_SECHEAP_SIZE			SZ_1M

/*
 * General Purpose registers
 *
 * GP registers start at CSR base + 0x20. CAMX defines them as:
 *   GEN_PURPOSE_REG(n) = n * 4, relative to (CSR + 0x20)
 *
 * So GP0 = CSR+0x20, GP1 = CSR+0x24, GP2 = CSR+0x28, GP3 = CSR+0x2C, etc.
 *
 * Offsets below are absolute from CSR base for direct use with csr_base.
 */
#define HFI_REG_FW_VERSION			0x24  /* GP1 - firmware writes version */
#define HFI_REG_HOST_ICP_INIT_REQ		0x28  /* GP2 - host signals init request */
#define HFI_REG_ICP_HOST_INIT_RESP		0x2C  /* GP3 - firmware signals ready */
#define HFI_REG_SHARED_MEM_PTR			0x30  /* GP4 - shared memory IOVA */
#define HFI_REG_SHARED_MEM_SIZE			0x34  /* GP5 - shared memory size */
#define HFI_REG_QTBL_PTR			0x38  /* GP6 - q table IOVA */
#define HFI_REG_SECONDARY_HEAP_PTR		0x3C  /* GP7 - secondary heap IOVA */
#define HFI_REG_SECONDARY_HEAP_SIZE		0x40  /* GP8 - secondary heap size */
#define HFI_REG_RESERVED			0x44  /* GP9 - reserved/status */
#define HFI_REG_SFR_PTR				0x48  /* GP10 - SFR buffer IOVA */
#define HFI_REG_QDSS_IOVA			0x4C  /* GP11 - QDSS buffer IOVA */
#define HFI_REG_QDSS_IOVA_SIZE			0x50  /* GP12 - QDSS buffer size */
#define HFI_REG_IO_REGION_1_IOVA		0x54  /* GP13 - IO region 1 IOVA */
#define HFI_REG_IO_REGION_1_SIZE		0x58  /* GP14 - IO region 1 size */
#define HFI_REG_IO_REGION_2_IOVA		0x5C  /* GP15 - IO region 2 IOVA */
#define HFI_REG_IO_REGION_2_SIZE		0x60  /* GP16 - IO region 2 size */
#define HFI_REG_FWUNCACHED_IOVA			0x64  /* GP17 - FW uncached region IOVA */
#define HFI_REG_FWUNCACHED_SIZE			0x68  /* GP18 - FW uncached region size */

/* HFI constants */
#define HFI_QUEUE_TABLE_VERSION			0xFFFFFFFF

struct hfi_resources {
	u32 shmem_size;
	u32 qdss_size;
	u32 fwuncached_size;
	u32 secheap_size;
	u32 q_tbl_size;
	u32 qdata_size[HFI_Q_MAX];
	u32 sfr_size;
};

struct icp_hfi_mem_region {
	void *vaddr;
	dma_addr_t dma_addr;
	size_t size;
};

struct icp_hfi_mem {
	/* These get their own allocations */
	struct icp_hfi_mem_region shmem;
	struct icp_hfi_mem_region qdss;
	struct icp_hfi_mem_region fwuncached;

	/* Pointers into the fwuncached region */
	struct icp_hfi_mem_region secheap;
	struct icp_hfi_mem_region q_tbl;
	struct icp_hfi_mem_region q_data[HFI_Q_MAX];
	struct icp_hfi_mem_region sfr;
};

struct icp_hfi_ops {
	void (*raise_irq)(void *priv);
};

struct icp_hfi {
	struct device *dev;
	struct icp_hfi_mem hfi_mem;
	const struct hfi_resources *res;

	const struct icp_hfi_ops *ops;
	void *priv;

	/*
	 * Queue Table Pointer
	 * Points to hfi_q_tbl_hdr at hfi_mem.q_tbl.vaddr
	 * Contains header + 4 x struct hfi_q_header
	 */
	struct hfi_q_tbl_hdr *q_tbl;

	/*
	* Synchronization
	*/
	struct mutex cmd_lock;
	struct completion cmd_complete;

	/*
	 * State
	 */
	bool ready;         /* HFI fully initialized */
	u32 fw_version;     /* From GP1 register after boot */
	u32 api_version;    /* From INIT_DONE response */

	/* Debug */
	struct hfi_sfr *sfr;
};

/*
 * HFI Queue Header - CAMX compatible
 * Each field is padded to 64 bytes (16 u32s) for cache line alignment.
 * The firmware expects this exact layout.
 */
struct hfi_q_hdr {
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
	u32 dummy14[15];
} __packed;

struct hfi_q_tbl_hdr {
	u32 version;
	u32 size;
	u32 q_hdr0_offset;
	u32 q_hdr_size;
	u32 num_queues;
	u32 num_active_queues;
	struct hfi_q_hdr q_hdr[];
} __packed;

struct hfi_pkt_hdr {
	u32 size;
	u32 pkt_type;
} __packed;

struct hfi_cmd_sys_init {
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
	u32 handle_type;
	u32 user_data0;
	u32 user_data1;
} __packed;

struct hfi_msg_create_handle_ack {
	u32 size;
	u32 pkt_type;
	u32 error;
	u32 fw_handle;
	u32 user_data0;
	u32 user_data1;
} __packed;

struct hfi_cmd_async {
	u32 size;
	u32 pkt_type;
	u32 opcode;
	u32 num_handles;
	u32 fw_handle;
	u64 user_data0;
	u64 user_data1;
	u32 payload[];
} __packed;

struct hfi_msg_async_ack {
	u32 size;
	u32 pkt_type;
	u32 opcode;
	u64 user_data0;
	u64 user_data1;
	u32 error;
} __packed;

struct hfi_cmd_set_property {
	u32 size;
	u32 pkt_type;
	u32 num_prop;
	u32 prop_data[];
} __packed;

struct hfi_msg_debug {
	u32 size;
	u32 pkt_type;
	u32 msg_type;
	u32 msg_size;
	u32 timestamp_hi;
	u32 timestamp_lo;
	u8  msg_data[];
} __packed;

struct hfi_debug_cfg {
	u32 debug_config;
	u32 debug_mode;
} __packed;

struct hfi_msg_debug_level {
	struct hfi_cmd_set_property hdr;
	u32 prop_id;
	struct hfi_debug_cfg cfg;
} __packed;

struct hfi_sfr {
	u32 size;
	char msg[HFI_SFR_LOG_SIZE];
};

static inline u32 hfi_pkt_size(void *pkt)
{
	return ((u32 *)pkt)[0];
}

static inline u32 hfi_pkt_type(void *pkt)
{
	return ((u32 *)pkt)[1];
}

static inline bool hfi_queue_empty(struct hfi_q_hdr *q)
{
	return READ_ONCE(q->read_idx) == READ_ONCE(q->write_idx);
}

static inline u32 hfi_queue_free(struct hfi_q_hdr *q)
{
	u32 ri = READ_ONCE(q->read_idx);
	u32 wi = READ_ONCE(q->write_idx);
	u32 used = (wi >= ri) ? (wi - ri) : (q->q_size - ri + wi);

	return q->q_size - used - 1;
}

/* Queue management */
int icp_hfi_init_queues(struct icp_hfi *hfi);
void icp_hfi_deinit_queues(struct icp_hfi *hfi);

/* ISR support - called from threaded IRQ handler */
void icp_hfi_flush_debug_queue(struct icp_hfi *hfi);
bool icp_hfi_process_msg_queue(struct icp_hfi *hfi);

/* Core operations */
int icp_hfi_core_init(struct icp_hfi *hfi);
void icp_hfi_dump_sfr(struct icp_hfi *hfi);

#endif /* __CAMSS_ICP_HFI_H__ */
