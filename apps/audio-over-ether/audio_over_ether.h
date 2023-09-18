/* circular buffer (The buffer size must be limited to a power of two.) */
#define CIRC_INDEX(new_index, buffer_size) ((new_index) & ((buffer_size) - 1))
#define CIRC_CNT(head, tail, buffer_size) (((head) - (tail)) & ((buffer_size) - 1))
#define CIRC_SPACE(head, tail, buffer_size) (((tail) - (head) - 1) & ((buffer_size) - 1))

/*
 * print ether_addr macro
 * ex) struct ether_addr mac = {{0xf1, 0xf2, 0xf3, 0xf4, 0xf5, 0xf6}};
 *     printf("mac address is " ETHER_ADDR_FMT "\n", ETHER_ADDR_PTR(&mac));
 */
#define ETHER_ADDR_FMT "%02X:%02X:%02X:%02X:%02X:%02X"
#define ETHER_ADDR_PTR(addr) (addr)->ether_addr_octet[0], (addr)->ether_addr_octet[1], (addr)->ether_addr_octet[2], (addr)->ether_addr_octet[3], (addr)->ether_addr_octet[4], (addr)->ether_addr_octet[5]

/* IOCTL */
#define IOCTL_PCM_START	_IOW('a', 0, int)

/* Netmap, AoE */
#define NM_BUFSZ			2048
#define NM_NUM_SLOTS		1024
#define AOE_NUM_SLOTS		2048
#define UNIT_BUF_SIZE		(NM_BUFSZ*NM_NUM_SLOTS)
#define AOE_BUF_SIZE		(NM_BUFSZ*NM_NUM_SLOTS*4 + sizeof(struct ext_slot))
#define HW_RXRING_IDX0		2
#define HOST_TXRING_IDX0	7170
#define AOE_LUT_INDEX		100000

/* netmap buf idx to aoebuf idx macro
 *  netmap    aoebuf
 *       2 ->      0 [HW_RXRING_IDX0]
 *    1025 ->   1023
 *    7170 ->   1024 [HOST_TXRING_IDX0]
 *    8193 ->   2047
 *   10000 ->   2048 [AOE_LUT_INDEX]
 *   11023 ->   3071
 */
#define AOE_BUF_IDX(idx)	(idx < AOE_LUT_INDEX ? (idx - HW_RXRING_IDX0)%2048 : idx + NM_NUM_SLOTS * 2 - AOE_LUT_INDEX)

/*
 * The AoE Sound Driver uses the size of the ether header
 * or aoe header to calculate the memory address.
 */
#ifdef CONFIG_SMPD_OPTION_AOE
#define ETH_ALEN       6
struct ether_header
{
  uint8_t  ether_dhost[ETH_ALEN];	/* destination eth addr	*/
  uint8_t  ether_shost[ETH_ALEN];	/* source ether addr	*/
  uint16_t ether_type;		        /* packet type ID field	*/
} __attribute__ ((__packed__));
#endif

#define ETHERTYPE_LE2		0x88b6	/* IEEE Std 802 - Local Experimental Ethertype 2. */
#define MTU_ETH				1500
#define AOE_MAX_PAYLEN		MTU_ETH - sizeof(struct aoe_header)
#define AOE_MIN_PAYLEN		46 - sizeof(struct aoe_header)
struct aoe_header {
	uint8_t  u8key;		/* session key */
	uint8_t  u8cmd;		/* command */
	uint8_t  u8sub;		/* sub command */
	uint8_t  u8opt;		/* option */
	uint32_t u32val;	/* value */
} __attribute__ ((__packed__));

struct aoe_packet {
	struct ether_header eh;
	struct aoe_header aoe;
	uint8_t body[AOE_MAX_PAYLEN];
} __attribute__((__packed__));

/* shared data */
struct slot {
	uint32_t buf_idx;	/* buffer index */
};
enum DMAStatus	{ACTIVE, INACTIVE};
struct ext_slot {
	struct slot slot[AOE_NUM_SLOTS];
	int head;
	int cur;
	int tail;
	int unarrived_dreq_packets;
	int num_slots;
	enum DMAStatus stat;
};
