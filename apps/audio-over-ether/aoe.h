#define ETHERTYPE_LE2	0x88b6	/* IEEE Std 802 - Local Experimental Ethertype 2. */
#define MTU_ETH		1500
#define MAX_BODYSIZE	MTU_ETH - sizeof(struct ether_header) - sizeof(struct aoe_header)

#define AOE_MIN_PAYLEN	60 - sizeof(struct ether_header) - sizeof(struct aoe_header)

enum Mode	{SERVER, CLIENT};
enum Status	{CONNECTED, CLOSED, SYN, TEARDOWN, EXIT};
#if defined(LOG) || defined(CLI)
static const char Connects[5][10] = {
	"CONNECTED", "CLOSED", "SYN", "TEARDOWN", "EXIT"
};
#endif
enum command{
	C_DISCOVER,	// Neighbor Discovery Broadcast
	C_SYN,		// Synchronize
	C_ACK,		// Acknowledge
	C_DREQ,		// Data Request
	C_DRES,		// Data Response
	C_QUERY,	// Query
	C_REPORT,	// Report
	C_CLOSE		// Close
};

struct aoe_packet {
	struct ether_header eh;
	struct aoe_header aoe;
	uint8_t body[MAX_BODYSIZE];	/* hardwired */
} __attribute__((__packed__));

#ifdef CSUM
static uint32_t checksum(const void *data, uint16_t len, uint32_t sum)
{
	const uint8_t *addr = data;
	uint32_t i;
	for (i = 0; i < (len & ~1U); i += 2) {
		sum += (uint16_t)ntohs(*((uint16_t *)(addr + i)));
		if (sum > 0xFFFF)
			sum -= 0xFFFF;
	}
	if (i < len) {
		sum += addr[i] << 8;
		if (sum > 0xFFFF)
			sum -= 0xFFFF;
	}
	return sum;
}

static uint16_t wrapsum(uint32_t sum)
{
	sum = ~sum & 0xFFFF;
	return (htons(sum));
}

static __inline uint16_t cksum_add(uint16_t sum, uint16_t a)
{
	uint16_t res = sum + a;
	return (res + (res < a));
}
#endif

/*
static void hexdump(const char *_p, int len)
{
	char buf[128];
	int i, j, i0;
	const unsigned char *p = (const unsigned char *)_p;
	for (i = 0; i < len; ) {
		memset(buf, ' ', sizeof(buf));
		sprintf(buf, "%5d: ", i);
		i0 = i;
		for (j=0; j < 16 && i < len; i++, j++)
			sprintf(buf+7+j*3, "%02x ", (uint8_t)(p[i]));
		i = i0;
		for (j=0; j < 16 && i < len; i++, j++)
			sprintf(buf+7+j + 48, "%c",
				isprint(p[i]) ? p[i] : '.');
		printf("%s\n", buf);
	}
}
*/
