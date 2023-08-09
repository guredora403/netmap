#define BILLION				1000000000
#define MAX_IFNAMELEN		64	/* our buffer for ifname */
#define DEFAULT_TIMEOUT_MS	50
#define TIMEOUT_LIMITS		20

/* Query sub command */
#define SIGMIXERCHG	90	/* signal number for extra command (hardware volume control) */
#define MODEL_SPEC	0	/* Model Spec Request */

#define timespec_diff_ns(from, to)	(BILLION * (to.tv_sec - from.tv_sec) + to.tv_nsec - from.tv_nsec)
#define timespec_to_ns(ts)	(uint64_t)ts.tv_sec * 1000000000 + ts.tv_nsec

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

#ifdef LOG
#define P(_fmt, ...)							\
	do {										\
		struct timespec __ts;					\
		clock_gettime(CLOCK_MONOTONIC, &__ts);	\
		fprintf(stderr, "%06ld.%03ld %-9s [%4d] %5u| " _fmt "\n",	\
		    __ts.tv_sec, __ts.tv_nsec/1000000,	\
		    __FUNCTION__, __LINE__, g.key, ##__VA_ARGS__);	\
        } while (0)
#else
#define P(_fmt, ...)	do {} while (0)
#endif
/*
#include <ctype.h>
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
