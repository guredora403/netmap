#define NETMAP_WITH_LIBS
#include <time.h>
#include <limits.h>
#include <libnetmap.h>
#include <ifaddrs.h>		/* getifaddrs */
#include <signal.h>
#include <errno.h>
#include <stdbool.h>
#include <sys/poll.h>
#include <sys/ioctl.h>
#include <arpa/inet.h>		/* ntohs */
#include <linux/if_packet.h>    /* sockaddr_ll */
#include <netinet/ether.h>
#include <alsa/asoundlib.h>
#include "audio_over_ether.h"
#include "vsound.h"
#include "aoe.h"

#define BILLION		1000000000
#define MAX_IFNAMELEN	64	/* our buffer for ifname */
#define MAX_ARGLEN	64

#define CAP		64
#define BROADCAST_MS	200
#define SYN_TIMEOUT_MS	1000
#define DEFAULT_TIMEOUT_MS	50
#define BASE_TIMEOUT_MS		s.pcm.period_us * AOE_NUM_SLOTS/1000
#define DREQ_TIMEOUT_MS		(BASE_TIMEOUT_MS * 3/4)
#define TIMEOUT_LIMITS	20

#define REMOTE_COMMAND	"/usr/local/bin/aoecmd.sh"
#define SIGMIXERCHG	90	/* signal number for extra command (hardware volume control) */

#define timespec_diff_ns(from, to)	(BILLION * (to.tv_sec - from.tv_sec) + to.tv_nsec - from.tv_nsec)

#ifdef SVR
static struct ext_slot *ext;
#endif

static void usage(int errcode)
{
#ifdef SVR
	const char *cmd = "aoebridge";
#endif
#ifdef CLI
	const char *cmd = "aoe";
#endif
	fprintf(stdout,
		"%s %s\n"
		"Usage:\n"
		"  -h             Show program usage and exit.\n"
		"  -i INTERFACE   (Optional)Network interface name. (Default eth0)\n"
		"\n"
#ifdef CLI
		"Client Option\n"
		"  -d MAC_ADDR    (Optional)Destination MAC address.\n"
#endif
#ifdef SVR
		"Server Option\n"
		"  -D DEVICE_NAME (Optional)select PCM by name. (Default hw:0,0)\n"
//		"  -d DREQ_LIMITS (Optional)Performance tuning parameter.(1-64)\n"
//		"  -r RECV_LIMITS (Optional)Performance tuning parameter.(1-64)\n"
#ifdef CSUM
		"  -s             (Optional)Enable checksums.\n"
#endif
#endif
		, cmd, AOE_VERSION);
	exit(errcode);
}

struct targ {
	enum Mode mode;
	enum Status stat;
	uint16_t key;
	int poll_timeout_ms;
	int options;
#define OPT_AUTO	1
#define OPT_FORWARD	2
#define OPT_CSUM	4
#define OPT_FIXED	8
	char *aoe_buf_addr;
	struct nmport_d *nmd;
	int aoe_buf_fd;
	int read_fd;
	char ifname[MAX_IFNAMELEN];
	char pcm_name[MAX_ARGLEN];
};
static struct targ g;

struct event_count {
	unsigned long error;
	unsigned long timeout;
	unsigned long recover;
};
struct stats {
	unsigned long min;
	unsigned long max;
	unsigned long ttl;
	unsigned long count;
};
#ifdef SVR
static char report[1500];
struct server_stats {
	int chunk_bytes;
	int cb_frames;
	snd_pcm_uframes_t period_size;
	int req;
	int recv;
	int dreq_limits;
	int recv_limits;
	int last_volume_value;
	struct vsound_pcm pcm;
	struct netmap_slot * cur_slot;
	struct timespec cur_ts;
	struct timespec last_ts;
	struct event_count ev;
	struct stats poll_stats;
#ifdef SERVER_STATS
	struct stats trip_stats[CAP];
	unsigned long recv_total;
	unsigned long forward_total;
	unsigned long poll_total;
	unsigned long req_count;
	unsigned long trip_us_total;
	unsigned long act_ns_total;
	int poll_count;
#endif
};
static struct server_stats s;

static void pcm_rate_changed(struct server_stats *, snd_pcm_format_t, unsigned int, unsigned int);
static void detect_initial_params(struct server_stats *);
static void pcm_ready(struct server_stats *);
static void pcm_exit(void);

#endif
#ifdef CLI
//static char audiobuf[1500];
struct client_stats {
	int chunk_bytes;
	int sig;
	struct timespec cur_ts;
	struct timespec last_ts;
	struct vsound_pcm pcm;
	uint32_t last_mixer_value;
	unsigned long readtimeout_count;
	unsigned long eof_count;
	unsigned long eintr_count;
};
static struct client_stats c;
#endif
#ifdef SVR
static void reset(struct server_stats *_s)
{
#ifdef SERVER_STATS
	_s->poll_count = 0;
#endif
	_s->req = 0;
	_s->recv = 0;
	ext->unarrived_dreq_packets = 0;
}
#endif
#ifdef SERVER_STATS
static void calc_stats(struct stats *t, unsigned long trip)
{
	if (t->min >= trip) {
		t->min = trip;
	} else if (t->max < trip) {
		t->max = trip;
	}
	t->ttl += trip;
	t->count++;
}
#define incr(x)	do {x += 1;} while (0)
#endif

#ifdef LOG
#define P(_fmt, ...)							\
	do {								\
		struct timeval _t0;					\
		gettimeofday(&_t0, NULL);				\
		fprintf(stderr, "%03d.%06d %-9s [%4d] %5u| " _fmt "\n",	\
		    (int)(_t0.tv_sec % 1000), (int)_t0.tv_usec,		\
		    __FUNCTION__, __LINE__, g.key, ##__VA_ARGS__);	\
        } while (0)
#else
#define P(_fmt, ...)	do {} while (0)
#endif

#ifdef CLI
#define READ_SLEEP_US	1000000 * 10 / c.pcm.rate
#define MAX_LOOP	2500
static int read_stats(void)
{
	return read(g.read_fd, &c.pcm, 0);
}
static int read_buf(int fd, char *buf, int count)
{
	int loop = MAX_LOOP, l = 0, r;
	do {
		r = read(fd, buf + l, count - l);
		if (likely(r > 0)) {
			l += r;
			loop = MAX_LOOP;
			continue;
		} else if (r == 0) {
			c.eof_count++;
			if (l > 0) {
				/* drain */
				memset(buf + l, '0', count - l);
			}
			return l;
		} else {
			if (errno == EINTR) {
				c.eintr_count++;
				continue;
			} else if (errno == EAGAIN) {
				if (--loop > 0) {
					usleep(READ_SLEEP_US);
					continue;
				}
			}
			c.readtimeout_count++;
			if (l > 0) {
				/* drain */
				memset(buf + l, '0', count - l);
				return l;
			}
			return -ENODATA;
		}
	} while (l < count);

	return l;
}
#endif

static void source_hwaddr(const char *ifname, struct ether_addr *host)
{
	struct ifaddrs *ifaphead, *ifap;
	char *pp;
	pp = (char *)ifname;
	if (getifaddrs(&ifaphead) != 0) {
		D("getifaddrs %s failed", ifname);
		usage(-1);
	}

	for (ifap = ifaphead; ifap; ifap = ifap->ifa_next) {
		struct sockaddr_ll *sdl =
			(struct sockaddr_ll *)ifap->ifa_addr;
		if (!sdl || sdl->sll_family != AF_PACKET)
			continue;
		if (strncmp(ifap->ifa_name, pp, IFNAMSIZ) != 0)
			continue;
		memcpy(&host->ether_addr_octet, sdl->sll_addr, sizeof(host->ether_addr_octet));
		break;
	}
	freeifaddrs(ifaphead);
}

#define MIN_PAYLEN	60 - sizeof(struct ether_header) - sizeof(struct aoe_header)
static void
update_packet(struct aoe_packet * pkt, struct ether_addr * src, struct ether_addr * dst,
		enum command cmd, uint8_t sub, uint32_t opt1, uint32_t opt2, uint16_t paylen)
{
	/* copy ether header */
	struct ether_header * eh = &pkt->eh;	/* 14 bytes */
	memcpy(&eh->ether_dhost, dst, 6);
	memcpy(&eh->ether_shost, src, 6);
	eh->ether_type = htons(ETHERTYPE_LE2);

	/* Set Null at the end of the buffer */
	if (paylen < MIN_PAYLEN) {
		memset(pkt->body + paylen, '\0', MIN_PAYLEN - paylen);
		paylen = MIN_PAYLEN;
	}
	pkt->body[paylen] = '\0';

	/* copy AoE header */
	pkt->aoe.u16key	= htons(g.key);
	pkt->aoe.u8cmd	= cmd;
	pkt->aoe.u8sub	= sub;
	pkt->aoe.u32opt1 = htonl(opt1);
	pkt->aoe.u32opt2 = htonl(opt2);
	pkt->aoe.u16len	= htons(paylen);
	pkt->aoe.u16sum	= 0;
#ifdef CSUM
	if (unlikely(g.options & OPT_CSUM))
		pkt->aoe.u16sum = wrapsum(checksum(pkt->body, paylen, 0));
#endif
}

/*
 * Search for the available TX buffer, set the Ether header and AoE header,
 * and return the starting address of the buffer.
 * Pay attention to setting the payload length at this point.
 */
static struct aoe_packet * prepare_tx_packet(struct targ * targ, struct ether_addr * src, struct ether_addr * dst,
		enum command cmd, uint8_t sub, uint32_t opt1, uint32_t opt2, uint16_t paylen)
{
	int i;
	struct netmap_if * nifp = targ->nmd->nifp;
	struct netmap_ring * txring = NULL;

	/* scan our queues and send on those with room */
	for (i = targ->nmd->first_tx_ring; i < targ->nmd->last_tx_ring; i++) {
		txring = NETMAP_TXRING(nifp, i);
		if (unlikely(nm_ring_empty(txring)))
			continue;

		u_int head = txring->head;
		struct netmap_slot *slot = &txring->slot[head];

		struct aoe_packet *pkt = (struct aoe_packet *)NETMAP_BUF(txring, slot->buf_idx);
		update_packet(pkt, src, dst, cmd, sub, opt1, opt2, paylen);

		/* Please be aware that the payload length may not meet the minimum size of the Ethernet frame. */
		int size = sizeof(struct ether_header) + sizeof(struct aoe_header) + ntohs(pkt->aoe.u16len);
		slot->len = size;
		slot->flags = NS_REPORT;
		txring->head = txring->cur = nm_ring_next(txring, head);
		return pkt;
	}
	/* */
	return NULL;
}
#ifdef CLI
static void sigusr_handler(int sig)
{
	c.sig = sig;
	return;
}
#endif
static void signal_handler(int sig)
{
	if (sig == SIGABRT || sig == SIGTERM || sig == SIGINT) {
		if (g.stat == CONNECTED) {
			g.stat = TEARDOWN;
		} else {
			g.stat = EXIT;
		}
		return;
	}
	signal(sig, SIG_DFL);
}

static void
swapto(int is_hwring, struct netmap_slot * rxslot)
{
	struct netmap_ring * txring;
	int i, first, last;
	uint32_t cur, t;

	if (is_hwring) {
		first = last = g.nmd->last_tx_ring;
	} else {
		first = g.nmd->first_tx_ring;
		last = g.nmd->last_tx_ring - 1;
	}
	for (i = first; i <= last; i++) {
		txring = NETMAP_TXRING(g.nmd->nifp, i);
		if (unlikely(nm_ring_empty(txring)))
			continue;

		cur = txring->cur;

		// swap buf_idx
		t = txring->slot[cur].buf_idx;
		txring->slot[cur].buf_idx = rxslot->buf_idx;
		rxslot->buf_idx = t;

		// set len
		txring->slot[cur].len = rxslot->len;

		// update flags
		txring->slot[cur].flags |= NS_BUF_CHANGED;
		rxslot->flags |= NS_BUF_CHANGED;

		// update ring pointer
		cur = nm_ring_next(txring, cur);
		txring->head = txring->cur = cur;

		break;
	}
}
static void __attribute__ ((noinline)) teardown(char *msg)
{
#ifdef CLI
	if (g.read_fd) {
		close(g.read_fd);
	}
#endif
	if (g.nmd && g.nmd->fd > 0) {
		/* final part: wait all the TX queues to be empty. */
		for (int i = g.nmd->first_tx_ring; i <= g.nmd->last_tx_ring; i++) {
			struct netmap_ring *txring = NETMAP_TXRING(g.nmd->nifp, i);
			while (nm_tx_pending(txring)) {
				ioctl(g.nmd->fd, NIOCTXSYNC, NULL);
				usleep(1);
			}
		}
		nmport_close(g.nmd);
		P("teardown ... netmap fd closed.");
	}

#ifdef SVR
	if (g.aoe_buf_addr) {
		ext->unarrived_dreq_packets = 0;
		munmap(g.aoe_buf_addr, AOE_BUF_SIZE * 2);
		P("teardown ... aoe buffer unmaped.");
	}
	if (g.aoe_buf_fd) {
		close(g.aoe_buf_fd);
		P("teardown ... aoe buffer fd closed.");
	}

	/* alsa */
	pcm_exit();
	P("teardown ... pcm exit.");
#endif
	if (msg != NULL) {
		P("EXIT stat:%-9s %s", Connects[g.stat], msg);
		exit(-1);
	} else {
		P("EXIT stat:%-9s", Connects[g.stat]);
	}
}

static void send_close(struct targ *targ, struct ether_addr *src, struct ether_addr *dst)
{
	/* C_CLOSE (Close)
	 *	u8cmd	C_CLOSE
	 *	u8sub	0 (not use)
	 *	u32opt1	0 (not use)
	 *	u32opt2	0 (not use)
	 */
	char * payload = "Good Bye";
	struct aoe_packet * _pkt = prepare_tx_packet(targ, src, dst, C_CLOSE, 0, 0, 0, strlen(payload));
	memcpy(_pkt->body, payload, strlen(payload));

	targ->stat = CLOSED;
	targ->key = 0;
	targ->poll_timeout_ms = DEFAULT_TIMEOUT_MS;
#ifdef CLI
	c.last_mixer_value = 0;
#endif
}

static inline int
unbox(void * buf, struct ether_addr * src, struct ether_addr * dst)
{
	struct aoe_packet * _pkt;
#ifdef SVR
	struct netmap_slot * const slot = s.cur_slot;
#endif
	struct ether_header * eh = (struct ether_header *)buf;
	struct aoe_header * aoe = (struct aoe_header *)(eh + 1);
#if (defined CSUM || defined CLI)
	char * payload = (char *)(aoe + 1);
#endif
	if (eh->ether_type == htons(ETHERTYPE_LE2)) {
		switch (aoe->u8cmd) {
		case C_DRES: { /* SERVER side */ }
#ifdef SVR
			struct timespec aoe_ts = {ntohl(aoe->u32opt1), ntohl(aoe->u32opt2)};
			if (g.stat != CONNECTED || g.key != ntohs(aoe->u16key)
				|| s.last_ts.tv_sec != aoe_ts.tv_sec || s.last_ts.tv_nsec != aoe_ts.tv_nsec)
				break;
#if (defined SERVER_STATS && defined CSUM)
			// checksum option
			if (unlikely(g.options & OPT_CSUM)) {
				uint16_t check = (aoe->u16sum != wrapsum(checksum(payload, ntohs(aoe->u16len), 0)));
				if (check)
					s.ev.error++;
			}
#endif
			// produce
			int _head = ext->head;
			uint32_t target;
			target = _head + aoe->u8sub;
			if (target >= (uint32_t)ext->num_slots)
				target -= ext->num_slots;
			uint32_t tmp;
			tmp = ext->slot[target].buf_idx;
			ext->slot[target].buf_idx = slot->buf_idx;
			slot->buf_idx = tmp;

			slot->flags |= NS_BUF_CHANGED;
			s.recv++;
			if (s.req == s.recv) {
				__atomic_store_n(&ext->head,
					(_head + s.recv > ext->num_slots -1 ? _head + s.recv - ext->num_slots:_head+s.recv), __ATOMIC_RELEASE);
#ifdef SERVER_STATS
				unsigned int _trip = timespec_diff_ns(aoe_ts, s.cur_ts)/1000;
				s.trip_us_total += _trip;
				calc_stats(&s.trip_stats[s.req-1], _trip);
				s.recv_total += s.recv;
				s.poll_total += s.poll_count;
				s.req_count++;
#endif
				reset(&s);
			}
#endif
			break;
		case C_DREQ: /* CLIENT side */
#ifdef CLI
			if (g.stat != CONNECTED || g.key != ntohs(aoe->u16key))
				break;
			int ret = 0;
			for (int i = 0, last = aoe->u8sub; i < last; i++) {
				_pkt = prepare_tx_packet(&g, src, dst, C_DRES, i, ntohl(aoe->u32opt1), ntohl(aoe->u32opt2), c.chunk_bytes);
				ret = read_buf(g.read_fd, (void *)_pkt->body, c.chunk_bytes);
				if (likely(ret > 0)) {
					/* C_DRES (Data Response)
					 *	u8cmd	C_DRES
					 *	u8sub	response index of requests
					 *	u32opt1	DREQ timespec sec
					 *	u32opt2	DREQ timespec nsec
					 */
				}
				if (unlikely(ret < c.chunk_bytes)) {
					P("recv *Data Request*, but buffer read return %d (%d:%s), send *Close*",
						ret, errno, strerror(errno));
					ioctl(g.nmd->fd, NIOCTXSYNC, NULL);
					send_close(&g, src, dst);
					break;
				}
			}
#endif
			break;
		case C_DISCOVER: /* SERVER side */
#ifdef SVR
			if (g.stat != CLOSED)
				break;

			// format/rate/channels
			snd_pcm_format_t _format = aoe->u8sub;
			unsigned int _rate = ntohl(aoe->u32opt1);
			unsigned int _channels = ntohl(aoe->u32opt2);
			if (s.pcm.format != _format
				|| s.pcm.rate != _rate
				|| s.pcm.channels != _channels) {

				// When changing rates, wait until playback is complete.
				if (ext->head != ext->tail)
					break;

				__atomic_store_n(&ext->head, 0, __ATOMIC_RELEASE);
				__atomic_store_n(&ext->cur, 0, __ATOMIC_RELEASE);
				__atomic_store_n(&ext->tail, 0, __ATOMIC_RELEASE);

				pcm_rate_changed(&s, _format, _rate, _channels);
				pcm_ready(&s);
				ext->num_slots = s.cb_frames * 4;
				P("pcminfo %s %u %u chunk_bytes:%d period_us:%u", snd_pcm_format_name(s.pcm.format), s.pcm.rate, s.pcm.channels, s.chunk_bytes, s.pcm.period_us);
			}

			P("recv *Neighbor Discovery* from " ETHER_ADDR_FMT ", %s->SYN, send *SYN*",
				ETHER_ADDR_PTR((struct ether_addr *)&eh->ether_shost), Connects[g.stat]);
			g.stat = SYN;
			g.poll_timeout_ms = DEFAULT_TIMEOUT_MS;

			// update dst
			memcpy(dst, &eh->ether_shost, 6);

			/* C_SYN (Synchronize)
			 *	u8cmd	C_SYN
			 *	u8sub	0:checksum off 1:checksum on
			 *	u32opt1	chunk_bytes
			 *	u32opt2	session key
			 */
			clock_gettime(CLOCK_MONOTONIC, &s.last_ts);
			g.key = (uint16_t)s.last_ts.tv_nsec;
			char * payload = "Synchronize";
			_pkt = prepare_tx_packet(&g, src, dst, C_SYN, g.options & OPT_CSUM ? 1:0, (uint32_t)s.chunk_bytes, g.key, strlen(payload));
			memcpy(_pkt->body, payload, strlen(payload));
#endif
			break;
		case C_SYN: /* CLIENT side */
#ifdef CLI
			if (g.stat != CLOSED) {
				struct ether_addr _tmp;
				memcpy(&_tmp, &eh->ether_shost, 6);
				if (memcmp(&_tmp, dst, 6) != 0) {
					P("recv *SYN* from another server. " ETHER_ADDR_FMT,
						ETHER_ADDR_PTR((struct ether_addr*)&eh->ether_shost));
				}
				break;
			}
			c.chunk_bytes = (int)ntohl(aoe->u32opt1);
			g.key = (uint16_t)ntohl(aoe->u32opt2);
			P("recv *SYN* from " ETHER_ADDR_FMT ", %s->CONNECTED chunk_bytes:%d, send *ACK*",
				ETHER_ADDR_PTR((struct ether_addr *)&eh->ether_shost),
				Connects[g.stat], c.chunk_bytes);
			g.stat = CONNECTED;
			g.poll_timeout_ms = -1;

			// update dst only
			if (g.options & OPT_AUTO)
				memcpy(dst, &eh->ether_shost, 6);

			if (aoe->u8sub)
				g.options |= OPT_CSUM;

			/* C_ACK (Acknowledge)
			 *	u8cmd	C_ACK
			 *	u8sub	0 (not use)
			 *	u32opt1	0 (not use)
			 *	u32opt2	0 (not use)
			 */
			char * _payload = "Acknowledge";
			_pkt = prepare_tx_packet(&g, src, dst, C_ACK, 0, 0, 0, strlen(_payload));
			memcpy(_pkt->body, payload, strlen(_payload));
#endif
			break;
		case C_ACK: /* SERVER side */
#ifdef SVR
			if (g.stat != SYN || g.key != ntohs(aoe->u16key))
				break;
			P("recv *ACK*   %s->CONNECTED", Connects[g.stat]);
			g.stat = CONNECTED;
#ifdef SERVER_STATS
			s.poll_count = 0;
#endif
#endif
			break;
		case C_QUERY: { /* SERVER side */ }
#ifdef SVR
			// Ignore consecutive queries
			static struct timespec previous_ts = {0,0};
			struct timespec current_ts;
			clock_gettime(CLOCK_MONOTONIC, &current_ts);
			if (timespec_diff_ns(previous_ts, current_ts) < 50*1000000) {
				break;
			} else {
				previous_ts = current_ts;
			}

			P("recv *QUERY* sig:%d opt1:%06x " ETHER_ADDR_FMT "->" ETHER_ADDR_FMT,
				aoe->u8sub, ntohl(aoe->u32opt1),
				ETHER_ADDR_PTR((struct ether_addr*)&eh->ether_shost),
				ETHER_ADDR_PTR((struct ether_addr*)&eh->ether_dhost));
			if (aoe->u8sub == SIGRTMIN) {
#ifdef SERVER_STATS
				/* reset counter (aoereset) */
				P("recv *QUERY(%u)*, reset counter", aoe->u8sub);
				s.recv_total	= 0;
				s.forward_total = 0;
				s.poll_total	= 0;
				s.req_count	= 0;
				s.trip_us_total	= 0;
				s.act_ns_total	= 0;
				memset(s.trip_stats, 0, sizeof(s.trip_stats));
				s.poll_stats = (struct stats){ 0 };
				s.poll_stats.min = ULONG_MAX;
#endif
				break;
			} else if (aoe->u8sub >= SIGRTMAX-14){
				int cmd_arg = 0;
				if (aoe->u8sub == SIGMIXERCHG) {
					uint32_t mixer_value = (uint32_t)ntohl(aoe->u32opt1);
					struct aoe_param p = get_aoe_param(&mixer_value);
//					if (!(g.options & OPT_FIXED)) {
//						s.dreq_limits = p.dreq;
//						s.recv_limits = p.recv;
//					}
					if (s.last_volume_value == p.volume)
						break;
					cmd_arg = s.last_volume_value = p.volume;
				}

				/* exec remote command */
				char cmd[100];
				sprintf(cmd, "%s %d %d ", REMOTE_COMMAND, aoe->u8sub, cmd_arg);
				system(cmd);
				break;
			}
			/* C_REPORT (Report)
			 *	u8cmd	C_REPORT
			 *	u8sub	sig
			 *	u32opt1	AoE Status
			 *	u32opt2	SND_PCM_STATUS
			 */
			struct ether_addr query_host = {0};
			memcpy(&query_host, &eh->ether_shost, 6);

			memset(report, '\0', sizeof(report));
			if (aoe->u8sub == SIGUSR1) {
				/* show server status (lsaoe) */
				sprintf(report,
					"  PCM PARAM  : %s %u %u chunk_bytes:%d period_us:%u\n"
					"  AoE STATS  : aoe.dreq=%d aoe.recv=%d %s (count:%lu timeout:%lu recover:%lu)\n",
					snd_pcm_format_name(s.pcm.format), s.pcm.rate, s.pcm.channels, s.chunk_bytes, s.pcm.period_us,
					s.dreq_limits, s.recv_limits,
					g.options & OPT_FIXED ? "fixed":"",
					s.recv_total, s.ev.timeout, s.ev.recover);
#ifdef CSUM
				if (g.options & OPT_CSUM) {
					char _buf[50];
					unsigned long erate = s.ev.error > 0 ? (s.ev.error * 100)/s.stats.count:0;
					sprintf(_buf, "  CHECKSUM   : error %lu (%lu.%02lu%%)\n", s.ev.error, erate / 100, erate % 100);
					strcat(report, _buf);
				}
#endif
				strcat(report, "\n");
			} else if (aoe->u8sub == SIGUSR2) {
#ifdef SERVER_STATS
				/* show stats table (aoestat) */
				unsigned long _avg = s.poll_stats.count > 0 ? s.poll_stats.ttl/s.poll_stats.count:0;
				sprintf(report,
					"period     (us) : %6u\n"
					"receive packets : %6lu (AoE %lu, Others %lu)\n"
					"poll total      : %6lu\n"
					"poll avg.  (ms) : %6lu (min %lu, max %lu, expected %u)\n"
					"poll/chunk x1000: %6lu (aoe.recv=%d)\n"
					"dreq/chunk x1000: %6lu (aoe.dreq=%d)\n"
					"trip/chunk (us) : %6lu\n"
					"act /chunk (ns) : %6lu\n",
					s.pcm.period_us,
					s.recv_total+s.forward_total, s.recv_total, s.forward_total,
					s.poll_total,
					_avg, s.poll_stats.min == ULONG_MAX ? 0:s.poll_stats.min, s.poll_stats.max, s.pcm.period_us*s.dreq_limits/1000,
					s.recv_total ? 1000*s.poll_total/s.recv_total:0, s.recv_limits,
					s.recv_total ? 1000*s.req_count/s.recv_total:0, s.dreq_limits,
					s.recv_total ? s.trip_us_total/s.recv_total:0,
					s.recv_total ? s.act_ns_total/s.recv_total:0);
				for (int i = 0; i < s.dreq_limits; i++) {
					char _buf[50];
					long tavg = s.trip_stats[i].count > 0 ? (long)((long)(s.trip_stats[i].ttl)/s.trip_stats[i].count)/100:0;
					sprintf(_buf, "%2d:%5lu %2ld.%1ld\n", i+1, s.trip_stats[i].count, tavg/10, tavg%10);
					strcat(report, _buf);
				}
#endif
			}
			P("recv *QUERY(%u)*, send *REPORT* (%luB)", aoe->u8sub, strlen(report));
			_pkt = prepare_tx_packet(&g, src, &query_host, C_REPORT, aoe->u8sub, g.stat, s.pcm.state, strlen(report));
			memcpy(_pkt->body, report, strlen(report));

#endif
			break;
		case C_REPORT:	/* CLIENT side */
#ifdef CLI
			P("recv *REPORT(%u)* (%uB)", aoe->u8sub, ntohs(aoe->u16len));
			int _fd = open("/run/report", O_CREAT|O_WRONLY|O_APPEND, S_IRUSR|S_IWUSR|S_IRGRP|S_IROTH);
			if (_fd) {
				if (aoe->u8sub == SIGUSR1) {
					char _buf[200];
					sprintf(_buf,
						"TARGET [" ETHER_ADDR_FMT "]\n"
						"\n"
						"  AoE STATUS : %s\n"
						"  AoE SESSION: %6u\n"
						"  AoE VSOUND : %s(%d) (timeout:%lu eof:%lu intr:%lu)\n",
						ETHER_ADDR_PTR((struct ether_addr*)&eh->ether_shost),
						Connects[ntohl(aoe->u32opt1)],
						ntohs(aoe->u16key),
						snd_pcm_state_name(ntohl(aoe->u32opt2)), ntohl(aoe->u32opt2),
						c.readtimeout_count, c.eof_count, c.eintr_count);
					write(_fd, _buf, strlen(_buf));
				}
				write(_fd, payload, ntohs(aoe->u16len));
				close(_fd);
			}
#endif
			break;
		case C_CLOSE:
			if (g.stat != CONNECTED || g.key != ntohs(aoe->u16key))
				break;
			P("recv *Close* %s->CLOSED", Connects[g.stat]);
			g.stat = CLOSED;
			g.key = 0;
#ifdef SVR
			reset(&s);

			/* pcm_stop */
/*			if (ioctl(g.aoe_buf_fd, IOCTL_PCM_STOP)) {
				P("ioctl error! (IOCTL_PCM_STOP) err:%d", errno);
			} else {
				P("ioctl success (IOCTL_PCM_STOP)!");
			}
*/
#endif
#ifdef CLI
			c.last_mixer_value = 0;
#endif
			break;
		default:
			P("UNKNOWN COMMAND");
		}
		return 1;
	}
#ifdef SERVER_STATS
	s.forward_total++;
#endif
	return 0;
}

/* ALSA */
#ifdef SVR
#define NOT_SUPPORT -1
#define NUM_FORMATS 3
#define NUM_RATES	10
#define OPENMODE    0   /* block */
int avail_rate[NUM_RATES] = {44100, 48000, 88200, 96000, 176400, 192000, 352800, 384000, 705600, 768000};
snd_pcm_format_t avail_format[NUM_FORMATS] = {SND_PCM_FORMAT_S16, SND_PCM_FORMAT_S24, SND_PCM_FORMAT_S32};

static inline void *
idx_to_bufp (unsigned long idx)
{
	if (idx >= AOE_LUT_INDEX) {
		unsigned long idx2 = idx - AOE_LUT_INDEX;
		return (void *)g.aoe_buf_addr + AOE_BUF_SIZE + NM_BUFSZ * idx2;
	} else {
		idx = idx < NM_NUM_SLOTS + HW_RXRING_IDX0 ? (idx - HW_RXRING_IDX0) : (idx + NM_NUM_SLOTS - HOST_TXRING_IDX0);
		return (void *)g.aoe_buf_addr + NM_BUFSZ * idx;
	}
}
extern int __snd_pcm_hw_params_set_rate_near
	(snd_pcm_t *pcm, snd_pcm_hw_params_t *params, unsigned int *val, int *dir);
extern int __snd_pcm_hw_params_set_period_size_near
	(snd_pcm_t *pcm, snd_pcm_hw_params_t *params, snd_pcm_uframes_t *val, int *dir);
extern int __snd_pcm_hw_params_set_buffer_size_near
	(snd_pcm_t *pcm, snd_pcm_hw_params_t *params, snd_pcm_uframes_t *val);
extern int __snd_pcm_hw_params_get_period_size
	(const snd_pcm_hw_params_t *params, snd_pcm_uframes_t *val, int *dir);
extern int __snd_pcm_hw_params_get_buffer_size
	(const snd_pcm_hw_params_t *params, snd_pcm_uframes_t *val);
static snd_pcm_t *handle;

static void detect_initial_params(struct server_stats *stats)
{
	snd_pcm_format_t _format = SND_PCM_FORMAT_UNKNOWN;
	unsigned int _rate = 0;
	unsigned int _channels = 0;
	snd_pcm_hw_params_t *params;

	int err = snd_pcm_open(&handle, g.pcm_name, SND_PCM_STREAM_PLAYBACK, OPENMODE);
	if (err < 0) {
		teardown("snd_pcm_open failed");
	}

	snd_pcm_hw_params_alloca(&params);
	snd_pcm_hw_params_any(handle, params);

	int i;
	P("Available formats:");
	for (i = 0; i < NUM_FORMATS ; i++) {
		if (snd_pcm_hw_params_test_format(handle, params, avail_format[i]) == 0) {
			P("- %s", snd_pcm_format_name(avail_format[i]));
			if (_format == SND_PCM_FORMAT_UNKNOWN)
				_format = avail_format[i];
		} else {
			avail_format[i] = NOT_SUPPORT;
		}
	}
	P("Available rates:");
	for (i = 0; i < NUM_RATES ; i++) {
		if (snd_pcm_hw_params_test_rate(handle, params,(unsigned int)avail_rate[i], SND_PCM_STREAM_PLAYBACK) == 0) {
			P("- %d", avail_rate[i]);
			if (_rate == 0)
				_rate = avail_rate[i];
		} else {
			avail_rate[i] = NOT_SUPPORT;
		}
	}
	if (snd_pcm_hw_params_test_channels(handle, params, 2) == 0) {
		P("Stereo channel supported.");
		_channels = 2;
	} else {
		teardown("Stereo channel not supported!");
	}

	pcm_rate_changed(stats, _format, _rate, _channels);
}
static int pcm_set_params(struct server_stats *stats)
{
	int err;
	snd_pcm_hw_params_t *params;
	snd_pcm_sw_params_t *swparams;
	snd_pcm_hw_params_alloca(&params);
	snd_pcm_sw_params_alloca(&swparams);
	err = snd_pcm_hw_params_any(handle, params);
	if (err < 0) {
		P("Broken configuration for this PCM: no configurations available");
		return -1;
	}

	snd_pcm_access_mask_t *mask = alloca(snd_pcm_access_mask_sizeof());
	snd_pcm_access_mask_none(mask);
	snd_pcm_access_mask_set(mask, SND_PCM_ACCESS_MMAP_INTERLEAVED);
	err = snd_pcm_hw_params_set_access_mask(handle, params, mask);
	if (err < 0) {
		P("Access type not available");
		return -1;
	}
	err = snd_pcm_hw_params_set_format(handle, params, stats->pcm.format);
	if (err < 0) {
		P("Sample format non available");
		return -1;
	}
	err = snd_pcm_hw_params_set_channels(handle, params, stats->pcm.channels);
	if (err < 0) {
		P("Channels count non available");
		return -1;
	}

	err = __snd_pcm_hw_params_set_rate_near(handle, params, &stats->pcm.rate, 0);
	snd_pcm_uframes_t buffer_size =  stats->period_size * stats->cb_frames;
	err = __snd_pcm_hw_params_set_period_size_near(handle, params, &stats->period_size, 0);
	err = __snd_pcm_hw_params_set_buffer_size_near(handle, params, &buffer_size);
	err = snd_pcm_hw_params(handle, params);
	if (err < 0) {
		P("Unable to set hw params for playback: %s (format:%d)", snd_strerror(err), stats->pcm.format);
		return -1;
	}

	snd_pcm_sw_params_current(handle, swparams);
	err = snd_pcm_sw_params_set_avail_min(handle, swparams, stats->period_size);
	err = snd_pcm_sw_params_set_start_threshold(handle, swparams, buffer_size);
	if (snd_pcm_sw_params(handle, swparams) < 0) {
		P("unable to install sw params:");
		return -1;
	}
	return 0;
}
static void pcm_exit()
{
	if (handle) {
		snd_pcm_drain(handle);
		snd_pcm_close(handle);
	}
	snd_config_update_free_global();
}
static void pcm_ready(struct server_stats *stats)
{
	if (handle) {
		snd_pcm_drain(handle);
		snd_pcm_close(handle);
	}

	int err = snd_pcm_open(&handle, g.pcm_name, SND_PCM_STREAM_PLAYBACK, OPENMODE);
	if (err < 0) {
		teardown("snd_pcm_open failed");
	} else {
		P("snd_pcm_open OK!");
	}
	if (pcm_set_params(stats) < 0) {
		teardown("pcm_set_params failed");
	} else {
		P("pcm_set_params OK!");
	}
}
static void pcm_rate_changed
	(struct server_stats * sptr, snd_pcm_format_t format, unsigned int rate, unsigned int channels)
{
	sptr->pcm.format = format;
	sptr->pcm.rate = rate;
	sptr->pcm.channels = channels;

	const size_t bytes_per_sample = snd_pcm_format_physical_width(format)/8;
	sptr->period_size = (MTU - sizeof(struct aoe_header)) / (channels * bytes_per_sample);
	sptr->chunk_bytes = sptr->period_size * channels * bytes_per_sample;
	sptr->pcm.period_us = 1000000 * sptr->period_size / rate;

	/*
	 * Note that ext->num_slots is four times cb_frames.
	 * Since the maximum value for num_slots is 1024, cb_frames can be up to a maximum of 256.
	 */
	switch (rate) {
	case 48000:
	case 44100:
		sptr->cb_frames = 32;
		sptr->dreq_limits = sptr->recv_limits = 32;
		break;
	case 96000:
	case 88200:
		sptr->cb_frames = 64;
		sptr->dreq_limits = sptr->recv_limits = 40;
		break;
	case 192000:
	case 176400:
		sptr->cb_frames = 128;
		sptr->dreq_limits = sptr->recv_limits = 48;
		break;
	case 384000:
	case 352800:
		sptr->cb_frames = 256;
		sptr->dreq_limits = sptr->recv_limits = 56;
		break;
	default:
		// 768000, 705600, and others
		sptr->cb_frames = 256;
		sptr->dreq_limits = sptr->recv_limits = 64;
		break;
	}
}
#endif

int main(int arc, char **argv)
{
//	struct aoe_packet pkt;
	struct ether_addr src, dst;
#ifdef SVR
	s = (struct server_stats){ 0 };
//	s.dreq_limits = CAP;
//	s.recv_limits = CAP;
#ifdef SERVER_STATS
	s.poll_stats = (struct stats){ 0 };
	s.poll_stats.min = ULONG_MAX;
#endif
#endif
	g = (struct targ){ 0 };
	g.mode = CLIENT;
	g.stat = CLOSED;
	g.options |= OPT_AUTO;
	g.poll_timeout_ms = DEFAULT_TIMEOUT_MS;
	strcpy(g.ifname, "eth0");
	strcpy(g.pcm_name, "hw:0,0");

	int ch;
	while ((ch = getopt(arc, argv, "hvi:D:d:sr:p:c:o:")) != -1) {
		switch(ch) {
		default:
			D("bad option %c %s", ch, optarg);
			usage(-1);
			break;
		case 'h':
		case 'v':
			usage(0);
			break;
		case 'i':	/* interface */
			if (strlen(optarg) > MAX_IFNAMELEN - 8) {
				D("ifname too long %s", optarg);
				break;
			}
			strcpy(g.ifname, optarg);
			break;
#ifdef CLI
		case 'd':	/* dst mac address */
			g.options &= ~OPT_AUTO;
			ether_aton_r(optarg, &dst);
			break;
#endif
#ifdef CSUM
		case 's':
			g.options |= OPT_CSUM;
			break;
#endif
#ifdef SVR
		case 'D':
			if (strlen(optarg) > MAX_ARGLEN) {
				D("device name too long %s", optarg);
				break;
			}
			strcpy(g.pcm_name, optarg);
			break;
//		case 'd': { /* dreq packets */ }
/*			int _dreq = (int)strtol(optarg, NULL, 10);
			if (1 <= _dreq && _dreq <= 64) {
				s.dreq_limits = _dreq;
				g.options |= OPT_FIXED;
			}
			break;*/
//		case 'r': { /* recv limits */ }
/*			int _recv = (int)strtol(optarg, NULL, 10);
			if (1 <= _recv && _recv <= 64) {
				s.recv_limits = _recv;
				g.options |= OPT_FIXED;
			}
			break;*/
#endif
		}
	}

	/* set signal */
	signal(SIGINT, signal_handler);
	signal(SIGTERM, signal_handler);
	signal(SIGABRT, signal_handler);
#ifdef CLI
	signal(SIGUSR1, sigusr_handler);
	signal(SIGUSR2, sigusr_handler);
	signal(SIGRTMIN, sigusr_handler);
	signal(SIGRTMIN+1, sigusr_handler);
	signal(SIGRTMAX-14, sigusr_handler);
	signal(SIGRTMAX-13, sigusr_handler);
	signal(SIGRTMAX-12, sigusr_handler);
	signal(SIGRTMAX-11, sigusr_handler);
#endif

	// netmap module check
	FILE *fp;
	fp = fopen("/sys/module/netmap/parameters/audio_over_ether", "r");
	if (fp) {
		if (fgetc(fp) == '1') {
			g.mode = SERVER;
#ifdef CSUM
			P("%s mode:SERVER start (CSUM:%s)", g.ifname, g.options & OPT_CSUM ? "ON":"OFF");
#else
			P("%s mode:SERVER start", g.ifname);
#endif
		}
		fclose(fp);
	} else {
#ifdef SVR
		teardown("The netmap must be AoE enabled");
#endif
	}

#ifdef SVR
	/* initial pcm setup */
	detect_initial_params(&s);
	pcm_ready(&s);

	/* open AoE buffer */
	g.aoe_buf_fd = open("/proc/aoe_buf", O_RDWR | O_SYNC);
	if (g.aoe_buf_fd < 0) {
		teardown("aoe_buf open failed");
	}
	g.aoe_buf_addr = mmap(NULL, AOE_BUF_SIZE*2, PROT_READ | PROT_WRITE, MAP_SHARED, g.aoe_buf_fd, 0);
	if (g.aoe_buf_addr == MAP_FAILED) {
		teardown("aoe_buf mmap failed");
	}
	/* ext_slot */
	ext = (struct ext_slot *)((void *)g.aoe_buf_addr + AOE_BUF_SIZE + NM_BUFSZ * AOE_NUM_SLOTS);
	ext->num_slots = s.cb_frames * 4;
	ext->tail = s.req = 0;
	__atomic_store_n(&ext->head, 0, __ATOMIC_RELEASE);
	__atomic_store_n(&ext->cur, 0, __ATOMIC_RELEASE);
#endif

#ifdef CLI
	g.read_fd = open("/dev/vsound", O_RDONLY);
	if (g.read_fd < 0)
		teardown("vsound open error");
#endif
	/* mac address */
	if (g.options & OPT_AUTO)
		memset(&dst, 0xff, 6);
	source_hwaddr(g.ifname, &src);
	P("HWaddr " ETHER_ADDR_FMT, ETHER_ADDR_PTR(&src));

	/* nmport */
	char netmapif[100];
	snprintf(netmapif, sizeof(netmapif), "netmap:%s*", g.ifname);
	g.nmd = nmport_prepare(netmapif);
	if (g.nmd == NULL)
		teardown("nmport_prepare failed");
	if (nmport_open_desc(g.nmd) < 0)
		teardown("nmport_open_desc failed");
	if (g.nmd->fd < 0)
		teardown("nmport setup failed");

	/* sanity check buf_idx */
#ifdef SVR
	struct netmap_ring *ring;
	ring = NETMAP_RXRING(g.nmd->nifp, g.nmd->first_rx_ring);
	if (ring->slot[0].buf_idx != HW_RXRING_IDX0) {
		D("HW RXRING buf_idx:%u", ring->slot[0].buf_idx);
		teardown("HW RXRING buf_idx anomaly");
	}
	ring = NETMAP_TXRING(g.nmd->nifp, g.nmd->last_tx_ring);
	if (ring->slot[0].buf_idx != HOST_TXRING_IDX0) {
		D("HOST TXRING buf_idx:%u", ring->slot[0].buf_idx);
		teardown("HOST TXRING buf_idx anomaly");
	}
#endif
	/* main loop */
	unsigned int cur, n, i;
	unsigned int is_hwring;
	struct netmap_ring *rxring;
	struct pollfd pollfd[1];
	memset(&pollfd, 0, sizeof(pollfd));
	pollfd[0].fd = g.nmd->fd;
	pollfd[0].events = POLLIN;
	unsigned int timeout_count = 0;

	while (1) {
#ifdef CLI
		clock_gettime(CLOCK_MONOTONIC, &c.cur_ts);
		read_stats();
		if (c.pcm.closed && g.stat == CONNECTED) {
			P("send *Close*");
			send_close(&g, &src, &dst);
		} else {
			switch (c.pcm.state) {
			case SND_PCM_STATE_PREPARED:
				if (g.stat == CLOSED) {
					if (timespec_diff_ns(c.last_ts, c.cur_ts) > BROADCAST_MS * 1000000) {
						if (++timeout_count > TIMEOUT_LIMITS) {
							P("Neighbor Discovery reach the upper limit!, PCM Suspended");
							timeout_count = 0;
							/* pcm suspend */
							ioctl(g.read_fd, 0, NULL);
							break;
						}

						/* C_DISCOVER (Neighbor Discovery Broadcast)
						 *	u8cmd	C_DISCOVER
						 *	u8sub	snd_pcm_format_t format
						 *	u32opt1	rate
						 *	u32opt2	channels
						 */
						if (g.options & OPT_AUTO)
							memset(&dst, 0xff, 6); // Broadcast

						char * payload = "Neighbor Discovery";
						struct aoe_packet * _pkt = prepare_tx_packet(&g, &src, &dst, C_DISCOVER, (uint8_t)c.pcm.format, c.pcm.rate, c.pcm.channels, strlen(payload));
						memcpy(_pkt->body, payload, strlen(payload));
						c.last_ts = c.cur_ts;
						g.poll_timeout_ms = BROADCAST_MS;
						P("send *Neighbor Discovery* SND_PCM_STATE:%s(%d) %s %u %u",
							snd_pcm_state_name(c.pcm.state), c.pcm.state,
							snd_pcm_format_name(c.pcm.format), c.pcm.rate, c.pcm.channels);
					}
				} else {
					/* clear count */
					timeout_count = 0;
				}
				break;
			case SND_PCM_STATE_OPEN:
			case SND_PCM_STATE_PAUSED:
			case SND_PCM_STATE_SUSPENDED:
			case SND_PCM_STATE_DISCONNECTED:
				if (g.stat == CONNECTED) {
					P("SND_PCM_STATE:%s(%d), send *Close*", snd_pcm_state_name(c.pcm.state), c.pcm.state);
					send_close(&g, &src, &dst);
				}
				break;
			default :
				break;
			}
			/* digital volume */
			if (g.stat == CONNECTED && c.last_mixer_value != c.pcm.aoe_mixer_value) {
				c.sig = SIGMIXERCHG;
			}
		}
#endif

#ifdef SVR
		if (likely(g.stat == CONNECTED)) {
			/* buffer control */
			int _tail = __atomic_load_n(&ext->tail, __ATOMIC_ACQUIRE);
			int avail = ext->head >= _tail ? (ext->head - _tail):(ext->num_slots + ext->head - _tail);
			g.poll_timeout_ms = DEFAULT_TIMEOUT_MS;

			/* DREQ */
			if (s.req == 0) {
				int space = ext->num_slots - avail -1;
				if (space > s.dreq_limits/2) {
					/* C_DREQ (Data Request)
					 *	u8cmd	C_DREQ
					 *	u8sub	Number of Request Chunks
					 *	u32opt1	DREQ timespec sec
					 *	u32opt2	DREQ timespec nsec
					 */
					s.req = space < s.dreq_limits ? space:s.dreq_limits;
					timeout_count = 0;
					s.last_ts = s.cur_ts;
					ext->unarrived_dreq_packets = s.req < s.recv_limits ? s.req:s.recv_limits ;
					char * payload = "Data Request";
					struct aoe_packet * _pkt = prepare_tx_packet(&g, &src, &dst, C_DREQ, s.req, s.cur_ts.tv_sec, s.cur_ts.tv_nsec, strlen(payload));
					memcpy(_pkt->body, payload, strlen(payload));
				} else {
					/* round down */
					g.poll_timeout_ms = (s.dreq_limits - space) * s.pcm.period_us /1000;
					if (g.poll_timeout_ms == 0)
						g.poll_timeout_ms = 1;
#ifdef SERVER_STATS
					calc_stats(&s.poll_stats, g.poll_timeout_ms);
#endif
				}
			} else {
				/* data request timeout */
				if (unlikely(timespec_diff_ns(s.last_ts, s.cur_ts) > DREQ_TIMEOUT_MS * 1000000)) {
#ifdef SERVER_STATS
					incr(s.ev.timeout);
#endif
					s.last_ts = s.cur_ts;

					if (++timeout_count > TIMEOUT_LIMITS) {
						P("DREQ TIMEOUT reach the upper limit!, send *Close*");
						timeout_count = 0;
						reset(&s);
						send_close(&g, &src, &dst);
					} else {
						P("RECOVER! req:%d recv:%d", s.req, s.recv);
#ifdef SERVER_STATS
						incr(s.ev.recover);
#endif
						/* RECOVER */
						__atomic_store_n(&ext->head,
							(ext->head + s.recv > ext->num_slots -1 ? ext->head + s.recv - ext->num_slots:ext->head+s.recv), __ATOMIC_RELEASE);
						reset(&s);

						/* DREQ */
						ext->unarrived_dreq_packets = s.req = 1;
						char * payload = "Data Request";
						struct aoe_packet * _pkt = prepare_tx_packet(&g, &src, &dst, C_DREQ, s.req, s.cur_ts.tv_sec, s.cur_ts.tv_nsec, strlen(payload));
						memcpy(_pkt->body, payload, strlen(payload));
					}
				} else {
					ext->unarrived_dreq_packets = (s.req - s.recv) < s.recv_limits ? (s.req - s.recv):s.recv_limits ;
				}
			}
		}
#endif

#ifdef SVR
		// If DMA is inactive, call snd_pcm_start using ioctl.
		if (ext->stat == INACTIVE) {
			const int ext_playable = ext->head >= ext->cur ? (ext->head - ext->cur):(ext->num_slots + ext->head - ext->cur);
			if (ext_playable >= ext->num_slots/2) {
				if (ioctl(g.aoe_buf_fd, IOCTL_PCM_START)) {
					P("ioctl error!");
				} else {
					P("ioctl success!(playable:%d)", ext_playable);
				}
			}
		}
#endif

#ifdef SERVER_STATS
		/* act_ns_total */
		if (g.stat == CONNECTED) {
			struct timespec _ts;
			clock_gettime(CLOCK_MONOTONIC, &_ts);
			s.act_ns_total += timespec_diff_ns(s.cur_ts, _ts);
		}
		s.poll_count++;
		int poll_ret = poll(pollfd, 1, g.poll_timeout_ms);
		clock_gettime(CLOCK_MONOTONIC, &s.cur_ts);
		if (poll_ret == 0)
			continue;
#else
		if (poll(pollfd, 1, g.poll_timeout_ms) == 0)
			continue;
#endif
		for (i = g.nmd->first_rx_ring; i <= g.nmd->last_rx_ring; i++) {
			is_hwring = (i != g.nmd->last_rx_ring);
			rxring = NETMAP_RXRING(g.nmd->nifp, i);
			cur = rxring->cur;
			for (n = nm_ring_space(rxring); n > 0; n--, cur = nm_ring_next(rxring, cur)) {
				if (is_hwring) {
					void *buf;
#ifdef CLI
					buf = NETMAP_BUF(rxring, rxring->slot[cur].buf_idx);
#endif
#ifdef SVR
					buf = idx_to_bufp(rxring->slot[cur].buf_idx);
					s.cur_slot = &rxring->slot[cur];
#endif
					if(likely(unbox(buf, &src, &dst))) {
						continue;
					}
				}
				swapto(is_hwring, &rxring->slot[cur]);
			}
			rxring->head = rxring->cur = cur;
		}

#ifdef CLI
		if (unlikely(c.sig > 0)) {
			/* C_QUERY (Query)
			 *	u8cmd	C_QUERY
			 *	u8sub	signal
			 *	u32opt1	mixer_value
			 *	u32opt2	0 (not use)
			 */
			struct ether_addr dummy_host = {0};
			if (c.sig == SIGMIXERCHG) {
				memcpy(&dummy_host, &dst, 6);
				struct aoe_param before = get_aoe_param(&c.last_mixer_value);
				struct aoe_param after = get_aoe_param(&c.pcm.aoe_mixer_value);
				P("send *Query* Mixer Change: v%d d%d r%d -> v%d d%d r%d (%06x)",
					before.volume, before.dreq, before.recv,
					after.volume, after.dreq, after.recv, c.pcm.aoe_mixer_value);
				c.last_mixer_value = c.pcm.aoe_mixer_value;
			} else if (g.options & OPT_AUTO || c.sig == SIGUSR1) {	/* show server status (lsaoe) */
				memset(&dummy_host, 0xff, 6);
			}
			char * payload = "Query";
			struct aoe_packet * _pkt = prepare_tx_packet(&g, &src, &dummy_host, C_QUERY, c.sig, c.pcm.aoe_mixer_value, 0, strlen(payload));
			memcpy(_pkt->body, payload, strlen(payload));
			c.sig = 0;
		}
#endif
		switch (g.stat) {
#ifdef SVR
		case SYN:/* SYN timeout */
			if (timespec_diff_ns(s.last_ts, s.cur_ts) > SYN_TIMEOUT_MS * 1000000) {
				g.stat = CLOSED;
				g.key = 0;
				P("SYN TIMEOUT!, reset last_ts");
			}
			break;
		case CLOSED:
			if (s.recv) {
				__atomic_store_n(&ext->head,
					(ext->head + s.recv > ext->num_slots -1 ? ext->head + s.recv - ext->num_slots:ext->head+s.recv), __ATOMIC_RELEASE);
			}
			reset(&s);
			break;
#endif
		case TEARDOWN:
			send_close(&g, &src, &dst);
			ioctl(g.nmd->fd, NIOCTXSYNC, NULL);
			goto END;
		case EXIT:
			goto END;
		default:
			break;
		}
	}
END:
	P("reach END: label");
	teardown(NULL);
	return 0;
}
