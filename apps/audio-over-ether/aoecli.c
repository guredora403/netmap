#include <limits.h>
#include <signal.h>
#include <errno.h>
#include <stdbool.h>
#include <sys/epoll.h>
#include <sys/ioctl.h>
#include <arpa/inet.h>			/* ntohs */
#include <linux/if_packet.h>    /* sockaddr_ll */
#include <netinet/ether.h>
#include <net/if.h>
#include <net/if_arp.h>
#include <net/ethernet.h>
#include <alsa/asoundlib.h>
#include "audio_over_ether.h"
#include "vsound.h"
#include "aoe.h"

#define BROADCAST_MS		500
#define MAX_EVENTS			10
#define MAX_RETRY_COUNT		200
#define RETRY_INTERVAL_US	1000*100
#define MODEL_INTERVAL_SEC	10
#define ONESHOT				NULL
#define likely(x)       __builtin_expect(!!(x), 1)
#define unlikely(x)     __builtin_expect(!!(x), 0)

static void usage(int errcode)
{
	const char *cmd = "aoecli";
	fprintf(stdout,
		"%s %s\n"
		"Usage:\n"
		"  -h             Show program usage and exit.\n"
		"  -i INTERFACE   (Optional)Network interface name. (Default eth0)\n"
		"  -d MAC_ADDR    (Optional)Destination MAC address.\n"
#ifdef CSUM
		"  -s             (Optional)Enable checksums.\n"
#endif
		, cmd, AOE_VERSION);
	exit(errcode);
}
struct globals {
	enum Status stat;
	uint16_t key;
	int poll_timeout_ms;
	int options;
	#define OPT_AUTO	1
	#define OPT_CSUM	2
	struct sockaddr_ll sll;
	int aoe_buf_fd;
	int read_fd;
	int sock_fd;
	int epoll_fd;
	char if_name[MAX_IFNAMELEN];
};
static struct globals g;

#define PACKET_BUFFER_SIZE sizeof(struct ether_header) + MTU_ETH
static char tx_buf[PACKET_BUFFER_SIZE];
static char rx_buf[PACKET_BUFFER_SIZE];
static char pkt_array[64][PACKET_BUFFER_SIZE];
struct client_stats {
	int chunk_bytes;
	int sig;
	struct timespec cur_ts;
	struct timespec last_ts;
	struct vsound_pcm pcm;
	struct vsound_pcm session_pcm;
	uint32_t last_mixer_value;
	uint32_t last_dreq_opt1;	// dreq sequence no
	uint32_t last_dreq_opt2;	// dreq sequence no
	unsigned long readtimeout_count;
	unsigned long eof_count;
	unsigned long eintr_count;
	unsigned long outgoing_count;
	struct timespec model_req_ts;
};
static struct client_stats c;

#define READ_SLEEP_US	(1000000 * 10 / c.pcm.rate)
static int read_stats(void)
{
	return read(g.read_fd, &c.pcm, 0);
}
static int read_buf(int fd, char *buf, int count, uint64_t timeout)
{
	int l = 0, r;
	do {
		r = read(fd, buf + l, count - l);
		if (likely(r > 0)) {
			l += r;
			continue;
		} else if (r == 0) {
			c.eof_count++;
			if (l > 0) {
				/* drain */
				memset(buf + l, 0x00, count - l);
			}
			return l;
		} else {
			if (errno == EINTR) {
				c.eintr_count++;
				continue;
			} else if (errno == EAGAIN) {
				struct timespec _now;
				clock_gettime(CLOCK_MONOTONIC, &_now);
				if (timespec_to_ns(_now) < timeout) {
					usleep(READ_SLEEP_US);
					continue;
				}
			}
			c.readtimeout_count++;
			if (l > 0) {
				/* drain */
				memset(buf + l, 0x00, count - l);
				return l;
			}
			return -ENODATA;
		}
	} while (l < count);

	return l;
}

static void sigusr_handler(int sig)
{
	c.sig = sig;
	return;
}
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
static void __attribute__ ((noinline)) teardown(char *msg)
{
	if (g.epoll_fd)
		close(g.epoll_fd);

	if (g.sock_fd)
		close(g.sock_fd);

	if (g.read_fd)
		close(g.read_fd);

	if (msg != NULL) {
		P("EXIT stat:%-9s %s", Connects[g.stat], msg);
		exit(-1);
	} else {
		P("EXIT stat:%-9s", Connects[g.stat]);
	}
}

static struct aoe_packet * prepare_tx_packet(
	struct aoe_packet * _pkt,
	struct ether_addr * src, struct ether_addr * dst,
	uint16_t key, enum command cmd, uint8_t sub,
	uint32_t opt1, uint32_t opt2, uint16_t paylen)
{
	/* copy ether header */
	if (_pkt == ONESHOT)
		_pkt = (struct aoe_packet*)tx_buf;
	struct ether_header * eh = &_pkt->eh;	/* 14 bytes */
	memcpy(&eh->ether_dhost, dst, 6);
	memcpy(&eh->ether_shost, src, 6);
	eh->ether_type = htons(ETHERTYPE_LE2);

	/* Set Null at the end of the buffer */
	if (paylen < AOE_MIN_PAYLEN) {
		memset(_pkt->body + paylen, '\0', AOE_MIN_PAYLEN - paylen);
		paylen = AOE_MIN_PAYLEN;
	}
	_pkt->body[paylen] = '\0';

	/* copy AoE header */
	_pkt->aoe.u16key	= htons(key);
	_pkt->aoe.u8cmd	= cmd;
	_pkt->aoe.u8sub	= sub;
	_pkt->aoe.u32opt1 = htonl(opt1);
	_pkt->aoe.u32opt2 = htonl(opt2);
	_pkt->aoe.u16len	= htons(paylen);
	_pkt->aoe.u16sum	= 0;

	return _pkt;
}

static void send_packet(struct aoe_packet * pkt)
{
	int tx_len = sizeof(struct ether_header) + sizeof(struct aoe_header) + ntohs(pkt->aoe.u16len);
	int ret;
	ret = sendto(g.sock_fd, (void *)pkt, tx_len, 0, (struct sockaddr *)&g.sll, sizeof(struct sockaddr_ll));
	if (ret == -1)
		g.stat = TEARDOWN;
}

static void send_query_model(struct ether_addr *src, struct ether_addr *dst)
{
	/* C_QUERY (Query)
	 *	u8cmd	C_QUERY
	 *	u8sub	signal
	 *	u32opt1	mixer_value
	 *	u32opt2	0 (not use)
	 */
	struct aoe_packet * _pkt = prepare_tx_packet(ONESHOT, src, dst, g.key, C_QUERY, MODEL_SPEC, 0, 0, 0);
	send_packet(_pkt);
}
static void send_close(struct ether_addr *src, struct ether_addr *dst)
{
	/* C_CLOSE (Close)
	 *	u8cmd	C_CLOSE
	 *	u8sub	0 (not use)
	 *	u32opt1	0 (not use)
	 *	u32opt2	0 (not use)
	 */
	char * payload = "Good Bye";
	struct aoe_packet * _pkt = prepare_tx_packet(ONESHOT, src, dst, g.key, C_CLOSE, 0, 0, 0, strlen(payload));
	memcpy(_pkt->body, payload, strlen(payload));
	send_packet(_pkt);

	c.session_pcm = (struct vsound_pcm) {0};
	g.stat = CLOSED;
	g.key = 0;
	g.poll_timeout_ms = DEFAULT_TIMEOUT_MS;
	c.last_mixer_value = 0;
	c.last_dreq_opt1 = 0;
	c.last_dreq_opt2 = 0;
}

static inline int unbox(struct aoe_packet * rx_pkt, struct ether_addr * src, struct ether_addr * dst)
{
	struct aoe_packet * tx_pkt;
	struct ether_header * eh = (struct ether_header *)rx_pkt;
	struct aoe_header * aoe = (struct aoe_header *)(eh + 1);
	char * rx_payload = (char *)(aoe + 1);

	if (eh->ether_type == htons(ETHERTYPE_LE2)) {
		switch (aoe->u8cmd) {
		case C_DREQ: /* CLIENT side */
			if (g.stat != CONNECTED || g.key != ntohs(aoe->u16key))
				break;

			/* Check if it's a retry or not. */
			uint32_t _opt1 =ntohl(aoe->u32opt1);
			uint32_t _opt2 =ntohl(aoe->u32opt2);
			if (_opt1 == c.last_dreq_opt1 && _opt2 == c.last_dreq_opt2) {
				P("recv retry request!");
				goto RETRY_REQUEST;
			}

			c.last_dreq_opt1 = _opt1;
			c.last_dreq_opt2 = _opt2;

			int ret = 0;
			/* Buffer read deadline */
			unsigned int buffered_playback_us = *((unsigned int*)rx_payload);
			struct timespec _ts;
			clock_gettime(CLOCK_MONOTONIC, &_ts);
			uint64_t limit_ns = timespec_to_ns(_ts) + buffered_playback_us * 1000;

			for (int i = 0, last = aoe->u8sub; i < last; i++) {
				/* C_DRES (Data Response)
				 *	u8cmd	C_DRES
				 *	u8sub	response index of requests
				 *	u32opt1	DREQ timespec sec
				 *	u32opt2	DREQ timespec nsec
				 */
				tx_pkt = prepare_tx_packet((struct aoe_packet *)pkt_array[i],
					src, dst, g.key, C_DRES, i, c.last_dreq_opt1, c.last_dreq_opt2, c.chunk_bytes);
				ret = read_buf(g.read_fd, (void *)tx_pkt->body, c.chunk_bytes, limit_ns);

				if (unlikely(ret < c.chunk_bytes)) {
					P("recv *Data Request*, but no data available (%d:%s), send *Close*",
						errno, strerror(errno));
					send_close(src, dst);
					return 1;
				}
			}
RETRY_REQUEST:
			for (int i = 0, last = aoe->u8sub; i < last; i++) {
				send_packet((struct aoe_packet *)pkt_array[i]);
			}

			/* dump first 16 Bytes */
			if (c.last_mixer_value != c.pcm.mixer_value) {
				struct aoe_packet * pkt = (struct aoe_packet *)pkt_array[0];
				uint8_t * s = (uint8_t *)pkt->body;
				P("%02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x",
					*(s), *(s+1), *(s+2), *(s+3), *(s+4), *(s+5), *(s+6), *(s+7),
					*(s+8), *(s+9), *(s+10), *(s+11), *(s+12), *(s+13), *(s+14), *(s+15));
			}

			break;
		case C_SYN: /* CLIENT side */
			if (g.stat != CLOSED) {
				struct ether_addr _tmp;
				memcpy(&_tmp, &eh->ether_shost, 6);
				if (memcmp(&_tmp, dst, 6) != 0) {
					P("recv *SYN* from another server. " ETHER_ADDR_FMT,
						ETHER_ADDR_PTR((struct ether_addr*)&eh->ether_shost));
				}
				break;
			}
			/* update session pcm info */
			c.chunk_bytes = (int)ntohl(aoe->u32opt1);
			c.session_pcm.rate = c.pcm.rate;
			c.session_pcm.format = c.pcm.format;
			c.session_pcm.channels = c.pcm.channels;
			g.key = (uint16_t)ntohs(aoe->u16key);

			P("recv *SYN* from " ETHER_ADDR_FMT ", %s->CONNECTED chunk_bytes:%d, send *ACK*",
				ETHER_ADDR_PTR((struct ether_addr *)&eh->ether_shost),
				Connects[g.stat], c.chunk_bytes);

			g.stat = CONNECTED;
			g.poll_timeout_ms = -1;

			// update dst only
			if (g.options & OPT_AUTO) {
				memcpy(dst, &eh->ether_shost, ETH_ALEN);
				memcpy(g.sll.sll_addr, dst, ETH_ALEN);
			}
			if (aoe->u8sub)
				g.options |= OPT_CSUM;

			/* C_ACK (Acknowledge)
			 *	u8cmd	C_ACK
			 *	u8sub	0 (not use)
			 *	u32opt1	0 (not use)
			 *	u32opt2	0 (not use)
			 */
			char * payload = "Acknowledge";
			tx_pkt = prepare_tx_packet(ONESHOT, src, dst, g.key, C_ACK, 0, 0, 0, strlen(payload));
			memcpy(tx_pkt->body, payload, strlen(payload));
			send_packet(tx_pkt);
			break;
		case C_REPORT:	/* CLIENT side */
			P("recv *REPORT(%u)* (%uB)", aoe->u8sub, ntohs(aoe->u16len));

			/* recieve Model Spec Report */
			if (aoe->u8sub == MODEL_SPEC) {
				struct ether_addr _tmp;
				memcpy(&_tmp, &eh->ether_shost, 6);
				if (g.options & OPT_AUTO || memcmp(&_tmp, dst, 6) == 0) {
					memcpy(dst, &_tmp, ETH_ALEN);
					memcpy(g.sll.sll_addr, dst, ETH_ALEN);
					struct vsound_model * model = (struct vsound_model*)rx_payload;
					ioctl(g.read_fd, IOCTL_VSOUND_APPLY_MODEL, model);
					P("Apply the model spec (name:%s)", model->name);
				}
				c.model_req_ts.tv_sec = 0; // complete
				break;
			}

			int _fd = open("/run/report", O_CREAT|O_WRONLY|O_APPEND, S_IRUSR|S_IWUSR|S_IRGRP|S_IROTH);
			if (_fd) {
				if (aoe->u8sub == SIGUSR1) {
					char _buf[200];
					sprintf(_buf,
						"TARGET [" ETHER_ADDR_FMT "]\n"
						"\n"
						"  AoE STATUS : %s\n"
						"  AoE SESSION: %6u\n"
						"  AoE VSOUND : %s(%d) (buffer:%ldB timeout:%lu eof:%lu intr:%lu outgo:%lu)\n",
						ETHER_ADDR_PTR((struct ether_addr*)&eh->ether_shost),
						Connects[ntohl(aoe->u32opt1)],
						ntohs(aoe->u16key),
						snd_pcm_state_name(c.pcm.state), c.pcm.state,
						c.pcm.buffer_bytes, c.readtimeout_count, c.eof_count, c.eintr_count, c.outgoing_count);
					write(_fd, _buf, strlen(_buf));
				}
				write(_fd, rx_payload, ntohs(aoe->u16len));
				close(_fd);
			}
			break;
		case C_CLOSE:
			if (g.stat != CONNECTED || g.key != ntohs(aoe->u16key))
				break;
			P("recv *Close* %s->CLOSED", Connects[g.stat]);
			g.stat = CLOSED;
			g.key = 0;
			c.last_mixer_value = 0;
			c.last_dreq_opt1 = 0;
			c.last_dreq_opt2 = 0;
			break;
		case C_DRES: {	/* SERVER side */ }
		case C_DISCOVER:/* SERVER side */
		case C_ACK: 	/* SERVER side */
		case C_QUERY: { /* SERVER side */ }
		default:
			P("UNKNOWN COMMAND (cmd:%d)", aoe->u8cmd);
		}
		return 1;
	}
	return 0;
}

int main(int arc, char **argv)
{
	struct ifreq ifr;
	struct ether_addr src, dst;
	g = (struct globals){ 0 };
	g.stat = CLOSED;
	g.options |= OPT_AUTO;
	g.poll_timeout_ms = DEFAULT_TIMEOUT_MS;
	strcpy(g.if_name, "eth0");

	int ch;
	while ((ch = getopt(arc, argv, "hvi:d:s")) != -1) {
		switch(ch) {
		default:
		case 'h':
		case 'v':
			usage(0);
			break;
		case 'i':	/* interface */
			if (strlen(optarg) > MAX_IFNAMELEN - 8) {
				P("if_name too long %s", optarg);
				break;
			}
			strcpy(g.if_name, optarg);
			break;
		case 'd':	/* dst mac address */
			g.options &= ~OPT_AUTO;
			ether_aton_r(optarg, &dst);
			break;
#ifdef CSUM
		case 's':
			g.options |= OPT_CSUM;
			break;
#endif
		}
	}

	/* set signal */
	signal(SIGINT, signal_handler);
	signal(SIGTERM, signal_handler);
	signal(SIGABRT, signal_handler);
	signal(SIGUSR1, sigusr_handler);
	signal(SIGUSR2, sigusr_handler);
	signal(SIGRTMIN, sigusr_handler);
	signal(SIGRTMIN+1, sigusr_handler);
	signal(SIGRTMAX-14, sigusr_handler);
	signal(SIGRTMAX-13, sigusr_handler);
	signal(SIGRTMAX-12, sigusr_handler);
	signal(SIGRTMAX-11, sigusr_handler);

	/* open Virtual Sound Card */
	g.read_fd = open("/dev/vsound", O_RDONLY);
	if (g.read_fd < 0)
		teardown("vsound open error");

	/* mac address (dst) */
	if (g.options & OPT_AUTO)
		memset(&dst, 0xff, 6);

	/* open socket */
	g.sock_fd = socket(PF_PACKET, SOCK_RAW, htons(ETHERTYPE_LE2));
	if (g.sock_fd == -1)
		teardown("socket open error");

	/* mac address (src) */
	memset(&ifr, 0, sizeof(ifr));
	strncpy(ifr.ifr_name, g.if_name, IFNAMSIZ);
	if (ioctl(g.sock_fd, SIOCGIFHWADDR, &ifr) == -1)
		teardown("SIOCGIFHWADDR fail");
	memcpy(&src, &ifr.ifr_hwaddr.sa_data, 6);
	P("aoecli %s [%s] HWaddr " ETHER_ADDR_FMT,
		AOE_VERSION, g.if_name, ETHER_ADDR_PTR(&src));
	P("Destination MAC address " ETHER_ADDR_FMT, ETHER_ADDR_PTR(&dst));

	/* wait for link up */
	for (int retry_count = 0; retry_count < MAX_RETRY_COUNT; retry_count++) {
		if (ioctl(g.sock_fd, SIOCGIFFLAGS, &ifr) == -1)
			teardown("SIOCGIFFLAGS fail");
		if (ifr.ifr_flags & IFF_RUNNING) {
			P("[%s] link is up! (%d ms)", g.if_name, retry_count * RETRY_INTERVAL_US/1000);
			if (retry_count > 0)
				usleep(RETRY_INTERVAL_US * 2); // last sleep
			break;
		}
		usleep(RETRY_INTERVAL_US);
	}

	/* MTU */
	if (ioctl(g.sock_fd, SIOCGIFMTU, &ifr) == -1)
		teardown("SIOCGIFMTU fail");
	P("MTU: %d", ifr.ifr_mtu);

	/* sockaddr_ll (dest) */
	memset(&g.sll, 0, sizeof(g.sll));
	g.sll.sll_family = AF_PACKET;	/* allways AF_PACKET */
	g.sll.sll_protocol = htons(ETHERTYPE_LE2);
	g.sll.sll_ifindex = if_nametoindex(g.if_name);
	g.sll.sll_hatype = 0;
	g.sll.sll_pkttype = 0;
	g.sll.sll_halen = ETH_ALEN;
	memcpy(g.sll.sll_addr, &dst, ETH_ALEN);

	/* bind */
/*	if (bind(g.sock_fd, (struct sockaddr *)&g.sll, sizeof(struct sockaddr_ll)) < 0)
		teardown("bind fail");
*/
	/* epoll */
	int nfds;
	struct epoll_event event, events[MAX_EVENTS];
	g.epoll_fd = epoll_create1(0);
	if (g.epoll_fd == -1)
		teardown("epoll_create1 fail");
	event.events = EPOLLIN;
	event.data.fd = g.sock_fd;
	if (epoll_ctl(g.epoll_fd, EPOLL_CTL_ADD, g.sock_fd, &event) == -1)
		teardown("epoll_ctl fail");

	/* Model Spec Request immidiately */
	c.model_req_ts.tv_sec = 1;

	/* main loop */
	unsigned int timeout_count = 0;
	while (1) {
		clock_gettime(CLOCK_MONOTONIC, &c.cur_ts);

		/* Model Spec Request */
		if (c.model_req_ts.tv_sec != 0 && c.model_req_ts.tv_sec <= c.cur_ts.tv_sec) {
			send_query_model(&src, &dst);
			c.model_req_ts = c.cur_ts;
			c.model_req_ts.tv_sec += MODEL_INTERVAL_SEC; // next time
		}

		/* read vsound status */
		ssize_t avail_bytes = read_stats();
		if (g.stat == CONNECTED && (
				 c.pcm.format != c.session_pcm.format
			  || c.pcm.rate != c.session_pcm.rate
			  || c.pcm.channels != c.session_pcm.channels) ) {
			P("send *Close* pcm information has been modified.");
			send_close(&src, &dst);
		} else {
			switch (c.pcm.state) {
//			case SND_PCM_STATE_PREPARED:
			case SND_PCM_STATE_DRAINING:
			case SND_PCM_STATE_RUNNING:
				if (g.stat == CLOSED && avail_bytes > 0) {
					if (timespec_diff_ns(c.last_ts, c.cur_ts) > BROADCAST_MS * 1000000) {
						if (++timeout_count > TIMEOUT_LIMITS) {
							P("Neighbor Discovery reach the upper limit!, PCM Suspended");
							timeout_count = 0;
							/* pcm suspend */
							ioctl(g.read_fd, IOCTL_VSOUND_STOP_IMMEDIATELY, NULL);
							break;
						}

						/* C_DISCOVER (Neighbor Discovery Broadcast)
						 *	u8cmd	C_DISCOVER
						 *	u8sub	0 (not use)
						 *	u32opt1	mtu
						 *	u32opt2	0 (not use)
						 *  payload struct vsound_pcm
						 */
						if (g.options & OPT_AUTO)
							memset(&dst, 0xff, 6); // Broadcast

						struct aoe_packet * tx_pkt = prepare_tx_packet(ONESHOT, &src, &dst, g.key, C_DISCOVER, 0, ifr.ifr_mtu, 0, sizeof(struct vsound_pcm));
						memcpy(tx_pkt->body, &c.pcm, sizeof(struct vsound_pcm));
						send_packet(tx_pkt);

						c.last_ts = c.cur_ts;
						g.poll_timeout_ms = BROADCAST_MS;
						P("send *Neighbor Discovery* (%s) %s %u %u (buffer:%ldB avail:%ldB)",
							snd_pcm_state_name(c.pcm.state), snd_pcm_format_name(c.pcm.format), c.pcm.rate, c.pcm.channels,
							c.pcm.buffer_bytes, avail_bytes);
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
					send_close(&src, &dst);
				}
				break;
			default :
				break;
			}
			/* digital volume */
			if (g.stat == CONNECTED && c.last_mixer_value != c.pcm.mixer_value) {
				c.sig = SIGMIXERCHG;
			}
		}

		/* epoll wait */
		nfds = epoll_wait(g.epoll_fd, events, MAX_EVENTS, g.poll_timeout_ms);
		if (nfds == -1) {
			if (errno != EINTR && g.stat != EXIT)
				g.stat = TEARDOWN;
		} else if (nfds == 0) {
			// timeout
			continue;
		} else {
			// unbox
			for (int i = 0; i < nfds; ++i) {
				if (events[i].data.fd == g.sock_fd) {
					socklen_t rll_size;
					struct sockaddr_ll rll;
					memset(&rll, 0, sizeof(rll));
					rll_size = sizeof(rll);
					recvfrom(g.sock_fd, rx_buf, PACKET_BUFFER_SIZE, 0, (struct sockaddr *)&rll, &rll_size);
					if (rll.sll_pkttype == PACKET_OUTGOING) {
						c.outgoing_count++;
					}
//					recv(g.sock_fd, rx_buf, PACKET_BUFFER_SIZE, 0);
					unbox((struct aoe_packet *)rx_buf, &src, &dst);
				}
			}
		}

		if (unlikely(c.sig > 0)) {
			/* C_QUERY (Query)
			 *	u8cmd	C_QUERY
			 *	u8sub	signal
			 *	u32opt1	mixer_value
			 *	u32opt2	0 (not use)
			 */
			struct ether_addr dummy_host = {0};
			memcpy(&dummy_host, &dst, 6);
			if (c.sig == SIGMIXERCHG) {
				struct vsound_control before = get_vsound_control(&c.last_mixer_value);
				struct vsound_control after = get_vsound_control(&c.pcm.mixer_value);
				P("send *Query* Mixer Change: volume:%d -> %d (%06x)",
					before.volume, after.volume, c.pcm.mixer_value);
				c.last_mixer_value = c.pcm.mixer_value;
			} else if (c.sig == SIGUSR1) {	/* show server status (lsaoe), always broadcast */
				memset(&dummy_host, 0xff, 6);
			}
			char * payload = "Query";
			struct aoe_packet *tx_pkt = prepare_tx_packet(ONESHOT, &src, &dummy_host, g.key, C_QUERY, c.sig, c.pcm.mixer_value, 0, strlen(payload));
			memcpy(tx_pkt->body, payload, strlen(payload));
			send_packet(tx_pkt);
			P("send *Query* (sig:%d)", c.sig);
			c.sig = 0;
		}
		switch (g.stat) {
		case TEARDOWN:
			send_close(&src, &dst);
			P("send *Close* teardown...");
			goto END;
		case EXIT:
			goto END;
		default:
			break;
		}
	}
END:
	teardown(NULL);
	return 0;
}
