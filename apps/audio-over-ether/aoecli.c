#include <signal.h>
#include <errno.h>
#include <stdbool.h>
#include <sys/epoll.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <arpa/inet.h>			/* ntohs */
#include <linux/if_packet.h>    /* sockaddr_ll */
#include <netinet/ether.h>		/* ether_aton_r */
#include <net/if.h>				/* if_nametoindex */
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
#define PREPARED			NULL
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
		"  -d MAC_ADDR    (Optional)Destination MAC address.(Default Auto Detect)\n"
		, cmd, AOE_VERSION);
	exit(errcode);
}
struct globals {
	enum Status stat;
	uint8_t key;
	int poll_timeout_ms;
	int options;
	#define OPT_AUTO	1
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
	uint8_t last_cyclic_seq_no;
	int chunk_bytes;
	int sig;
	struct timespec cur_ts;
	struct timespec last_ts;
	struct vsound_pcm pcm;
	struct vsound_pcm session_pcm;
	uint32_t last_mixer_value;
	unsigned long eintr_count;
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
	/* vsound returns EAGAIN when there is no data. It never returns 0. */
	/* If the return value is smaller than the request, it indicates underrun. */
	int l = 0, r;
	do {
		r = read(fd, buf + l, count - l);
		if (likely(r > 0)) {
			l += r;
			continue;
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
			return l;
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
static void teardown(char *msg)
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
	struct aoe_packet * _pkt, struct ether_addr * src, struct ether_addr * dst,
	uint16_t key, enum command cmd, uint8_t sub, uint8_t opt, uint32_t val)
{
	/* copy ether header */
	if (_pkt == ONESHOT)
		_pkt = (struct aoe_packet*)tx_buf;
	struct ether_header * eh = &_pkt->eh;	/* 14 bytes */
	memcpy(&eh->ether_dhost, dst, 6);
	memcpy(&eh->ether_shost, src, 6);
	eh->ether_type = htons(ETHERTYPE_LE2);

	/* copy AoE header */
	_pkt->aoe.u8key	= key;
	_pkt->aoe.u8cmd	= cmd;
	_pkt->aoe.u8sub	= sub;
	_pkt->aoe.u8opt	= opt;
	_pkt->aoe.u32val = htonl(val);
	return _pkt;
}
static void reset(void)
{
	g.stat = CLOSED;
	g.key = 0;
	g.poll_timeout_ms = DEFAULT_TIMEOUT_MS;
	c.session_pcm = (struct vsound_pcm) {0};
	c.last_mixer_value = 0;
	c.last_cyclic_seq_no = 0;
}
static void send_packet(struct aoe_packet * pkt, void *buf, size_t paylen)
{
	if (buf)
		memcpy(pkt->body, buf, paylen);

	/* Set Null at the end of the buffer */
	if (paylen < AOE_MIN_PAYLEN) {
		memset(pkt->body + paylen, '\0', AOE_MIN_PAYLEN - paylen);
		paylen = AOE_MIN_PAYLEN;
	}
	if (paylen < AOE_MAX_PAYLEN)
		pkt->body[paylen] = '\0';

	int tx_len = sizeof(struct ether_header) + sizeof(struct aoe_header) + paylen;
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
	 *	u8opt	0 (not use)
	 *	u32val	mixer_value
	 */
	struct aoe_packet * _pkt = prepare_tx_packet(ONESHOT, src, dst, g.key, C_QUERY, MODEL_SPEC, 0, 0);
	send_packet(_pkt, PREPARED, 0);
}
static void send_close(struct ether_addr *src, struct ether_addr *dst)
{
	/* C_CLOSE (Close)
	 *	u8cmd	C_CLOSE
	 *	u8sub	0 (not use)
	 *	u8opt	0 (not use)
	 *	u32val	0 (not use)
	 */
	char * payload = "Good Bye";
	struct aoe_packet * _pkt = prepare_tx_packet(ONESHOT, src, dst, g.key, C_CLOSE, 0, 0, 0);
	send_packet(_pkt, payload, strlen(payload));
	reset();
}

static inline int unbox(struct aoe_packet * rx_pkt, struct ether_addr * src, struct ether_addr * dst, ssize_t paylen)
{
	struct aoe_packet * tx_pkt;
	struct ether_header * eh = (struct ether_header *)rx_pkt;
	struct aoe_header * aoe = (struct aoe_header *)(eh + 1);
	char * rx_payload = (char *)(aoe + 1);

	if (eh->ether_type == htons(ETHERTYPE_LE2)) {
		switch (aoe->u8cmd) {
		case C_DREQ: /* CLIENT side */
			if (likely(g.stat == CONNECTED)) {
				if (unlikely(g.key != aoe->u8key))
					break;
#ifdef PACKET_LOSS_EMULATION
				if (rand() % 1000 == 0) {
					P("packet loss emulation *dreq loss* (opt:%u)", aoe->u8opt);
					break;
				}
#endif
			} else {
				/* first dreq check */
				int _chunk_bytes = (int)ntohl(aoe->u32val);
				if (_chunk_bytes == 0)
					break;

				/* update session pcm info */
				c.chunk_bytes = _chunk_bytes;
				c.session_pcm.rate = c.pcm.rate;
				c.session_pcm.format = c.pcm.format;
				c.session_pcm.channels = c.pcm.channels;
				g.key = aoe->u8key;

				P("recv first *Data Request* from " ETHER_ADDR_FMT ", %s->CONNECTED chunk_bytes:%d",
					ETHER_ADDR_PTR((struct ether_addr *)&eh->ether_shost),
					Connects[g.stat], c.chunk_bytes);

				/* update dst only */
				if (g.options & OPT_AUTO) {
					memcpy(dst, &eh->ether_shost, ETH_ALEN);
					memcpy(g.sll.sll_addr, dst, ETH_ALEN);
				}

				/* prepare all packet except for last_cyclic_seq_no */
				for (int i = 0, last = aoe->u8sub; i < last; i++) {
					prepare_tx_packet((struct aoe_packet *)pkt_array[i],
						src, dst, g.key, C_DRES, i, 0, 0);
				}
			}

			/* Check if it's a retry or not. */
			if (unlikely(c.last_cyclic_seq_no == aoe->u8opt)) {
				P("recv retry request! (opt:%u)", aoe->u8opt);
				goto RETRY_REQUEST;
			}
			c.last_cyclic_seq_no = aoe->u8opt;

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
				 *	u8opt	cyclic seq no
				 *	u32val	0 (not use)
				 */
				tx_pkt = (struct aoe_packet *)pkt_array[i];
				tx_pkt->aoe.u8opt = c.last_cyclic_seq_no;
				ret = read_buf(g.read_fd, (void *)tx_pkt->body, c.chunk_bytes, limit_ns);

				if (unlikely(ret < c.chunk_bytes)) {
					P("recv *Data Request*, but there are %dB left (%d:%s), send *Close*",
						ret, errno, strerror(errno));
					send_close(src, dst);
					return 1;
				}
			}
RETRY_REQUEST:
			for (int i = 0, last = aoe->u8sub; i < last; i++) {
#ifdef PACKET_LOSS_EMULATION
				if (rand() % 1000 == 999 && rand() % last == i) {
					P("packet loss emulation *recv loss* (opt:%u)", c.last_cyclic_seq_no);
					continue;
				}
#endif
				send_packet((struct aoe_packet *)pkt_array[i], PREPARED, c.chunk_bytes);
			}

			/* dump first 16 Bytes */
			if (unlikely(g.stat != CONNECTED)) {
				g.stat = CONNECTED;
				struct aoe_packet * pkt = (struct aoe_packet *)pkt_array[0];
				uint8_t * s = (uint8_t *)pkt->body;
				P("%02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x",
					*(s), *(s+1), *(s+2), *(s+3), *(s+4), *(s+5), *(s+6), *(s+7),
					*(s+8), *(s+9), *(s+10), *(s+11), *(s+12), *(s+13), *(s+14), *(s+15));
			}

			break;
		case C_REPORT:	/* CLIENT side */
			P("recv *Report(%u)* (%zdB) ", aoe->u8sub, paylen);

			/* recieve Model Spec Report */
			if (aoe->u8sub == MODEL_SPEC) {
				struct ether_addr _tmp;
				memcpy(&_tmp, &eh->ether_shost, 6);
				if (g.options & OPT_AUTO || memcmp(&_tmp, dst, 6) == 0) {
					memcpy(dst, &_tmp, ETH_ALEN);
					memcpy(g.sll.sll_addr, dst, ETH_ALEN);
					struct vsound_model * model = (struct vsound_model*)rx_payload;
					ioctl(g.read_fd, IOCTL_VSOUND_APPLY_MODEL, model);
					P("Apply the model spec (%s)", model->name);
				}
				c.model_req_ts.tv_sec = 0; // complete
				break;
			}

			int _fd = open("/run/report", O_CREAT|O_WRONLY|O_APPEND, S_IRUSR|S_IWUSR|S_IRGRP|S_IROTH);
			if (_fd) {
				if (aoe->u8sub == SIGUSR1) {
					char _buf[200];
					sprintf(_buf,
						"PLAYER  %s\n"
						"TARGET  [" ETHER_ADDR_FMT "]\n"
						"\n"
						"  AoE STATUS : %s\n"
						"  AoE SESSION: %6u\n"
						"  AoE VSOUND : %s(%d) (buffer:%zdB intr:%lu)\n",
						c.pcm.comm,
						ETHER_ADDR_PTR((struct ether_addr*)&eh->ether_shost),
						Connects[aoe->u8opt],
						aoe->u8key,
						snd_pcm_state_name(c.pcm.state), c.pcm.state,
						c.pcm.buffer_bytes, c.eintr_count);
					write(_fd, _buf, strlen(_buf));
				}
				write(_fd, rx_payload, paylen);
				close(_fd);
			}
			break;
		case C_CLOSE:
			if (g.stat != CONNECTED || g.key != aoe->u8key)
				break;

			if (memcmp(dst, &eh->ether_shost, 6) != 0)
				break;

			P("recv *Close* %s->CLOSED", Connects[g.stat]);
			reset();
			break;
		case C_DRES: {	/* SERVER side */ }
		case C_DISCOVER:/* SERVER side */
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
	mlockall(MCL_CURRENT | MCL_FUTURE);
#ifdef PACKET_LOSS_EMULATION
	srand(time(NULL));
#endif
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
	P("Destination MAC address " ETHER_ADDR_FMT "%s", ETHER_ADDR_PTR(&dst), g.options & OPT_AUTO ? " (Auto Detect)":"");

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
	if (bind(g.sock_fd, (struct sockaddr *)&g.sll, sizeof(struct sockaddr_ll)) < 0)
		teardown("bind fail");

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
		ssize_t start_threshold;
		if (g.stat == CONNECTED && (
				 c.pcm.format != c.session_pcm.format
			  || c.pcm.rate != c.session_pcm.rate
			  || c.pcm.channels != c.session_pcm.channels) ) {
			P("send *Close* pcm information has been modified.");
			P("%s->%s %d->%d %d->%d",
				snd_pcm_format_name(c.session_pcm.format),
				snd_pcm_format_name(c.pcm.format),
				c.session_pcm.rate, c.pcm.rate, c.session_pcm.channels, c.pcm.channels);
			send_close(&src, &dst);
		} else {
			switch (c.pcm.state) {
//			case SND_PCM_STATE_PREPARED:
			case SND_PCM_STATE_DRAINING:
			case SND_PCM_STATE_RUNNING:
				if (g.stat == CLOSED && avail_bytes > 0) {
					 /* 0.2 seconds' worth of buffer for the start_threshold. */
					start_threshold = (c.pcm.rate/5) * c.pcm.channels * snd_pcm_format_physical_width(c.pcm.format)/8;
					start_threshold = c.pcm.buffer_bytes/2 < start_threshold ? c.pcm.buffer_bytes/2 : start_threshold;
					P("%s local_start_threshold:%zd avail_bytes:%zd", c.pcm.comm, start_threshold, avail_bytes);
					if (avail_bytes < start_threshold)
						break;

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
						 *	u8opt	0 (not use)
						 *	u32val	mtu
						 *  payload struct vsound_pcm
						 */
						if (g.options & OPT_AUTO)
							memset(&dst, 0xff, 6); // Broadcast

						struct aoe_packet * tx_pkt = prepare_tx_packet(ONESHOT, &src, &dst, g.key, C_DISCOVER, 0, 0, ifr.ifr_mtu);
						send_packet(tx_pkt, &c.pcm, sizeof(struct vsound_pcm));

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
		} else if (nfds > 0) {
			// unbox
			for (int i = 0; i < nfds; ++i) {
				if (events[i].data.fd == g.sock_fd) {
					ssize_t received_bytes = recv(g.sock_fd, rx_buf, PACKET_BUFFER_SIZE, 0);
					if (received_bytes > 0) {
						unbox((struct aoe_packet *)rx_buf, &src, &dst,
							received_bytes - sizeof(struct ether_header) - sizeof(struct aoe_header));
					}
				}
			}
		}

		if (unlikely(c.sig > 0)) {
			/* C_QUERY (Query)
			 *	u8cmd	C_QUERY
			 *	u8sub	signal
			 *	u8opt	0 (not use)
			 *	u32val	mixer_value
			 */
			struct ether_addr dummy_host = {0};
			memcpy(&dummy_host, &dst, 6);
			if (c.sig == SIGMIXERCHG) {
				struct vsound_control before = get_vsound_control(&c.last_mixer_value);
				struct vsound_control after = get_vsound_control(&c.pcm.mixer_value);
				P("send *Query(%d)* Mixer Change: volume:%d -> %d (%06x)",
					c.sig, before.volume, after.volume, c.pcm.mixer_value);
				c.last_mixer_value = c.pcm.mixer_value;
			} else {
				P("send *Query(%d)* val:%d", c.sig, c.pcm.mixer_value);
			}
			if (c.sig == SIGUSR1) {	/* show server status (lsaoe), always broadcast */
				memset(&dummy_host, 0xff, 6);
			}
			char * payload = "Query";
			struct aoe_packet *tx_pkt = prepare_tx_packet(ONESHOT, &src, &dummy_host, g.key, C_QUERY, c.sig, 0, c.pcm.mixer_value);
			send_packet(tx_pkt, payload, strlen(payload));
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
