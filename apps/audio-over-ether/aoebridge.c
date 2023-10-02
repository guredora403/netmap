#define NETMAP_WITH_LIBS
#include <libnetmap.h>
#include <limits.h>
#include <signal.h>
#include <errno.h>
#include <stdbool.h>
#include <sys/poll.h>
#include <sys/ioctl.h>
#include <arpa/inet.h>		/* ntohs */
#include <netinet/ether.h>
#include <alsa/asoundlib.h>
#include <alsa/pcm.h>
#include "audio_over_ether.h"
#include "vsound.h"
#include "aoe.h"

#define MAX_ARGLEN		64
#define CAP				64
#define REMOTE_COMMAND	"/usr/local/bin/aoecmd.sh"
#define INITIAL_DREQ_TIMEOUT_US	1000*1000*4
static void usage(int errcode)
{
	const char *cmd = "aoebridge";
	fprintf(stdout,
		"%s %s\n"
		"Usage:\n"
		"  -h             Show program usage and exit.\n"
		"  -i INTERFACE   (Optional)Network interface name. (Default eth0)\n"
//		"  -D DEVICE_NAME (Optional)select PCM by name. (Default hw:0,0)\n"
		, cmd, AOE_VERSION);
	exit(errcode);
}

enum OutputType {I2S, USB};
static struct ext_slot *ext;
struct globals {
	enum Status stat;
	uint8_t key;
	enum OutputType type;
	int poll_timeout_ms;
	int options;
#define OPT_AUTO	1
	char *aoe_buf_addr;
	struct nmport_d *nmd;
	int aoe_buf_fd;
	int read_fd;
	snd_pcm_t *handle;
	char if_name[MAX_IFNAMELEN];
	char pcm_name[MAX_ARGLEN];
	char ip_addr[INET_ADDRSTRLEN];	/* return for aoesshon */
	struct vsound_model model;
};
static struct globals g;

struct event_count {
	unsigned long error;
};
struct stats {
	unsigned long min;
	unsigned long max;
	unsigned long ttl;
	unsigned long count;
};
static char report[1500];
struct server_stats {
	uint8_t cyclic_seq_no;
	struct vsound_pcm pcm;
	int chunk_bytes;
	int cb_frames;
	snd_pcm_uframes_t period_size;
	int req;
	int recv;
	int dreq_limits;
	int dreq_limits_max;
	int last_volume_value;
	int client_mtu;
	int server_mtu;
	bool pcm_started;
	struct netmap_slot * cur_slot;
	struct timespec cur_ts;
	struct timespec last_ts;
	struct event_count ev;
	struct stats poll_stats;
	unsigned long recv_total;
	unsigned long dreq_timeout;
	unsigned long dreq_retry;
#ifdef SERVER_STATS
	struct stats trip_stats[CAP];
	unsigned long forward_total;
	unsigned long poll_total;
	unsigned long req_count;
	unsigned long trip_us_total;
	unsigned long act_ns_total;
	int poll_count;
#endif
};
static struct server_stats s;

static void reset(void)
{
#ifdef SERVER_STATS
	s.poll_count = 0;
#endif
	s.req = 0;
	s.recv = 0;
	__atomic_store_n(&ext->unarrived_dreq_packets, 0, __ATOMIC_RELEASE);
}
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

static void teardown(char *msg)
{
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

	if (g.aoe_buf_addr) {
		__atomic_store_n(&ext->unarrived_dreq_packets, 0, __ATOMIC_RELEASE);
		munmap(g.aoe_buf_addr, AOE_BUF_SIZE);
		P("teardown ... aoe buffer unmaped.");
	}
	if (g.aoe_buf_fd) {
		close(g.aoe_buf_fd);
		P("teardown ... aoe buffer fd closed.");
	}

	/* alsa */
	if (g.handle) {
		snd_pcm_drain(g.handle);
		snd_pcm_close(g.handle);
	}
	snd_config_update_free_global();

	P("teardown ... pcm exit.");
	if (msg != NULL) {
		P("EXIT stat:%-9s %s", Connects[g.stat], msg);
		exit(-1);
	} else {
		P("EXIT stat:%-9s", Connects[g.stat]);
	}
}

static size_t update_packet(struct aoe_packet * pkt, struct ether_addr * src, struct ether_addr * dst,
		enum command cmd, uint8_t sub, uint8_t opt, uint32_t val, size_t paylen)
{
	/* copy ether header */
	struct ether_header * eh = &pkt->eh;	/* 14 bytes */
	memcpy(&eh->ether_dhost, dst, 6);
	memcpy(&eh->ether_shost, src, 6);
	eh->ether_type = htons(ETHERTYPE_LE2);

	/* Set Null at the end of the buffer */
	if (paylen < AOE_MIN_PAYLEN) {
		memset(pkt->body + paylen, '\0', AOE_MIN_PAYLEN - paylen);
		paylen = AOE_MIN_PAYLEN;
	}
	if (paylen < AOE_MAX_PAYLEN)
		pkt->body[paylen] = '\0';

	/* copy AoE header */
	pkt->aoe.u8key	= g.key;
	pkt->aoe.u8cmd	= cmd;
	pkt->aoe.u8sub	= sub;
	pkt->aoe.u8opt	= opt;
	pkt->aoe.u32val = htonl(val);
	return paylen;
}

/*
 * Search for the available TX buffer, set the Ether header and AoE header,
 * and return the starting address of the buffer.
 * Pay attention to setting the payload length at this point.
 */
static struct aoe_packet * prepare_tx_packet(struct ether_addr * src, struct ether_addr * dst,
		enum command cmd, uint8_t sub, uint8_t opt, uint32_t val, size_t paylen)
{
	int i;
	struct netmap_if * nifp = g.nmd->nifp;
	struct netmap_ring * txring = NULL;

	/* scan our queues and send on those with room */
	for (i = g.nmd->first_tx_ring; i < g.nmd->last_tx_ring; i++) {
		txring = NETMAP_TXRING(nifp, i);
		if (unlikely(nm_ring_empty(txring)))
			continue;

		u_int head = txring->head;
		struct netmap_slot *slot = &txring->slot[head];

		struct aoe_packet *pkt = (struct aoe_packet *)NETMAP_BUF(txring, slot->buf_idx);
		paylen = update_packet(pkt, src, dst, cmd, sub, opt, val, paylen);

		/* Please be aware that the payload length may not meet the minimum size of the Ethernet frame. */
		int size = sizeof(struct ether_header) + sizeof(struct aoe_header) + paylen;
		slot->len = size;
		slot->flags = NS_REPORT;
		txring->head = txring->cur = nm_ring_next(txring, head);
		return pkt;
	}
	/* TODO: There is no available buffer in the transmission ring. */
	return NULL;
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

static inline void * idx_to_bufp (unsigned long idx)
{
	return (void *)g.aoe_buf_addr + NM_BUFSZ * AOE_BUF_IDX(idx);
}

static void swapto(int is_hwring, struct netmap_slot * rxslot)
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

/* ALSA */
#define OPENMODE	0   /* block */
#define NOT_SUPPORT	-1
#define NUM_FORMATS	3
#define NUM_RATES	10
#define SNDRV_PCM_FMTBIT(fmt)      (1ULL << (int)fmt)
int avail_rate[NUM_RATES] = {44100, 48000, 88200, 96000, 176400, 192000, 352800, 384000, 705600, 768000};
unsigned int sndrv_pcm_rate[NUM_RATES] =
	{(1U<<6), (1U<<7), (1U<<9), (1U<<10), (1U<<11), (1U<<12), (1U<<13), (1U<<14), (1U<<15), (1U<<16)};
//snd_pcm_format_t avail_format[NUM_FORMATS] = {SND_PCM_FORMAT_S16, SND_PCM_FORMAT_S24, SND_PCM_FORMAT_S32};

static void pcm_rate_changed (snd_pcm_format_t format,
		unsigned int rate, unsigned int channels, ssize_t buffer_bytes)
{
	s.pcm.format = format;
	s.pcm.rate = rate;
	s.pcm.channels = channels;
	s.pcm.buffer_bytes = buffer_bytes;

	const size_t bytes_per_sample = snd_pcm_format_physical_width(format)/8;

	/* Adjust to the minimum MTU. */
	int target_mtu = MTU_ETH;
	if (target_mtu > s.client_mtu)
		target_mtu = s.client_mtu;
	if (target_mtu > s.server_mtu)
		target_mtu = s.server_mtu;

	ssize_t packet_buffer_bytes = target_mtu - sizeof(struct aoe_header);

	/* Consider the case where the buffer size of vsound is smaller than (MTU - AoE header). */
	if (buffer_bytes < packet_buffer_bytes) {
		packet_buffer_bytes = buffer_bytes;
		packet_buffer_bytes -= buffer_bytes % (channels * bytes_per_sample);
	}

	s.period_size = packet_buffer_bytes / (channels * bytes_per_sample);
	s.period_size -= s.period_size % 2;
	s.chunk_bytes = s.period_size * channels * bytes_per_sample;
	s.pcm.period_us = 1000000 * s.period_size / rate;

	/* Consideration to ensure that DREQ does not exceed the buffer size of vsound. */
	int req_limits = (buffer_bytes/2) / s.chunk_bytes;
	s.dreq_limits_max = MIN(buffer_bytes / s.chunk_bytes, 64);

	/*
	 * Note that ext->num_slots is four times cb_frames.
	 * Since the maximum value for num_slots is 1024, cb_frames can be up to a maximum of 256.
	 */
	if (s.pcm.format == SND_PCM_FORMAT_S16) {
		/* for S16 */
		switch (rate) {
		case 48000:
		case 44100:
			s.cb_frames = 16;
			req_limits = (req_limits < 6) ? req_limits:6;
			break;
		case 96000:
		case 88200:
			s.cb_frames = 32;
			req_limits = (req_limits < 12) ? req_limits:12;
			break;
		case 192000:
		case 176400:
			s.cb_frames = 64;
			req_limits = (req_limits < 24) ? req_limits:24;
			break;
		case 384000:
		case 352800:
			s.cb_frames = 128;
			req_limits = (req_limits < 48) ? req_limits:48;
			break;
		default:
			// 768000, 705600, and others
			s.cb_frames = 256;
			req_limits = (req_limits < 64) ? req_limits:64;
			break;
		}
	} else {
		/* for S24, S32 */
		switch (rate) {
		case 48000:
		case 44100:
			s.cb_frames = 32;
			req_limits = (req_limits < 12) ? req_limits:12;
			break;
		case 96000:
		case 88200:
			s.cb_frames = 64;
			req_limits = (req_limits < 24) ? req_limits:24;
			break;
		case 192000:
		case 176400:
			s.cb_frames = 128;
			req_limits = (req_limits < 48) ? req_limits:48;
			break;
		case 384000:
		case 352800:
			s.cb_frames = 256;
			req_limits = (req_limits < 64) ? req_limits:64;
			break;
		default:
			// 768000, 705600, and others
			s.cb_frames = 256;
			req_limits = (req_limits < 64) ? req_limits:64;
			break;
		}
	}
	s.dreq_limits = req_limits;
}

void detectSoundModule(void) {
    FILE *file = fopen("/proc/asound/modules", "r");
    if (file == NULL) {
        teardown("Failed to open /proc/asound/modules");
    }

    char line[256];
    int moduleNumber = -1;

    while (fgets(line, sizeof(line), file)) {
        int currentModuleNumber;
        char currentModuleName[256];

        if (sscanf(line, "%d %s", &currentModuleNumber, currentModuleName) == 2) {
            if (strcmp(currentModuleName, "snd_usb_audio") == 0) {
                moduleNumber = currentModuleNumber;
                break;
            }
        }
    }

    fclose(file);

	if (moduleNumber > 0) {
		sprintf(g.pcm_name, "hw:%d", moduleNumber);
		g.type = USB;
	} else {
		strcpy(g.pcm_name, "hw:0");
		g.type = I2S;
	}
    return;
}

static void spec(void)
{
	snd_pcm_format_t _format = SND_PCM_FORMAT_UNKNOWN;
	unsigned int _rate = 0;
	unsigned int _channels = 0;
	snd_pcm_hw_params_t *params;

	/* initialize */
	g.model = (struct vsound_model){ 0 };

	if (g.handle) {
		snd_pcm_drain(g.handle);
		snd_pcm_close(g.handle);
	}
	int retry_count = 10;
	while(snd_pcm_open(&g.handle, g.pcm_name, SND_PCM_STREAM_PLAYBACK, OPENMODE) < 0) {
		P("spec1 loop %d %d", retry_count, errno);
		if (--retry_count == 0) {
			P("%s", g.pcm_name);
			teardown("snd_pcm_open failed (spec)");
		}
		usleep(50*1000);
	}

	/* name */
	snd_pcm_info_t *pcminfo;
	snd_pcm_info_alloca(&pcminfo);
	snd_pcm_info_set_device(pcminfo, 0);
	snd_pcm_info_set_subdevice(pcminfo, 0);
	snd_pcm_info_set_stream(pcminfo, SND_PCM_STREAM_PLAYBACK);
	int err = snd_pcm_info(g.handle, pcminfo);
	if (err < 0) {
		teardown("snd_pcm_info failed");
	}
	strcpy(g.model.name, snd_pcm_info_get_name(pcminfo));

	P("[%s] %s", g.pcm_name, g.model.name);

	/* hw_params */
	snd_pcm_hw_params_alloca(&params);
	snd_pcm_hw_params_any(g.handle, params);

	int i;
	P("Available formats:");
	for (i = 0; i <= SND_PCM_FORMAT_LAST ; i++) {
		if (snd_pcm_hw_params_test_format(g.handle, params, i) == 0) {
			g.model.formats |= SNDRV_PCM_FMTBIT(i);
			P("- %s", snd_pcm_format_name(i));
			if (_format == SND_PCM_FORMAT_UNKNOWN)
				_format = i;
		} else {
//			avail_format[i] = NOT_SUPPORT;
		}
	}
	P("Available rates:");
	for (i = 0; i < NUM_RATES ; i++) {
		if (snd_pcm_hw_params_test_rate(g.handle, params,(unsigned int)avail_rate[i], SND_PCM_STREAM_PLAYBACK) == 0) {
			g.model.rates |= sndrv_pcm_rate[i];
			g.model.rate_max = avail_rate[i];
			P("- %d", avail_rate[i]);
			if (_rate == 0) {
				_rate = avail_rate[i];
				g.model.rate_min = avail_rate[i];
			}
		} else {
//			avail_rate[i] = NOT_SUPPORT;
		}
	}
	if (snd_pcm_hw_params_test_channels(g.handle, params, 2) == 0) {
//		P("Stereo channel supported.");
		_channels = 2;
		g.model.channels_min = 2;
		g.model.channels_max = 2;
	} else {
		teardown("Stereo channel not supported!");
	}

	/* buffer size check */
	snd_pcm_uframes_t buffer_size_max;
	snd_pcm_hw_params_get_buffer_size_max(params, &buffer_size_max);
	if ((g.model.rate_max >= 352800 && buffer_size_max < 380928/4)
		|| (g.model.rate_max >= 176400 && buffer_size_max < 190464/4))
		P("The hardware buffer size is insufficient! buffer_size_max:%lu", buffer_size_max);
//	P("buffer_size_max:%lu", buffer_size_max);

	pcm_rate_changed(_format, _rate, _channels, 4096);
}
static int pcm_set_params(void)
{
	int err;
	snd_pcm_hw_params_t *params;
	snd_pcm_sw_params_t *swparams;
	snd_pcm_hw_params_alloca(&params);
	snd_pcm_sw_params_alloca(&swparams);
	err = snd_pcm_hw_params_any(g.handle, params);
	if (err < 0) {
		P("Broken configuration for this PCM: no configurations available");
		return -1;
	}

	snd_pcm_access_mask_t *mask = alloca(snd_pcm_access_mask_sizeof());
	snd_pcm_access_mask_none(mask);
	snd_pcm_access_mask_set(mask, SND_PCM_ACCESS_MMAP_INTERLEAVED);
	err = snd_pcm_hw_params_set_access_mask(g.handle, params, mask);
	if (err < 0) {
		P("Access type not available");
		return -1;
	}
	err = snd_pcm_hw_params_set_format(g.handle, params, s.pcm.format);
	if (err < 0) {
		P("Sample format non available");
		return -1;
	}
	err = snd_pcm_hw_params_set_channels(g.handle, params, s.pcm.channels);
	if (err < 0) {
		P("Channels count non available");
		return -1;
	}

	err = snd_pcm_hw_params_set_rate_near(g.handle, params, &s.pcm.rate, 0);
	snd_pcm_uframes_t buffer_size =  s.period_size * s.cb_frames;
	err = snd_pcm_hw_params_set_period_size_near(g.handle, params, &s.period_size, 0);
	err = snd_pcm_hw_params_set_buffer_size_near(g.handle, params, &buffer_size);
	err = snd_pcm_hw_params(g.handle, params);
	if (err < 0) {
		P("Unable to set hw params for playback: %s (format:%d)", snd_strerror(err), s.pcm.format);
		return -1;
	}

	snd_pcm_sw_params_current(g.handle, swparams);
	err = snd_pcm_sw_params_set_avail_min(g.handle, swparams, s.period_size);
	err = snd_pcm_sw_params_set_start_threshold(g.handle, swparams, buffer_size);
	if (snd_pcm_sw_params(g.handle, swparams) < 0) {
		P("unable to install sw params:");
		return -1;
	}
	return 0;
}

static void pcm_ready(void)
{
	if (g.handle) {
		snd_pcm_drain(g.handle);
		snd_pcm_close(g.handle);
	}
	int err = snd_pcm_open(&g.handle, g.pcm_name, SND_PCM_STREAM_PLAYBACK, OPENMODE);
	if (err < 0)
		teardown("snd_pcm_open failed");
	if (pcm_set_params() < 0)
		teardown("pcm_set_params failed");
}

static void send_dreq(struct ether_addr *src, struct ether_addr *dst, ssize_t space, unsigned int buffered_playback_us, bool retry)
{
	/* C_DREQ (Data Request)
	 *	u8cmd	C_DREQ
	 *	u8sub	Number of Request Chunks
	 *	u8opt	cyclic seq no
	 *	u32val	chunk_bytes (first request only)
	 *  payload buffered playback us
	 *			Calculate the playable time from the buffered slots and include it in the payload.
	 */
	s.recv = 0;
	s.last_ts = s.cur_ts;
	if (!retry) {
//		s.req = space < s.dreq_limits ? space:s.dreq_limits;
		s.req = space < s.dreq_limits_max ? space:s.dreq_limits_max;
		s.cyclic_seq_no++;
	}
	uint32_t _val = 0;
	if (!s.pcm_started && s.cyclic_seq_no == 1)
		_val = (uint32_t)s.chunk_bytes;

	__atomic_store_n(&ext->unarrived_dreq_packets, s.req, __ATOMIC_RELEASE);
	struct aoe_packet * _pkt = prepare_tx_packet(src, dst, C_DREQ, s.req, s.cyclic_seq_no, _val, sizeof(buffered_playback_us));
	memcpy(_pkt->body, &buffered_playback_us, sizeof(buffered_playback_us));
}
static void send_model_report(struct ether_addr *src, struct ether_addr *dst)
{
	/* C_REPORT (Report)
	 *	u8cmd	C_REPORT
	 *	u8sub	sig
	 *	u8opt	AoE Status
	 *	u32val	0 (not use)
	 */
	struct aoe_packet * _pkt = prepare_tx_packet(src, dst, C_REPORT, MODEL_SPEC, 0, 0, sizeof(struct vsound_model));
	memcpy(_pkt->body, &g.model, sizeof(struct vsound_model));
//	ioctl(g.nmd->fd, NIOCTXSYNC, NULL);
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
	struct aoe_packet * _pkt = prepare_tx_packet(src, dst, C_CLOSE, 0, 0, 0, strlen(payload));
	memcpy(_pkt->body, payload, strlen(payload));

	g.stat = CLOSED;
	g.key = 0;
	g.poll_timeout_ms = DEFAULT_TIMEOUT_MS;
	reset();
}

static inline int unbox(void * buf, struct ether_addr * src, struct ether_addr * dst)
{
	struct aoe_packet * _pkt;
	struct netmap_slot * const slot = s.cur_slot;
	struct ether_header * eh = (struct ether_header *)buf;
	struct aoe_header * aoe = (struct aoe_header *)(eh + 1);
	char * rx_payload = (char *)(aoe + 1);
	if (eh->ether_type == htons(ETHERTYPE_LE2)) {
		switch (aoe->u8cmd) {
		case C_DRES: { /* SERVER side */ }
			if (g.stat != CONNECTED || g.key != aoe->u8key || s.cyclic_seq_no != aoe->u8opt)
				break;
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
				unsigned int _trip = timespec_diff_ns(s.last_ts, s.cur_ts)/1000;
				s.trip_us_total += _trip;
				calc_stats(&s.trip_stats[s.req-1], _trip);
				s.poll_total += s.poll_count;
				s.req_count++;
#endif
				s.recv_total += s.recv;
				reset();
			} else {
				/* Update the number of undelivered packets. */
				__atomic_store_n(&ext->unarrived_dreq_packets, s.req - s.recv, __ATOMIC_RELEASE);
			}
			break;
		case C_DISCOVER: /* SERVER side */
			if (g.stat != CLOSED)
				break;

			/* new session key (Do not use zero. Anything is fine, but here it's 100.) */
			clock_gettime(CLOCK_MONOTONIC, &s.last_ts);
			g.key = (uint8_t)s.last_ts.tv_nsec;
			g.key = g.key == 0 ? 100:g.key;
			P("recv *Neighbor Discovery* from " ETHER_ADDR_FMT ", send first *Data Request*", ETHER_ADDR_PTR(dst));

			// mtu
			s.client_mtu = ntohl(aoe->u32val);

			// update dst
			memcpy(dst, &eh->ether_shost, 6);

			// format/rate/channels
			struct vsound_pcm * pcm = (struct vsound_pcm*)rx_payload;
			snd_pcm_format_t _format = pcm->format;
			unsigned int _rate = pcm->rate;
			unsigned int _channels = pcm->channels;
			ssize_t _buffer_bytes = pcm->buffer_bytes;
			if (s.pcm.format != _format
				|| s.pcm.rate != _rate
				|| s.pcm.channels != _channels
				|| s.pcm.buffer_bytes != _buffer_bytes) {

				// When changing rates, wait until playback is complete.

				while (__atomic_load_n(&ext->stat, __ATOMIC_ACQUIRE) == ACTIVE || snd_pcm_state(g.handle) == SND_PCM_STATE_DRAINING)
					usleep(1000 * 10);

				__atomic_store_n(&ext->head, 0, __ATOMIC_RELEASE);
				__atomic_store_n(&ext->cur, 0, __ATOMIC_RELEASE);
				__atomic_store_n(&ext->tail, 0, __ATOMIC_RELEASE);

				pcm_rate_changed(_format, _rate, _channels, _buffer_bytes);
				pcm_ready();
				ext->num_slots = s.cb_frames * 8;
				P("pcminfo changed %s %u %u chunk_bytes:%d period_us:%u buffer:%ldB(%ldms)",
					snd_pcm_format_name(s.pcm.format), s.pcm.rate, s.pcm.channels,
					s.chunk_bytes, s.pcm.period_us, _buffer_bytes,
					(s.pcm.period_us*_buffer_bytes/s.chunk_bytes)/1000);
			}

			g.poll_timeout_ms = DEFAULT_TIMEOUT_MS;
			g.stat = CONNECTED;
			s.pcm_started = false;
#ifdef SERVER_STATS
			s.poll_count = 0;
#endif
			/* initial dreq */
			s.cyclic_seq_no = 0;
//			unsigned int buffered_playback_us =  ext->num_slots * s.pcm.period_us;
			unsigned int buffered_playback_us = INITIAL_DREQ_TIMEOUT_US;
			send_dreq(src, dst, ext->num_slots -1 , buffered_playback_us, false);

			break;
		case C_QUERY: { /* SERVER side */ }
			// Ignore consecutive queries
			static struct timespec previous_ts = {0,0};
			struct timespec current_ts;
			clock_gettime(CLOCK_MONOTONIC, &current_ts);
			if (timespec_diff_ns(previous_ts, current_ts) < 50*1000000) {
				break;
			} else {
				previous_ts = current_ts;
			}

			P("recv *Query(%u)* val:%06x " ETHER_ADDR_FMT "->" ETHER_ADDR_FMT,
				aoe->u8sub, ntohl(aoe->u32val),
				ETHER_ADDR_PTR((struct ether_addr*)&eh->ether_shost),
				ETHER_ADDR_PTR((struct ether_addr*)&eh->ether_dhost));
			struct ether_addr query_host = {0};
			memcpy(&query_host, &eh->ether_shost, 6);

			/* Model Spec Report */
			if (aoe->u8sub == MODEL_SPEC) {
				send_model_report(src, &query_host);
				P("send *Report(%d)* model spec (%s)", MODEL_SPEC, g.model.name);
				break;
			} if (aoe->u8sub == SIGRTMIN) {
#ifdef SERVER_STATS
				/* reset counter (aoereset) */
				s.forward_total = 0;
				s.poll_total	= 0;
				s.req_count	= 0;
				s.trip_us_total	= 0;
				s.act_ns_total	= 0;
				memset(s.trip_stats, 0, sizeof(s.trip_stats));
				s.poll_stats = (struct stats){ 0 };
				s.poll_stats.min = ULONG_MAX;
#endif
				s.recv_total	= 0;
				s.dreq_timeout	= 0;
				s.dreq_retry	= 0;
				break;
			} else if (aoe->u8sub >= SIGRTMAX-14){
				int cmd_arg = 0;
				if (aoe->u8sub == SIGMIXERCHG) {
					uint32_t mixer_value = (uint32_t)ntohl(aoe->u32val);
					struct vsound_control p = get_vsound_control(&mixer_value);
					if (s.last_volume_value == p.volume)
						break;
					cmd_arg = s.last_volume_value = p.volume;
				}

				/* exec remote command */
				char cmd[100];
				sprintf(cmd, "%s %d %d ", REMOTE_COMMAND, aoe->u8sub, cmd_arg);
				system(cmd);

				/* aoesshon */
				if (aoe->u8sub == SIGRTMAX-14){
					_pkt = prepare_tx_packet(src, &query_host, C_REPORT, aoe->u8sub, 0, 0, strlen(g.ip_addr));
					memcpy(_pkt->body, g.ip_addr, strlen(g.ip_addr));
				}
				break;
			}
			/* C_REPORT (Report)
			 *	u8cmd	C_REPORT
			 *	u8sub	sig
			 *	u8opt	AoE Status
			 *	u32val	0 (not use)
			 */
			memset(report, '\0', sizeof(report));
			if (aoe->u8sub == SIGUSR1) {
				/* show server status (lsaoe) */
				sprintf(report,
					"  PCM PARAM  : %s %u %u chunk_bytes:%d period_us:%u\n"
					"  AoE STATS  : dreq=%d (dreq timeout:%lu retry:%lu total packets:%lu)\n"
					"  DMA STATS  : %s (cb_frames:%d)\n"
					"  SOUND CARD : %s\n",
					snd_pcm_format_name(s.pcm.format), s.pcm.rate, s.pcm.channels, s.chunk_bytes, s.pcm.period_us,
					s.dreq_limits, s.dreq_timeout, s.dreq_retry, s.recv_total,
					ext->stat == ACTIVE ? "ACTIVE":"INACTIVE", s.cb_frames,
					g.model.name);
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
					"poll/chunk x1000: %6lu\n"
					"dreq/chunk x1000: %6lu (aoe.dreq=%d)\n"
					"trip/chunk (us) : %6lu\n"
					"act /chunk (ns) : %6lu\n",
					s.pcm.period_us,
					s.recv_total+s.forward_total, s.recv_total, s.forward_total,
					s.poll_total,
					_avg, s.poll_stats.min == ULONG_MAX ? 0:s.poll_stats.min, s.poll_stats.max, s.pcm.period_us*s.dreq_limits/1000,
					s.recv_total ? 1000*s.poll_total/s.recv_total:0,
					s.recv_total ? 1000*s.req_count/s.recv_total:0, s.dreq_limits,
					s.recv_total ? s.trip_us_total/s.recv_total:0,
					s.recv_total ? s.act_ns_total/s.recv_total:0);
				for (int i = 0; i < s.dreq_limits_max; i++) {
					char _buf[50];
					long tavg = s.trip_stats[i].count > 0 ? (long)((long)(s.trip_stats[i].ttl)/s.trip_stats[i].count)/100:0;
					sprintf(_buf, "%2d:%5lu %2ld.%1ld\n", i+1, s.trip_stats[i].count, tavg/10, tavg%10);
					strcat(report, _buf);
				}
#endif
			}
			P("send *Report(%u)* (%luB)", aoe->u8sub, strlen(report));
			_pkt = prepare_tx_packet(src, &query_host, C_REPORT, aoe->u8sub, g.stat, 0, strlen(report));
			memcpy(_pkt->body, report, strlen(report));
			break;
		case C_CLOSE:
			if (g.stat != CONNECTED || g.key != aoe->u8key)
				break;

			P("recv *Close* %s->CLOSED", Connects[g.stat]);
			g.stat = CLOSED;
			g.key = 0;
			reset();
			break;
		case C_DREQ:	/* CLIENT side */
		case C_REPORT:	/* CLIENT side */
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

int main(int arc, char **argv)
{
	struct ether_addr src, dst;
	s = (struct server_stats){ 0 };
	s.client_mtu = MTU_ETH;
	s.server_mtu = MTU_ETH;
	s.pcm_started = false;
#ifdef SERVER_STATS
	s.poll_stats = (struct stats){ 0 };
	s.poll_stats.min = ULONG_MAX;
#endif
	g = (struct globals){ 0 };
	g.stat = CLOSED;
	g.poll_timeout_ms = DEFAULT_TIMEOUT_MS;
	strcpy(g.if_name, "eth0");

	int ch;
	while ((ch = getopt(arc, argv, "hvi:D:s")) != -1) {
		switch(ch) {
		default:
		case 'h':
		case 'v':
			usage(0);
			break;
		case 'i':	/* interface */
			if (strlen(optarg) > MAX_IFNAMELEN - 8) {
				D("ifname too long %s", optarg);
				break;
			}
			strcpy(g.if_name, optarg);
			break;
/*		case 'D':
			if (strlen(optarg) > MAX_ARGLEN) {
				D("device name too long %s", optarg);
				break;
			}
			strcpy(g.pcm_name, optarg);
			break;
*/
		}
	}

	/* set signal */
	signal(SIGINT, signal_handler);
	signal(SIGTERM, signal_handler);
	signal(SIGABRT, signal_handler);

	/* initial pcm setup */
	P("aoebridge %s", AOE_VERSION);

	strcpy(g.pcm_name, "hw:0");
	g.type = I2S;
	spec();
	pcm_ready();

	/* open AoE buffer */
	/* Make sure to open 'aoe_buf' with the I2S DMA buffer in an enabled state. */
	g.aoe_buf_fd = open("/proc/aoe_buf", O_RDWR | O_SYNC);
	if (g.aoe_buf_fd < 0) {
		teardown("aoe_buf open failed");
	}
	g.aoe_buf_addr = mmap(NULL, AOE_BUF_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, g.aoe_buf_fd, 0);
	if (g.aoe_buf_addr == MAP_FAILED) {
		teardown("aoe_buf mmap failed");
	}
	/* ext_slot */
	ext = (struct ext_slot *)((void *)g.aoe_buf_addr + UNIT_BUF_SIZE*4);
	ext->num_slots = s.cb_frames * 8;
	ext->tail = s.req = 0;
	__atomic_store_n(&ext->head, 0, __ATOMIC_RELEASE);
	__atomic_store_n(&ext->cur, 0, __ATOMIC_RELEASE);

	/* mac address (dst) */
	memset(&dst, 0xff, 6);

	/* mac address (src) */
	struct ifreq ifr = (struct ifreq){ 0 };
	strncpy(ifr.ifr_name, g.if_name, IFNAMSIZ);
	int sock_fd = socket(PF_PACKET, SOCK_RAW, htons(ETHERTYPE_LE2));
	if (sock_fd == -1)
		teardown("socket open error");
	if (ioctl(sock_fd, SIOCGIFHWADDR, &ifr) == -1)
		teardown("SIOCGIFHWADDR fail");
	memcpy(&src, &ifr.ifr_hwaddr.sa_data, 6);

	/* MTU */
	if (ioctl(sock_fd, SIOCGIFMTU, &ifr) == -1)
		teardown("SIOCGIFMTU fail");
	s.server_mtu = ifr.ifr_mtu;
	close(sock_fd);

	/* ip addr for report*/
	struct sockaddr_in dst_addr = {0};
	struct sockaddr_in src_addr = {0};
	socklen_t addr_len = sizeof(struct sockaddr_in);
	sock_fd = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
	dst_addr.sin_family = AF_INET;
	dst_addr.sin_port = htons(7);
	inet_aton("128.0.0.0", &dst_addr.sin_addr);
	connect(sock_fd,(struct sockaddr *)&dst_addr,sizeof(dst_addr));
	getsockname(sock_fd, (struct sockaddr *)&src_addr, &addr_len);
	inet_ntop(AF_INET, &src_addr.sin_addr, g.ip_addr, INET_ADDRSTRLEN);
	close(sock_fd);

	/* switch USB-DAC if available */
	/* The switch to USB-DAC should be done after opening 'aoe_buf'. */
	detectSoundModule();
	if (g.type == USB) {
		spec();
		pcm_ready();
	}

	P("[%s] inet addr: %s  HWaddr: "ETHER_ADDR_FMT "  MTU: %d",
		g.if_name, g.ip_addr, ETHER_ADDR_PTR(&src), ifr.ifr_mtu);

	/* nmport */
	char netmapif[100];
	snprintf(netmapif, sizeof(netmapif), "netmap:%s*", g.if_name);
	g.nmd = nmport_prepare(netmapif);
	if (g.nmd == NULL)
		teardown("nmport_prepare failed");
	if (nmport_open_desc(g.nmd) < 0)
		teardown("nmport_open_desc failed");
	if (g.nmd->fd < 0)
		teardown("nmport setup failed");

	/* sanity check buf_idx */
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

	/* Model Spec Report */
	send_model_report(&src, &dst);
	P("send *Report(%d)* model spec (%s)", MODEL_SPEC, g.model.name);

	/* main loop */
	unsigned int cur, n, i;
	unsigned int is_hwring;
	struct netmap_ring *rxring;
	struct pollfd pollfd[1];
	memset(&pollfd, 0, sizeof(pollfd));
	pollfd[0].fd = g.nmd->fd;
	pollfd[0].events = POLLIN;

	uint64_t next_retry_ns = 0;

	while (1) {
		if (likely(g.stat == CONNECTED)) {
			/* buffer control */
			int _tail = __atomic_load_n(&ext->tail, __ATOMIC_ACQUIRE);
			int _cur = __atomic_load_n(&ext->cur, __ATOMIC_ACQUIRE);
			int avail = CIRC_CNT(ext->head, _tail, ext->num_slots);
			int real_avail = CIRC_CNT(ext->head, _cur, ext->num_slots);
			int space = ext->num_slots - avail -1;
			g.poll_timeout_ms = DEFAULT_TIMEOUT_MS;

			/* DREQ */
			if (s.req == 0) {
				if (space > s.dreq_limits/2) {
					/* debug */
					snd_pcm_state_t state = snd_pcm_state(g.handle);
					if (state == SND_PCM_STATE_DRAINING)
						P("dreq... now draining?!");
//					unsigned int buffered_playback_us = avail * s.pcm.period_us;
					unsigned int buffered_playback_us = real_avail * s.pcm.period_us;
					send_dreq(&src, &dst, space, buffered_playback_us, false);
					/* Wake up before a buffer underrun occurs.
					 * However, if there is already limited time remaining,
					 * set a standard timeout with the expectation of an immediate response. */
					if (buffered_playback_us > DEFAULT_TIMEOUT_MS * 1000 * 1.1) {
						g.poll_timeout_ms = (buffered_playback_us / 1000) - DEFAULT_TIMEOUT_MS;
					}
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
				/* retry */
				enum DMAStatus _stat = __atomic_load_n(&ext->stat, __ATOMIC_ACQUIRE);
//				snd_pcm_state_t state = snd_pcm_state(g.handle);
				if (_stat == ACTIVE && 0 < real_avail && real_avail < s.cb_frames) {
					/* Usually, `real_avail` is maintained in a state close to full,
					 * but if it falls below `cb_frames`,
					 * it is considered either a packet loss or a playback stop. */
					struct timespec _ts;
					clock_gettime(CLOCK_MONOTONIC, &_ts);
					if (next_retry_ns < timespec_to_ns(_ts)) {
						P("retry! avail:%d real_avail:%d req:%d recv:%d (opt:%u)",
							avail, real_avail, s.req, s.recv, s.cyclic_seq_no);
						s.dreq_retry++;

						unsigned int buffered_playback_us = real_avail * s.pcm.period_us;
						send_dreq(&src, &dst, space, buffered_playback_us, true);

						/* Consider the timeout period to avoid repeating resend requests,
						 * as no response can be obtained when playback is stopped.*/
						g.poll_timeout_ms = MAX(DEFAULT_TIMEOUT_MS, buffered_playback_us/2000);
						next_retry_ns = timespec_to_ns(_ts) + g.poll_timeout_ms * 1000 * 1000;
					}
				} else if (_stat == INACTIVE && avail == 0 && s.pcm_started) {
//				} else if (_stat == INACTIVE && s.pcm_started &&
//					(state == SND_PCM_STATE_XRUN || state ==SND_PCM_STATE_SETUP)) {
					/* data request timeout */
					P("send *Close* There is no response to the data request, and DMA has stopped. (req:%d recv:%d)", s.req, s.recv);
					send_close(&src, &dst);
					s.dreq_timeout++;
				} else if (_stat == INACTIVE && ! s.pcm_started) {
					/* Is it necessary to retry the initial DREQ? */
//					unsigned int buffered_playback_us =  ext->num_slots * s.pcm.period_us;
//					if (timespec_diff_ns(s.last_ts, s.cur_ts) > buffered_playback_us * 1000) {
					unsigned int buffered_playback_us =  INITIAL_DREQ_TIMEOUT_US;
					if (timespec_diff_ns(s.last_ts, s.cur_ts) > (long)INITIAL_DREQ_TIMEOUT_US * 1000) {
						P("retry initial dreq!? (req:%d recv:%d)", s.req, s.recv);

						/* initial dreq */
						send_dreq(&src, &dst, space , buffered_playback_us, true);
					}
				}
			}
			/* If DMA is inactive, call snd_pcm_start using ioctl. */
			snd_pcm_state_t state = snd_pcm_state(g.handle);
			if (__atomic_load_n(&ext->stat, __ATOMIC_ACQUIRE) == INACTIVE &&
				(state == SND_PCM_STATE_PREPARED || state == SND_PCM_STATE_XRUN || state ==SND_PCM_STATE_SETUP)) {
				const int ext_playable = ext->head >= _cur ? (ext->head - _cur):(ext->num_slots + ext->head - _cur);
				if (ext_playable >= ext->num_slots/2) {
					if (ioctl(g.aoe_buf_fd, IOCTL_PCM_START, &g.type)) {
						P("ioctl error!");
					} else {
						P("ioctl success!(playable:%d)", ext_playable);
						s.pcm_started = true;
					}
				}
			} else if (state == SND_PCM_STATE_DRAINING) {
//				P("draining now!");
			}
		}
#ifdef SERVER_STATS
		/* act_ns_total */
		if (g.stat == CONNECTED) {
			struct timespec _ts;
			clock_gettime(CLOCK_MONOTONIC, &_ts);
			s.act_ns_total += timespec_diff_ns(s.cur_ts, _ts);
		}
		s.poll_count++;
#endif
		int poll_ret = poll(pollfd, 1, g.poll_timeout_ms);
		clock_gettime(CLOCK_MONOTONIC, &s.cur_ts);
		if (poll_ret == 0 && g.stat != EXIT && g.stat != TEARDOWN)
			continue;

		for (i = g.nmd->first_rx_ring; i <= g.nmd->last_rx_ring; i++) {
			is_hwring = (i != g.nmd->last_rx_ring);
			rxring = NETMAP_RXRING(g.nmd->nifp, i);
			cur = rxring->cur;
			for (n = nm_ring_space(rxring); n > 0; n--, cur = nm_ring_next(rxring, cur)) {
				if (is_hwring) {
					void *buf;
					buf = idx_to_bufp(rxring->slot[cur].buf_idx);
					s.cur_slot = &rxring->slot[cur];
					if(likely(unbox(buf, &src, &dst))) {
						continue;
					}
				}
				swapto(is_hwring, &rxring->slot[cur]);
			}
			rxring->head = rxring->cur = cur;
		}

		switch (g.stat) {
		case TEARDOWN:
			send_close(&src, &dst);
			ioctl(g.nmd->fd, NIOCTXSYNC, NULL);
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
