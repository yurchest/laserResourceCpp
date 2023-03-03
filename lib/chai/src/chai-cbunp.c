/*
*  chai-cbunp.c
*  CAN-bus-USBnp CAN Hardware Abstraction Interface for Windows
*
*  Author: Fedor Nedeoglo, 1998-2016
*
*  Marathon Ltd. Moscow, 2016
*
*  Date: 11 Aug 2016
*
*/

#include <stdio.h>
#include "chai.h"
#include "unican.h"
#include "chai-lnx.h"
#include <unistd.h>
#include <pthread.h>
#include <stdlib.h>
#include <ctype.h>
#include <signal.h>
#include <string.h>
#include <sys/timeb.h>
#include <poll.h>
#include <sys/eventfd.h>

#include "chai-cbunp.h"
#include "ftd2xx.h"

//#define DEBUG_CBUNP

#ifdef DEBUG_CBUNP
#define CBU_DEBUG(...) do { printf("cbunp debug: ");\
    printf(__VA_ARGS__); fflush(stdout); } while(0)
#else
#define CBU_DEBUG(...)
#endif

char cbunp_desc_1stchan[] = "CAN-bus-USBnp A";
char cbunp_desc_2ndchan[] = "CAN-bus-USBnp B";
char cbunps_desc[] = "CAN-bus-USBnps A";

//FILE *fcbunp = NULL;
//#define RXBUF_SIZE  32768UL
#define RXBUF_SIZE  2048UL
#define CT_EOF       0xFE
#define CT_CLOSE_APP 0xFD
#define CT_EOTS      0xFC  // END OF TRANSMIT SERIES
#define CT_SEQ_STUB  0x0
#define CT_LEN_MIN   2
#define CT_LEN_MAX   128   // 127 + CT_EOF

#define CT_CMDRESP_TOUT 1000


#define GET_IDENTITY_STRING 0x01
#define CIOPEN              0x02
#define CICLOSE             0x03
#define CISTART             0x04
#define CISTOP              0x05
#define CISETFILTER         0x06
#define CISETBAUD           0x07
#define CIWRITE             0x08
#define CISTATUS            0x09
#define CIHWRESET           0x0A
#define CISETLOM            0x0B
#define CISETWTOUT          0x0C
#define CIRX                0x0D
#define CIERR               0x0E
// add-ons for firmware version since 1.4
#define CITXWAKEUP          0x0F
#define CITXSLEEP           0x10
#define CITRSTAT            0x11
#define CITRCANCEL          0x12
#define CITRANSMIT          0x13
#define CISETTRTRESHOLD     0x14
#define CIREGREAD           0x15
#define CIREGWRITE          0x16
// new add-ons in firmware version 1.5
#define CITRANSMIT_SERIES   0x17
#define CILINKPING          0x18


typedef struct {
    canmsg_t *d;		/*!< data - array of can_msg_t type values */
    _u16 sloc;			/*!< next store element adress (index of array d) */
    _u16 rloc;			/*!< next retrieve element adress (index of array d) */
    _u16 size;			/*!< size of data array */
    _u16 count;			/*!< current number of elements in data array */
} frameque_t;

enum {
    CMD_NOEXIST = 0,
    CMD_WAIT,
    CMD_COMPLETE
};

enum {
    CI_RCSTATE_WAKE = 0,
    CI_RCSTATE_SLEEP = 1
};

typedef struct {
    int code;
    int state;
    _u8 res;
    _u8 seq;
    int dlen;
    _u8 data[CT_LEN_MAX];
} ctcmd_t;

typedef struct {
    FT_HANDLE        ftHandle;
	EVENT_HANDLE	 ftEvent;
	pthread_t        ftThread;
	pthread_mutex_t  trlock;
	pthread_mutex_t  rcqlock;
	pthread_mutex_t  errlock;
	pthread_mutex_t  cmdlock;
	pthread_mutex_t  tract_lock;
	struct pollfd	 cmdEvent;
    _u16             rcq_thres;
    _u16             trq_thres;
    _u16             tout;
    _u32             fmver;
    _u8              sigenable;
    canerrs_t        curerrs;
    _u32             sovr_cnt;
    _u32             hovr_cnt;
    int              rstate;
    frameque_t       *rcq;
    ctcmd_t          cmd;
    _u8              rxbuf[RXBUF_SIZE];
    int              rxcnt;
    _u8              exit_flag;
    _u8              chip_state;
} cbunpdev_t;

static cbunpdev_t *cdev[CI_CHAN_NUMS];

/**********************************************************
*  cbunp device struct and CAN frame queue functions
*/

static frameque_t *fquealloc(_u16 size)
{
    frameque_t *q;

    if ((q = malloc(sizeof(frameque_t))) == NULL)
        return NULL;
    q->d = malloc(sizeof(canmsg_t) * size);
    if (q->d == NULL) {
        free(q);
        return NULL;
    }
    q->sloc = 0;
    q->rloc = 0;
    q->count = 0;
    q->size = size;
    return q;
}

static _s16 fquecncl(frameque_t * q)
{
    if (!q)
        return -1;
    if (q->d)
        free(q->d);
    free(q);
    return 0;
}

static _s16 fqueempty(frameque_t * q)
{
    q->sloc = 0;
    q->rloc = 0;
    q->count = 0;
    return 0;
}

static _s16 fquestor(frameque_t * q, canmsg_t * msg)
{
    if (q->count == q->size) {
		CBU_DEBUG("QUEUE IS FULL!!!!");
        return -1;			// queue is full
    }
    memcpy(q->d + q->sloc, msg, sizeof(canmsg_t));
    q->count++;
    q->sloc += 1;
    if ( q->sloc >= q->size)
        q->sloc=0;
    return 0;
}

static _s16 fqueretr_many(frameque_t * q, canmsg_t * msg, _s16 num)
{
    _s16 i,ret;

    if (q->count == 0) return -1;  // queue is empty

    if(q->count > num) ret = num;
    else ret = q->count;

    for(i=0; i<ret; i++) {
        msg[i] = q->d[q->rloc];
        q->rloc += 1;
        if( q->rloc >= q->size )
            q->rloc=0;
        q->count--;
    }
    return ret;
}


cbunpdev_t *cbunpdev_alloc (void)
{
    cbunpdev_t * dev;

    dev = (cbunpdev_t *) malloc(sizeof(cbunpdev_t));
    if (dev == NULL)
        return NULL;
    dev->rcq = fquealloc(CIQUE_DEFSIZE_RC);
    if (dev->rcq == NULL) {
        free(dev);
        return NULL;
	}
	pthread_mutex_init(&dev->trlock, NULL);
	pthread_mutex_init(&dev->rcqlock, NULL);
	pthread_mutex_init(&dev->errlock, NULL);
	pthread_mutex_init(&dev->cmdlock, NULL);
	pthread_mutex_init(&dev->tract_lock, NULL);
	dev->ftHandle   = NULL;
	pthread_mutex_init(&dev->ftEvent.eMutex, NULL);
	pthread_cond_init(&dev->ftEvent.eCondVar, NULL); dev->ftEvent.iVar = 0;
	dev->cmdEvent.fd = eventfd(0, 0);
	dev->cmdEvent.events = POLLIN;
	dev->cmdEvent.revents = 0;
    dev->trq_thres  = CIQUE_TR_THRESHOLD_DEF;
    dev->rcq_thres  = CIQUE_RC_THRESHOLD_DEF;
    dev->rstate     = CI_RCSTATE_SLEEP;
    dev->tout       = CI_WRITE_TIMEOUT_DEF;
    dev->cmd.state  = CMD_NOEXIST;
    dev->cmd.seq    = 0;
    memset( &dev->curerrs, 0, sizeof(canerrs_t) );
    memset( dev->rxbuf, 0, RXBUF_SIZE );
    dev->rxcnt      = 0;
    dev->exit_flag  = 0;
    dev->sovr_cnt   = 0;
    dev->hovr_cnt   = 0;
    dev->chip_state = CAN_INIT;
    dev->sigenable  = 0;
    return dev;
}

void cbunpdev_free (cbunpdev_t * dev)
{
    fquecncl(dev->rcq);
	pthread_mutex_destroy (&dev->trlock);
	pthread_mutex_destroy (&dev->rcqlock);
	pthread_mutex_destroy (&dev->errlock);
	pthread_mutex_destroy (&dev->cmdlock);
	pthread_mutex_destroy (&dev->tract_lock);

	pthread_mutex_destroy (&dev->ftEvent.eMutex);
	pthread_cond_destroy (&dev->ftEvent.eCondVar);
	dev->cmdEvent.fd = 0;
    free(dev);
}


void cbunp_rc_checkchange_state(int chan)
{
    if (cdev[chan]->rcq->count >= cdev[chan]->rcq_thres) {
		if (cdev[chan]->rstate == CI_RCSTATE_SLEEP) {
			cdev[chan]->rstate = CI_RCSTATE_WAKE;
			eventfd_write(chios[chan].select_evs[CAN_EVRD].fd, 1);
//			printf("eventfd_write: EVRD\n");
        }
    } else {
		if (cdev[chan]->rstate == CI_RCSTATE_WAKE) {
			cdev[chan]->rstate = CI_RCSTATE_SLEEP;
			struct pollfd wait_object;
			u_int64_t event_value;
			int ret;
			if (chios[chan].select_evs[CAN_EVRD].fd > 0) {
				wait_object.fd = chios[chan].select_evs[CAN_EVRD].fd;
				wait_object.events = POLLIN;
				wait_object.revents = 0;
				ret = poll(&wait_object, 1, 0);
				if (ret < 0)
					return;
				if ((wait_object.revents & POLLIN) != 0) {
					ret = eventfd_read(chios[chan].select_evs[CAN_EVRD].fd, &event_value);
					if (ret < 0)
						return;
				}
			}
//			printf("eventfd_reset: EVRD\n");
        }
    }
}

/**********************************************************
*  CAN tunnel functions
*/

int ct_send_frame(int chan, _u8 *frame, int flen)
{
    FT_STATUS ftStatus;
    DWORD BytesTx;

#ifdef DEBUG_CBUNP
    int i;
    printf("TX %2d: ", flen);
    for (i=0; i<flen; i++)
        printf("0x%x ", frame[i]);
    printf("\n"); fflush(stdout);
#endif
//    CBU_DEBUG("chan: %d, ftHandle: %d, frame: %d, flen: %d\n", chan, cdev[chan]->ftHandle, frame, flen);
    for (int i = 0; i < flen; ++i) {
	CBU_DEBUG("Frame[%d] = %d\n", i, frame[i]);
    }
    CBU_DEBUG("FT_Write called...\n");
    ftStatus = FT_Write(cdev[chan]->ftHandle, frame, flen, &BytesTx);
    CBU_DEBUG("FT_Write returned %d\n", ftStatus);
    if (ftStatus != FT_OK) {
        return -1;
    }
    CBU_DEBUG("sending frame OK\n");
    return 0;
}

int ct_decode_data(_u8 *dout, _u8 *dcod, _u8 cdlen)
{
    _u8 lsbn;
    _u8 i, octets, octnum;
    int shift = 0, dlen;

    octets = cdlen >> 3; // equal to cdlen divided by 8
    if (octets > 0) {
        lsbn = 7;
    } else {
        lsbn = cdlen-1;
    }
    octnum = 0;
    dlen = 0;
    shift = 0;

    CBU_DEBUG("decode oct %d: lsbn = %d, lsbyte = 0x%x\n", octnum, lsbn, dcod[lsbn]);

    for (i=0; i<cdlen-1; i++) {
        dout[dlen] = dcod[i]<<1 | ((dcod[lsbn]>>shift) & 0x01);
        dlen++;
        shift++;
        if (shift == 7) {
            i++; // skip lsbs byte
            shift = 0;
            octnum++;
            if (octnum == octets) {
                lsbn = cdlen - 1;
            } else {
                lsbn = (octnum+1)*8 - 1;
            }
            CBU_DEBUG("decode oct %d: lsbn = %d, lsbyte = 0x%x\n", octnum, lsbn, dcod[lsbn]);
        }
    }
#ifdef DEBUG_CBUNP
    CBU_DEBUG("decode len %2d: ", dlen);
    for(i=0; i<dlen; i++) {
        printf("0x%x ", dout[i]);
    }
    printf("\n");fflush(stdout);
#endif
    return dlen;
}

int ct_calc_code_len(int dlen)
{
	int cdlen = 0;

	if (dlen > 0) {
		cdlen = dlen + (dlen / 7);
		if (dlen % 7) cdlen += 1;
	}
	return cdlen;
}

int ct_code_data(_u8 *dcod_out, _u8 *data, _u8 dlen)
{
    _u8 byte;
    int cdlen = 0;
    int i, shift;

    byte = 0;
    cdlen = shift = 0;
    for (i=0; i < dlen; i++) {
        if (data[i] & 0x01)
            byte |= (0x01<<shift);
        dcod_out[cdlen] = data[i]>>1;
        cdlen++;
        shift++;
        if (shift == 7) {
            dcod_out[cdlen] = byte;
            cdlen++;
            byte = 0;
            shift = 0;
        }
    }
    if (shift != 0) {
        dcod_out[cdlen] = byte;
        cdlen++;
    }
    return cdlen;
}

void cbnp_notify(int chan, int canev)
{
    switch(canev) {
    case CIEV_RC:
		pthread_mutex_lock(&cdev[chan]->rcqlock);
        cbunp_rc_checkchange_state(chan);
		pthread_mutex_unlock(&cdev[chan]->rcqlock);
		if (cdev[chan]->sigenable & RX_SIG_FLAG) {
//            ReleaseSemaphore(chios[chan].cb_semas[CAN_SIG_RX], 1, NULL);
			int res = eventfd_write(chios[chan].cb_semas[CAN_SIG_RX].fd, 1);
//			printf("eventfd_write: SIG_RX = %d\n", res);
		}
        break;
    case CIEV_EWL:
		pthread_mutex_lock(&cdev[chan]->errlock);
        cdev[chan]->curerrs.ewl++;

		eventfd_write(chios[chan].select_evs[CAN_EVER].fd, 1);
//		printf("eventfd_write: EVER_EWL\n");

		pthread_mutex_unlock(&cdev[chan]->errlock);
		if (cdev[chan]->sigenable & ERR_SIG_FLAG) {
//            ReleaseSemaphore(chios[chan].cb_semas[CAN_SIG_EWL], 1, NULL);
			eventfd_write(chios[chan].cb_semas[CAN_SIG_EWL].fd, 1);
//			printf("eventfd_write: EWL\n");
		}
        break;
    case CIEV_BOFF:
        cbunp_stop(chan);
		pthread_mutex_lock(&cdev[chan]->errlock);
		cdev[chan]->curerrs.boff++;

		eventfd_write(chios[chan].select_evs[CAN_EVER].fd, 1);
//		printf("eventfd_write: EVER_BOFF\n");

		pthread_mutex_unlock(&cdev[chan]->errlock);
		if (cdev[chan]->sigenable & ERR_SIG_FLAG) {
//            ReleaseSemaphore(chios[chan].cb_semas[CAN_SIG_BOFF], 1, NULL);
			eventfd_write(chios[chan].cb_semas[CAN_SIG_BOFF].fd, 1);
//			printf("eventfd_write: SIG_BOFF\n");
		}
        break;
    case CIEV_HOVR:
		pthread_mutex_lock(&cdev[chan]->errlock);
        cdev[chan]->curerrs.hwovr++;
		cdev[chan]->hovr_cnt++;

		eventfd_write(chios[chan].select_evs[CAN_EVER].fd, 1);
//		printf("eventfd_write: EVER_HOVR\n");

		pthread_mutex_unlock(&cdev[chan]->errlock);
		if (cdev[chan]->sigenable & ERR_SIG_FLAG) {
//            ReleaseSemaphore(chios[chan].cb_semas[CAN_SIG_HOVR], 1, NULL);
			eventfd_write(chios[chan].cb_semas[CAN_SIG_HOVR].fd, 1);
//			printf("eventfd_write: SIG_HOVR\n");

		}
        break;
    case CIEV_WTOUT:
		pthread_mutex_lock(&cdev[chan]->errlock);
		cdev[chan]->curerrs.wtout++;

		eventfd_write(chios[chan].select_evs[CAN_EVER].fd, 1);
//		printf("eventfd_write: EVER_WTOUT\n");

		pthread_mutex_unlock(&cdev[chan]->errlock);
		if (cdev[chan]->sigenable & ERR_SIG_FLAG) {
//            ReleaseSemaphore(chios[chan].cb_semas[CAN_SIG_WTOUT], 1, NULL);
			eventfd_write(chios[chan].cb_semas[CAN_SIG_WTOUT].fd, 1);
//			printf("eventfd_write: WTOUT\n");

		}
        break;
    case CIEV_SOVR:
		pthread_mutex_lock(&cdev[chan]->errlock);
        cdev[chan]->curerrs.swovr++;
		cdev[chan]->sovr_cnt++;

		eventfd_write(chios[chan].select_evs[CAN_EVER].fd, 1);
//		printf("eventfd_write: EVER_SOVR\n");

		pthread_mutex_unlock(&cdev[chan]->errlock);
		if (cdev[chan]->sigenable & ERR_SIG_FLAG) {
//            ReleaseSemaphore(chios[chan].cb_semas[CAN_SIG_SOVR], 1, NULL);
			eventfd_write(chios[chan].cb_semas[CAN_SIG_SOVR].fd, 1);
//			printf("eventfd_write: SIG_SOVR\n");
		}
        break;
    default:
        break;
    }
}

void ct_analyse_frame(int chan, int eofind)
{
    int i, cmdind;
    _u8 len, cmd;
    int dlen, ret;
    canmsg_t f;
    _u8 data[64];

    if (eofind < CT_LEN_MIN) {
        //fprintf(fcbunp, "ch %d: bad frame(too small)\n", chan);
        return; //bad frame
    }

    len = cdev[chan]->rxbuf[eofind-1];
    if (len > eofind) { // wrong frame (len is greater then num of bytes in buf)
        //fprintf(fcbunp, "ch %d: wrong frame (len is greater then num of bytes in buf)\n", chan);
        //fflush(fcbunp);
        return;
    }
    cmdind = eofind-len;
    cmd = cdev[chan]->rxbuf[cmdind];
	CBU_DEBUG("\n\nSIGNAL: %d\n\n", cmd);
    switch (cmd) {
    case CIRX:
        if (len < 9) goto badlen;
        f.len = cdev[chan]->rxbuf[cmdind+1] & 0x0F;
        if (f.len > 8) goto badcan; // bad frame
        dlen = ct_decode_data(data, &cdev[chan]->rxbuf[cmdind+2], (int) len - 3);
        f.flags = 0;
        if (cdev[chan]->rxbuf[cmdind+1] & 0x10) { //EFF
            msg_seteff(&f);
            f.id = ((_u32) data[3]<<24) | ((_u32) data[2]<<16) | ((_u32) data[1]<<8) | (_u32) data[0];
            if (cdev[chan]->rxbuf[cmdind+1] & 0x20) { // RTR
                if (dlen != 8) goto badcan; // bad frame (data = 4id+4ts)
                msg_setrtr(&f);
                f.ts = ((_u32) data[7])<<24 | ((_u32) data[6])<<16 | ((_u32) data[5])<<8 | (_u32) data[4];
            } else { // data frame
                if (dlen != (8 + f.len)) goto badcan; // bad frame (data = 4id+4ts+f.len)
                for (i=0; i<f.len; i++) {
                    f.data[i] = data[4+i];
                }
                f.ts = ((_u32) data[7+f.len])<<24 | ((_u32) data[6+f.len])<<16 | ((_u32) data[5+f.len])<<8 | (_u32) data[4+f.len];
            }
        } else { // SFF
            f.id = ((_u32) data[1]<<8) | (_u32) data[0];
            if (cdev[chan]->rxbuf[cmdind+1] & 0x20) { // RTR
                if (dlen != 6) goto badcan; // bad frame (data = 2id+4ts)
                msg_setrtr(&f);
                f.ts = ((_u32) data[5]) << 24 | ((_u32) data[4]) << 16 | ((_u32) data[3]) << 8 | (_u32) data[2];
            } else { // data frame
                if (dlen != (6 + f.len)) goto badcan; // bad frame (data = 2id+4ts+f.len)
                for (i=0; i<f.len; i++) {
                    f.data[i] = data[2+i];
                }
                f.ts = ((_u32) data[5+f.len]) << 24 | ((_u32) data[4+f.len]) << 16 | ((_u32) data[3+f.len]) << 8 | (_u32) data[2+f.len];
            }
        }
		pthread_mutex_lock(&cdev[chan]->rcqlock);
        ret = fquestor(cdev[chan]->rcq, &f);
		pthread_mutex_unlock(&cdev[chan]->rcqlock);
        if (  ret < 0 ) { // SOVR sig
            cbnp_notify(chan, CIEV_SOVR);
        } else { //RX sig
            cbnp_notify(chan, CIEV_RC);
        }
        break;
    case CITXWAKEUP:
        if ( len != 2 ) goto badlen; // bad frame
		pthread_mutex_lock(&cdev[chan]->trlock);

		eventfd_write(chios[chan].select_evs[CAN_EVWR].fd, 1);
//		printf("eventfd_write: EVWR\n");

		pthread_mutex_unlock(&cdev[chan]->trlock);
        //printf("CITXWAKEUP\n");
        break;
    case CITXSLEEP:
        if ( len != 2 ) goto badlen; // bad frame
		pthread_mutex_lock(&cdev[chan]->trlock);
		eventfd_write(chios[chan].select_evs[CAN_EVWR].fd, 1);
//		printf("eventfd_write: EVWR\n");
		pthread_mutex_unlock(&cdev[chan]->trlock);
		//printf("CITXSLEEP\n");
		break;
	case CIERR:
		CBU_DEBUG("CI ERR!!\n");
		if ( len != 3 ) goto badlen; // bad frame
		cbnp_notify(chan, cdev[chan]->rxbuf[cmdind+1]);
		break;
	case GET_IDENTITY_STRING: case CISTATUS: case CITRSTAT: case CITRCANCEL:
	case CIREGREAD: case CITRANSMIT_SERIES:
		if (len < 6) {
			goto badlen;
		}
		pthread_mutex_lock(&cdev[chan]->cmdlock);
		if (cdev[chan]->cmd.code != cmd || cdev[chan]->cmd.state != CMD_WAIT ||
			cdev[chan]->cmd.seq != cdev[chan]->rxbuf[cmdind+1] )
		{
			pthread_mutex_unlock(&cdev[chan]->cmdlock);
			return;
		}
		cdev[chan]->cmd.res = cdev[chan]->rxbuf[cmdind+2];
		cdev[chan]->cmd.dlen =
			ct_decode_data(cdev[chan]->cmd.data, &cdev[chan]->rxbuf[cmdind+3], (int) len - 4);
		cdev[chan]->cmd.state = CMD_COMPLETE;
		pthread_mutex_unlock(&cdev[chan]->cmdlock);
		eventfd_write(cdev[chan]->cmdEvent.fd, 1);
//		printf("eventfd_write: cmdEvent\n");
		break;
    case CIOPEN: case CICLOSE: case CISTART: case CISTOP: case CISETFILTER:
    case CISETBAUD: case CIWRITE: case CITRANSMIT: case CIHWRESET: case CISETLOM:
    case CISETWTOUT: case CISETTRTRESHOLD:
		if (len != 4) goto badlen;
		pthread_mutex_lock(&cdev[chan]->cmdlock);
		if (cdev[chan]->cmd.code != cmd || cdev[chan]->cmd.state != CMD_WAIT ||
			cdev[chan]->cmd.seq != cdev[chan]->rxbuf[cmdind+1] )
		{
			pthread_mutex_unlock(&cdev[chan]->cmdlock);
			return;
		}
		cdev[chan]->cmd.res = cdev[chan]->rxbuf[cmdind+2];
		cdev[chan]->cmd.state = CMD_COMPLETE;
		pthread_mutex_unlock(&cdev[chan]->cmdlock);
		eventfd_write(cdev[chan]->cmdEvent.fd, 1);
//		printf("eventfd_write: cmdEvent\n");
		break;
	default:
        break;
    }
    return;

badlen:
    //    fprintf(fcbunp, "ch %d: badlen %d for cmd %d \n", chan, len, cmd);
    //    fflush(fcbunp);
    return;
badcan:
    //    fprintf(fcbunp, "ch %d: bad can frame rcved \n", chan);
    //    fflush(fcbunp);
    return;
}

void ct_shift2begin(_u8 *buf, int begind, int len)
{
    int i;
    for(i=0; i<len; i++) {
        buf[i] = buf[begind+i];
    }
}

void ct_analyse_rxbuf(int chan)
{
    int i, prev_eof_ind = -1;

    for (i=0; i<cdev[chan]->rxcnt; i++) {
        if (cdev[chan]->rxbuf[i] == CT_EOF) {
            prev_eof_ind = i;
            ct_analyse_frame(chan, i);
        }
    }

    if ( cdev[chan]->rxbuf[ cdev[chan]->rxcnt -1 ] != CT_EOF ) { // part of last frame recieved
        //fprintf(fcbunp, "ch %d: incomplete usb frame, rxcnt = %d\n", chan, cdev[chan]->rxcnt);
        if (cdev[chan]->rxcnt == RXBUF_SIZE && prev_eof_ind == -1) {
            //fprintf(fcbunp, "ch %d: buffer is full and no CT_EOF char (no frames in full buffer - erase it)\n", chan);
            // buffer is full and no CT_EOF char (no frames in full buffer - erase it)
            cdev[chan]->rxcnt = 0;
        } else if (prev_eof_ind != -1) {
            //fprintf(fcbunp, "ch %d: save part of last frame, erase all fully recieved ones\n", chan);
            // save part of last frame, erase all fully recieved ones, by moving tail to buf begin
            // printf("cbunp debug: move in buf\n");
            ct_shift2begin(cdev[chan]->rxbuf, prev_eof_ind+1, cdev[chan]->rxcnt-(prev_eof_ind+1));
            cdev[chan]->rxcnt -= (prev_eof_ind+1);
        } else {
            //fprintf(fcbunp, "ch %d: something is very very wrong\n", chan);
        }
    } else {
        cdev[chan]->rxcnt = 0; // erase rcv buffer with analysed frames
    }
}

void* ct_usbrcv_thread(void *arg)
{
	_u8 chan = *((_u8*)arg);
    DWORD RxBytes, ToRead, EventDWord, TxBytes;
    FT_STATUS ftStatus;


    while (1) {
        if (cdev[chan]->exit_flag) {
            return 0;
        }
        RxBytes = 0;
        FT_GetStatus(cdev[chan]->ftHandle, &RxBytes, &TxBytes, &EventDWord);
        if (RxBytes > 0) {
			CBU_DEBUG("GOT DATA %d bytes\n\n", RxBytes);
            if ( RxBytes < (RXBUF_SIZE - cdev[chan]->rxcnt) ) {
                ToRead = RxBytes;
            } else {
                ToRead = RXBUF_SIZE - cdev[chan]->rxcnt;
            }
            ftStatus = FT_Read(cdev[chan]->ftHandle, &cdev[chan]->rxbuf[ cdev[chan]->rxcnt ],
                ToRead, &RxBytes);
            if (ftStatus == FT_OK) {
                cdev[chan]->rxcnt += RxBytes;
                ct_analyse_rxbuf(chan);
            }
        } else {
            if (cdev[chan]->exit_flag) {
                return 0;
            }
			pthread_mutex_lock(&cdev[chan]->ftEvent.eMutex);
			pthread_cond_wait(&cdev[chan]->ftEvent.eCondVar, &cdev[chan]->ftEvent.eMutex);
			cdev[chan]->ftEvent.iVar--;
			pthread_mutex_unlock(&cdev[chan]->ftEvent.eMutex);
			CBU_DEBUG("\n\nTRIGGERED [ftEvent]:\n");
//            WaitForSingleObject(cdev[chan]->ftEvent, INFINITE);
        }
    }
    return 0;
}


/**********************************************************
*  Found devices functions
*/

_s16 cbunp_check_alive(char *serial)
{
    FT_STATUS ftStatus;
    FT_HANDLE ftHandle;
    DWORD BytesTx;
    DWORD BytesRx;
    int i;
    _u8 byte;
    _u8 frame_identity[4] = {GET_IDENTITY_STRING, CT_SEQ_STUB, 3, CT_EOF};

    CBU_DEBUG("cbunp_check_alive begin\n");

	ftStatus = FT_OpenEx( (PVOID) serial, FT_OPEN_BY_SERIAL_NUMBER, &ftHandle);
    if (ftStatus != FT_OK) {
        return -ECIGEN;
    }
    FT_ResetDevice(ftHandle);
    FT_SetLatencyTimer(ftHandle, 2);
    FT_Purge(ftHandle, FT_PURGE_RX | FT_PURGE_TX);
    byte = CT_CLOSE_APP;
    ftStatus = FT_Write(ftHandle, &byte, 1, &BytesTx);
    if (ftStatus != FT_OK) {
        return -ECIGEN;
    }
    ftStatus = FT_Write(ftHandle, &frame_identity, 4, &BytesTx);
    if (ftStatus != FT_OK) {
        return -ECIGEN;
    }
    for (i=0; i<50; i++) {
        BytesRx = 0;
        FT_GetQueueStatus(ftHandle,&BytesRx);
        if (BytesRx > 0) {
            //printf("byte from device is present\n");
            break;
        }
		usleep(1000);
    }
    // try to read one byte to check there is response from dev
    ftStatus = FT_Read(ftHandle, &byte, 1, &BytesRx);
    if (ftStatus != FT_OK) {
        return -ECIGEN;
    }
    if (BytesRx != 1)
        return -ECIGEN;
    byte = CT_CLOSE_APP;
    FT_Write(ftHandle, &byte, 1, &BytesTx);
    FT_Purge(ftHandle, FT_PURGE_RX | FT_PURGE_TX);
    FT_ResetDevice(ftHandle);
    FT_Close(ftHandle);

	CBU_DEBUG("cbunp_check_alive finish\n");

	return 0;
}

// return -1 error; 0 pid is not running; 1 pid is running
_s16 check_pid_running(DWORD pid)
{
	int ret = kill(pid, 0);
	if (ret == -1)
		return 0;
	else
		return 1;
}

void cbunp_check_and_free(_s16 chnum)
{
    int chnum2, brdnum;

	CBU_DEBUG("cbunp_check_and_free begin\n");

	if (cbunp_check_alive(chans[chnum].serial) < 0) { // dead device channel

		CBU_DEBUG("cbunp_check_and_free: dead device channel %d detected\n", chnum);

		brdnum = chans[chnum].brdnum;
        if (chboards[chnum].b.chip[0] == chnum)
            chnum2 = chboards[chnum].b.chip[1];
        else
            chnum2 = chboards[chnum].b.chip[0];
        ci_board_free(brdnum);
        ci_chan_free(chnum);
        ci_chan_free(chnum2);
    }

	CBU_DEBUG("cbunp_check_and_free fin\n");
}

// returns:
// 1 - dead chan [remove from devlist]
// 0 - chan is ok or busy (is opened by some procces) [remain in devlist]
// -1 - something wrong in system code [chan remain in devlist]
_s16 cbunp_check_chan_is_dead(_s16 chnum)
{
    int ret = 0, ret1;

    if (!chans[chnum].exist) return -1;
    if (chans[chnum].brdtype != CAN_BUS_USB_NP) return -1;
    if (chans[chnum].pid == 0) { //not opened channel
        if (cbunp_check_alive(chans[chnum].serial) < 0) { // dead device channel
            ret = 1;
        }
    } else {
        // check the process is running
        ret1 = check_pid_running(chans[chnum].pid);
        if (ret1 == 0) {
            //printf("PID %d for chan %d is not running\n", chans[chnum].pid , chnum);
            if (cbunp_check_alive(chans[chnum].serial) < 0) { // dead device channel
                ret = 1;
            }
        } else if (ret1 < 0) {
            //printf("check_pid_running() failed\n");
            ret = -1;
        } else if (ret1 > 0) {
            //printf("PID %d for chan %d is running\n", chans[chnum].pid , chnum);
            ret = 0;
        }
    }
    return ret;
}

void clear_dead_cbunp_devices(void)
{
    int i, ret0, ret1, chnum0, chnum1;

    for (i = 0; i < CI_BRD_NUMS; i++) {
        if (chboards[i].exist == 0) continue;
        if (chboards[i].type != CAN_BUS_USB_NP) continue;
        chnum0 = chboards[i].b.chip[0];
        chnum1 = chboards[i].b.chip[1];
        ret0 = cbunp_check_chan_is_dead(chnum0);
        ret1 = cbunp_check_chan_is_dead(chnum1);
        if (ret0 == 1 && ret1 == 1) { // both chans is dead
            //printf("dead cbunp dev\n");
            // remove board from devlist
            ci_board_free(i);
            ci_chan_free(chnum0);
            ci_chan_free(chnum1);
            chans[chnum0].pid = 0;
            chans[chnum1].pid = 0;
        } else {
            //printf("alive or busy cbunp dev\n");
        }
    }
}

// return 1 in list; 0 - not in list
_s16 cbunp_check_in_devlist(char *serial, DWORD LocId)
{
    int i, ret = 0;

    for (i = 0; i < CI_CHAN_NUMS; i++) {
        if (!chans[i].exist) continue;
        if (chans[i].brdtype != CAN_BUS_USB_NP) continue;
        if ( strcmp(chans[i].serial, serial) == 0 ) {
            return 1;
        }
    }
    return 0;
}


_s16 find_cbunp_devices(void)
{
    int i, k, found = 0, brdnum, chnum, chnum2;
    FT_STATUS ftStatus;
    DWORD numDevs, Flags, ID, Type, LocId, LocId2;
    DWORD vendor = 0x0403;
    DWORD product = 0xE8E9;
    FT_HANDLE ftHandleTemp;
	char serial[64] = { 0 }, serial2[64] = { 0 };
	char descr[64] = { 0 };
	ftStatus = FT_SetVIDPID(vendor, product);
	if (ftStatus != FT_OK)
		return -ECIGEN;

    clear_dead_cbunp_devices();
	//printf("CBUNP_FIND ENTER!!!\n");
	// create FTDI device information list
    numDevs = 0;
//    CBU_DEBUG("VIDPID: %#8x, %#8x\n", vendor, product);

    if ( (ftStatus = FT_CreateDeviceInfoList(&numDevs)) != FT_OK ) {
        return -ECIGEN;
    }
	CBU_DEBUG("found %d FTDI devices\n", numDevs);

    for (i = 0; i < (int) numDevs; i++) {
        ftStatus = FT_GetDeviceInfoDetail(i, &Flags, &Type, &ID, &LocId, serial, descr, &ftHandleTemp);
        if (ftStatus != FT_OK) {
            return -ECIGEN;
        }

        CBU_DEBUG("FTDI device %d: Type = %d, ID = %d, LocID = %d, serial = %s, descr = %s\n", i, Type, ID, LocId, serial, descr);

#ifdef DEBUG_CBUNP
        if ( strcmp(descr, cbunp_desc_1stchan) == 0 || strcmp(descr, cbunps_desc) == 0) {
            CBU_DEBUG("CBUNP dev found: Type = %d, ID = %d, LocID = %d, serial = %s, descr = %s\n", Type, ID, LocId, serial, descr);
            if (cbunp_check_alive(serial) < 0) {
                CBU_DEBUG("CHECK ALIVE FAILED!!!\n");
            }
        }
#endif
        if ( strcmp(descr, cbunp_desc_1stchan) == 0 ) { // CAN-bus-USBnp
            if (cbunp_check_in_devlist(serial, LocId)) continue;
            // find second channel
            for (k = 0; k < (int) numDevs; k++) {
                ftStatus = FT_GetDeviceInfoDetail(k, &Flags, &Type, &ID, &LocId2, serial2, descr, &ftHandleTemp);
                if (ftStatus != FT_OK) {
                    return -ECIGEN;
                }
                if ( strcmp(descr, cbunp_desc_2ndchan) == 0 && strncmp(serial, serial2, strlen(serial)-1) == 0) {
#ifdef DEBUG_CBUNP
                    CBU_DEBUG("CBUNP dev found: Type = %d, ID = %d, LocID = %d, serial = %s, descr = %s\n", Type, ID, LocId2, serial2, descr);
                    if (cbunp_check_alive(serial2) < 0) {
                        CBU_DEBUG("CHECK ALIVE FAILED!!!\n");
                    }
#endif
                    chnum = ci_chan_register_new(CAN_BUS_USB_NP);
                    chnum2 = ci_chan_register_new(CAN_BUS_USB_NP);
                    if (chnum >= 0 && chnum2 >= 0) {
                        brdnum = ci_board_register_new(CAN_BUS_USB_NP, "Marathon Ltd. Moscow", "CAN-bus-USBnp",
                            1, chnum, chnum2, -1, -1);
                        if (brdnum >= 0) {
                            ci_chan_assign_to_board(chnum, brdnum);
                            strcpy(chans[chnum].serial, serial);
                            //chans[chnum].locid = LocId;
                            ci_chan_assign_to_board(chnum2, brdnum);
                            strcpy(chans[chnum2].serial, serial2);
                            //chans[chnum2].locid = LocId2;
                            found += 2;
                        } else {
                            ci_chan_free(chnum);
                            ci_chan_free(chnum2);
                        }
                    } else {
                        ci_chan_free(chnum);
                        ci_chan_free(chnum2);
                    }
                }
            }
        }
        /*else if ( strcmp(descr, cbunps_desc) == 0 ) { // CAN-bus-USBnps
        chnum = ci_chan_register_new(CAN_BUS_USB_NPS);
        if (chnum >= 0) {
        brdnum = ci_board_register_new(CAN_BUS_USB_NPS, "Marathon Ltd. Moscow", "CAN-bus-USBnps",
        1, chnum, -1, -1, -1);
        if (brdnum >= 0) {
        ci_chan_assign_to_board(chnum, brdnum);
        strcpy(chans[chnum].serial, serial);
        chans[chnum].locid = LocId;
        found++;
        } else {
        ci_chan_free(chnum);
        }
        }
        }*/
    }
    return found;
}

/**********************************************************
*  CHAI functions
*/

static void _cbunp_exit(void)
{
    int i;
    for (i=0; i<CI_CHAN_NUMS; i++) {
        if ( cdev[i] ) {
            cbunp_close(i);
        }
    }
}

_s16 cbunp_init(void)
{
    int i;
    for (i=0; i<CI_CHAN_NUMS; i++) {
        cdev[i] = NULL;
    }
    atexit(_cbunp_exit);
    //fcbunp = fopen("c:\\cbunplog.txt", "a");
    return 0;
}

void cbunp_cmd_transact_alloc(_u8 chan, int code)
{
	CBU_DEBUG("Transact alloc: Trying to get cmdlock!\n");
	pthread_mutex_lock(&cdev[chan]->cmdlock);
	CBU_DEBUG("Transact alloc: Got cmdlock!\n");
    cdev[chan]->cmd.code = code;
    cdev[chan]->cmd.state = CMD_WAIT;
    cdev[chan]->cmd.seq++;
	if (cdev[chan]->cmd.seq > 127) {
        cdev[chan]->cmd.seq = 0;
	}
	CBU_DEBUG("Transact alloc: cmdlock unlocked!\n");
	pthread_mutex_unlock(&cdev[chan]->cmdlock);
}

void cbunp_cmd_transact_free(_u8 chan)
{
	CBU_DEBUG("Transact free: Trying to get cmdlock!\n");
	pthread_mutex_lock(&cdev[chan]->cmdlock);
	CBU_DEBUG("Transact free: Got cmdlock!\n");
    cdev[chan]->cmd.code = -1;
    cdev[chan]->cmd.state = CMD_NOEXIST;
	CBU_DEBUG("Transact free: cmdlock unlocked!\n");
	pthread_mutex_unlock(&cdev[chan]->cmdlock);
}

int cbunp_cmd_transact_wait_complete(_u8 chan)
{
    int ret = 0;
//    WaitForSingleObject(cdev[chan]->cmdEvent, CT_CMDRESP_TOUT);
	int rc = 0;

	rc = poll(&cdev[chan]->cmdEvent, 1, CT_CMDRESP_TOUT);
	eventfd_t ev = 0;
	if (cdev[chan]->cmdEvent.revents & POLLIN) {
//		printf("polled: cmdEvt\n");
		ret = eventfd_read(cdev[chan]->cmdEvent.fd, &ev);
		if (ret < 0)
			return -1;
	}
	if (rc == 0) {
		CBU_DEBUG("Waiting TIMEOUT!\n");
	}
	else if (rc < 0) {
		CBU_DEBUG("ERROR WAITING!\n");
	}
	else {
		CBU_DEBUG("Waiting successfull: %d\n", rc);
		CBU_DEBUG("cmdState: %d.\n", cdev[chan]->cmd.state);
	}
	pthread_mutex_lock(&cdev[chan]->cmdlock);
	if (cdev[chan]->cmd.state != CMD_COMPLETE) {
		ret = -ECITOUT;
	}
    // free transact
    cdev[chan]->cmd.code = -1;
    cdev[chan]->cmd.state = CMD_NOEXIST;
	pthread_mutex_unlock(&cdev[chan]->cmdlock);
    return ret;
}

int cbunp_cmd_execute(int chan, _u8 *frame, int flen)
{
    int ret = 0;

	pthread_mutex_lock(&cdev[chan]->tract_lock);
    cbunp_cmd_transact_alloc(chan, frame[0]);
    frame[1] = cdev[chan]->cmd.seq;
    CBU_DEBUG("sending frame...\n");
    if (ct_send_frame(chan, frame, flen) >= 0) {
	CBU_DEBUG("cmd transact wait complete on chan: %d\n", chan);
        ret = cbunp_cmd_transact_wait_complete(chan);
        if (ret >= 0) {
            if (cdev[chan]->cmd.res != 0) {
                ret = - (_s16) cdev[chan]->cmd.res;
            }
		}
		CBU_DEBUG("TRANSACT WAIT returned: %d\n", ret);
    } else { //error
        cbunp_cmd_transact_free(chan);
        ret = -ECIIO;
    }
	pthread_mutex_unlock(&cdev[chan]->tract_lock);
    return ret;
}

_s16 ct_close_app(_u8 chan)
{
    _u8 ct_close_app = CT_CLOSE_APP;
    if (ct_send_frame(chan, &ct_close_app, 1) < 0) {
        return -ECIIO;
    }
    return 0;
}

_s16 cbunp_get_identity(_u8 chan, char *idstr, int strlen)
{
    int ret =0;
    _u8 frame[4] = {GET_IDENTITY_STRING, CT_SEQ_STUB, 3, CT_EOF};
    int dlen = strlen - 1;

    if (cdev[chan] == NULL) return -ECIINVAL;

    ret = cbunp_cmd_execute(chan, frame, 4);
    if (ret < 0)
        return ret;
    if (cdev[chan]->cmd.dlen < dlen)
        dlen = cdev[chan]->cmd.dlen;
    memcpy(idstr, cdev[chan]->cmd.data, dlen);
    idstr[dlen] = '\0';

    return 0;
}

_u32 cbunp_get_fmw_ver(_u8 chan)
{
    char idstr[128];
    char *p;
    int c;
    _u32 ver = 0, vermin = 0, vermaj = 0;

    if (cbunp_get_identity(chan, idstr, 128) >= 0) {
        //printf ("ID: %s\n", idstr);
        p = idstr;
        while(*p++) {
            c = (int) *p;
            if (isdigit(c)) {
                sscanf(p, "%lu", &vermaj);
                //printf("vermaj = %lu\n", vermaj);
                break;
            }
        }
        while(*p++) {
            if (*p == '.') {
                sscanf(p+1, "%lu", &vermin);
                //printf("vermin = %lu\n", vermin);
                break;
            }
        }
        ver = CHAI_VER(0,vermaj,vermin);
    } else {
        CBU_DEBUG ("ID: getting idstring failed\n");
    }
    return ver;
}

_s16 _cbunp_setwritetout(_u8 chan, _u16 msec);
_s16 _cbunp_set_trqthreshold(_u8 chan, _u16 thres);

_s16 cbunp_open(_u8 chan, _u8 flags)
{
    FT_STATUS ftStatus;
    int ret =0;
    _u8 frame_open[5] = {CIOPEN, CT_SEQ_STUB, flags, 4, CT_EOF};


	CBU_DEBUG("cbunp_open begin\n");
    if (cdev[chan] != NULL) {
        return -ECIBUSY;
    }
	if ( (cdev[chan] = cbunpdev_alloc()) == NULL) {
        return -ECIMFAULT;
	}

	//CBU_DEBUG("cbunp_open 1\n");

	ftStatus = FT_OpenEx((PVOID)chans[chan].serial, FT_OPEN_BY_SERIAL_NUMBER, &cdev[chan]->ftHandle);
	//ftStatus = FT_OpenEx( (PVOID) chans[chan].locid, FT_OPEN_BY_LOCATION, &cdev[chan]->ftHandle);
    if (ftStatus != FT_OK) {
        cbunpdev_free(cdev[chan]);
        cdev[chan] = NULL;
        return -ECIGEN;
    }

	//CBU_DEBUG("cbunp_open 2\n");

    FT_ResetDevice(cdev[chan]->ftHandle);
    //FT_SetUSBParameters(cdev[chan]->ftHandle, 64, 0);
    FT_SetLatencyTimer(cdev[chan]->ftHandle, 2);
    FT_SetChars(cdev[chan]->ftHandle, CT_EOF, 1, CT_EOF, 0);
	FT_SetEventNotification(cdev[chan]->ftHandle, FT_EVENT_RXCHAR, (PVOID)&cdev[chan]->ftEvent);
    FT_Purge(cdev[chan]->ftHandle, FT_PURGE_RX | FT_PURGE_TX);
    ret = ct_close_app(chan);
    if (ret < 0) return ret;
	usleep(100000); // wait firmare application to close

	//CBU_DEBUG("cbunp_open 3\n");
	CBU_DEBUG("ci alloc semas evs\n");
	if ( (ret = ci_alloc_semas_evs(chan)) < 0) {
        goto err;
	}

	CBU_DEBUG("ci start cbthread\n");
    if ( (ret = ci_start_cbthread(chan)) < 0 ) {
        ci_free_semas_evs(chan);
        goto err;
    }

	CBU_DEBUG("cbunp creating thread\n");
//    cdev[chan]->ftThread = (HANDLE) _beginthreadex(NULL, 0, ct_usbrcv_thread, (void *) chan, 0, NULL);
	ret = pthread_create(&cdev[chan]->ftThread, NULL, &ct_usbrcv_thread, (void*)&chan);
	if (ret) {
        ret = -ECINORES;
        goto err_stopcb;
    }
	pthread_attr_t tAttr;
	int policy = 0;
	int maxPrioForPolicy = 0;

	pthread_attr_init(&tAttr);
	pthread_attr_getschedpolicy(&tAttr, &policy);
	maxPrioForPolicy = sched_get_priority_max(policy);
	pthread_setschedprio(cdev[chan]->ftThread, maxPrioForPolicy);
	pthread_attr_destroy(&tAttr);
//    SetThreadPriority(cdev[chan]->ftThread, THREAD_PRIORITY_HIGHEST);

    // todo: check identity string of can-bus-usbnp

	//CBU_DEBUG("cbunp_open 4\n");

    // open can-bus-usbnp
    ret = cbunp_cmd_execute(chan, frame_open, 5);
    if (ret < 0) {
        goto err_stopft;
    }
	//CBU_DEBUG("cbunp_open 5\n");
    // set default write timeout
    cdev[chan]->tout = CI_WRITE_TIMEOUT_DEF;
    ret = _cbunp_setwritetout(chan, cdev[chan]->tout);
    if (ret < 0) {
        cbunp_close(chan);
        return ret;
    }
	//CBU_DEBUG("cbunp_open 6\n");
	//CBU_DEBUG("cbunp_open 7\n");
    CBU_DEBUG("chan %d: cbunp opened\n", chan);
    cdev[chan]->fmver = cbunp_get_fmw_ver(chan);
    CBU_DEBUG("firmware version: %lu\n", cdev[chan]->fmver);
    // set default trq threshold
	if (cdev[chan]->fmver >= CHAI_VER(0,1,4) ) { // firmware vsersion >= 1.4
		cdev[chan]->trq_thres = CIQUE_TR_THRESHOLD_DEF;
		ret = _cbunp_set_trqthreshold(chan, cdev[chan]->trq_thres);
		if (ret < 0) {
			cbunp_close(chan);
			return ret;
		}
	}
	CBU_DEBUG("cbunp_open finish\n");
    return 0;

err_stopft:
	cdev[chan]->exit_flag = 1;
	pthread_mutex_lock(&cdev[chan]->ftEvent.eMutex);
	cdev[chan]->ftEvent.iVar++;
	pthread_cond_broadcast(&cdev[chan]->ftEvent.eCondVar);
	pthread_mutex_unlock(&cdev[chan]->ftEvent.eMutex);
	pthread_join(cdev[chan]->ftThread, NULL);
//    CloseHandle(cdev[chan]->ftThread);
err_stopcb:
    cdev[chan]->sigenable = 0;
    ci_stop_cbthread(chan);
    ci_free_semas_evs(chan);
err:
    FT_Close(cdev[chan]->ftHandle);
    cbunpdev_free(cdev[chan]);
    cdev[chan] = NULL;
    return ret;
}

_s16 cbunp_getfirmwarever(_u8 chan, _u32 *ver)
{
    *ver = (_u32) cdev[chan]->fmver;
    return 0;
}

_s16 cbunp_close(_u8 chan)
{
    int ret =0;
    _u8 frame_close[4] = {CICLOSE, CT_SEQ_STUB, 3, CT_EOF};

	// check not opened
    if (cdev[chan] == NULL) {
        return -ECIINVAL;
    }

    cbunp_stop(chan);
    //cbunp_cmd_execute(chan, frame_close, 4);
    ret = ct_close_app(chan);

	cdev[chan]->exit_flag = 1;
	pthread_mutex_lock(&cdev[chan]->ftEvent.eMutex);
	cdev[chan]->ftEvent.iVar++;
	pthread_cond_broadcast(&cdev[chan]->ftEvent.eCondVar);
	pthread_mutex_unlock(&cdev[chan]->ftEvent.eMutex);
	pthread_join(cdev[chan]->ftThread, NULL);
//    WaitForSingleObject(cdev[chan]->ftThread, INFINITE);
//    CloseHandle(cdev[chan]->ftThread);
    cdev[chan]->sigenable = 0;
    ci_stop_cbthread(chan);
    ci_free_semas_evs(chan);
    FT_ResetDevice(cdev[chan]->ftHandle);
    FT_Close(cdev[chan]->ftHandle);
    cbunpdev_free(cdev[chan]);
    cdev[chan] = NULL;

    /*fprintf(fcbunp, "chan %d: cbunp closed, cbunp_canrx = %d, cbunp_unknown=%d, cbunp_cansovr=%d\n",
    chan, cbunp_canrx[chan], cbunp_unknown[chan], cbunp_cansovr[chan]);
    fflush(fcbunp);*/
    return ret;
}

_s16 cbunp_check_chan_opened(_u8 chan)
{
    if (cdev[chan] == NULL) return -ECIINVAL;
    return 0;
}

_s16 cbunp_check_wait_condition(_u8 chan, _u8 wait_flags)
{
    if ( cdev[chan]->fmver < CHAI_VER(0,1,4) &&  (wait_flags & CI_WAIT_TR) )
        return -ECINOSYS;
    return 0;
}


_s16 cbunp_start(_u8 chan)
{
    int ret =0;
    _u8 frame[4] = {CISTART, CT_SEQ_STUB, 3, CT_EOF};

    ret = cbunp_cmd_execute(chan, frame, 4);
    if (ret < 0)
        return ret;
    cdev[chan]->chip_state = CAN_RUNNING;
    return 0;
}

_s16 cbunp_stop(_u8 chan)
{
    int ret =0;
    _u8 frame[4] = {CISTOP, CT_SEQ_STUB, 3, CT_EOF};

    ret = cbunp_cmd_execute(chan, frame, 4);
    if (ret < 0)
        return ret;
    cdev[chan]->chip_state = CAN_INIT;
    return 0;
}


_s16 cbunp_setfilter(_u8 chan, _u32 acode, _u32 amask)
{
    int ret = 0, cdlen = 0;
    _u8 frame[CT_LEN_MAX]; //real length is 13 bytes
    _u8 data[8];

    frame[0] = CISETFILTER;
    frame[1] = CT_SEQ_STUB;
    data[0] = (_u8) ( acode        & 0xFF);
    data[1] = (_u8) ((acode >>  8) & 0xFF);
    data[2] = (_u8) ((acode >> 16) & 0xFF);
    data[3] = (_u8) ((acode >> 24) & 0xFF);
    data[4] = (_u8) ( amask        & 0xFF);
    data[5] = (_u8) ((amask >>  8) & 0xFF);
    data[6] = (_u8) ((amask >> 16) & 0xFF);
    data[7] = (_u8) ((amask >> 24) & 0xFF);
    cdlen = ct_code_data(&frame[2], data, 8);
    if (cdlen != 10) {
        return -ECIGEN;
    }
    frame[12] = 13; // len
    frame[13] = CT_EOF;

    ret = cbunp_cmd_execute(chan, frame, 14);
    return ret;
}


_s16 cbunp_setbaud(_u8 chan, _u8 bt0, _u8 bt1)
{
    int ret = 0, cdlen = 0;
    _u8 frame[CT_LEN_MAX]; //real length is 7 bytes
    _u8 data[2];

    frame[0] = CISETBAUD;
    frame[1] = CT_SEQ_STUB;
    data[0] = bt0;
    data[1] = bt1;
    cdlen = ct_code_data(&frame[2], data, 2);
    if (cdlen != 3) {
        return -ECIGEN;
    }
    frame[2+cdlen] = 6; // len
    frame[6] = CT_EOF;

    ret = cbunp_cmd_execute(chan, frame, 7);
    return ret;
}

_s16 cbunp_read(_u8 chan, canmsg_t * mbuf, _s16 cnt)
{
    int rcnt;
	pthread_mutex_lock(&cdev[chan]->rcqlock);
	rcnt = fqueretr_many(cdev[chan]->rcq, mbuf, cnt);
	cbunp_rc_checkchange_state(chan);
	pthread_mutex_unlock(&cdev[chan]->rcqlock);
    return rcnt;
}

_s16 cbunp_rcquegetcnt(_u8 chan, _u16 *rcqcnt)
{
	pthread_mutex_lock(&cdev[chan]->rcqlock);
    *rcqcnt = cdev[chan]->rcq->count;
	pthread_mutex_unlock(&cdev[chan]->rcqlock);
    return 0;
}

_s16 cbunp_rcquecancel(_u8 chan, _u16 *rcqcnt)
{
	pthread_mutex_lock(&cdev[chan]->rcqlock);
    *rcqcnt = cdev[chan]->rcq->count;
    cdev[chan]->rcq->count = cdev[chan]->rcq->rloc = cdev[chan]->rcq->sloc = 0;
    cbunp_rc_checkchange_state(chan);
	pthread_mutex_unlock(&cdev[chan]->rcqlock);
    return 0;
}


_s16 cbunp_transmit(_u8 chan, _u8 mode, canmsg_t * mbuf)
{
    int ret = 0, cdlen = 0, dlen, i;
    _u8 frame[CT_LEN_MAX];
    _u8 data[32];

    if ( mode == CI_TR_QUEUE && cdev[chan]->fmver < CHAI_VER(0,1,4) )
        return -ECINOSYS;

    if ( mode == CI_TR_QUEUE ) {
        frame[0] = CITRANSMIT;
    } else {
        frame[0] = CIWRITE;
    }
    frame[1] = CT_SEQ_STUB;
    frame[2] = mbuf->len & 0x0F;
    dlen = 0;
    if (msg_iseff(mbuf)) {
        frame[2] |= 0x10;
        dlen += 4;
        data[0] = (_u8) (mbuf->id & 0xFF);
        data[1] = (_u8) ((mbuf->id >>  8) & 0xFF);
        data[2] = (_u8) ((mbuf->id >> 16) & 0xFF);
        data[3] = (_u8) ((mbuf->id >> 24) & 0xFF);
    } else {
        dlen += 2;
        data[0] = (_u8) (mbuf->id & 0xFF);
        data[1] = (_u8) ((mbuf->id >>  8) & 0xFF);
    }
    if (msg_isrtr(mbuf)) {
        frame[2] |= 0x20;
    } else {
        for (i=0; i<mbuf->len; i++)
            data[dlen+i] = mbuf->data[i];
        dlen += mbuf->len;
    }
    if (mode == CI_TR_QUEUE && mbuf->flags & FRAME_TRDELAY) { // dealy transmit frame
        frame[2] |= 0x40;
        data[dlen] = (_u8) (mbuf->ts & 0xFF);
        data[dlen + 1] = (_u8) ((mbuf->ts >>  8) & 0xFF);
        data[dlen + 2] = (_u8) ((mbuf->ts >> 16) & 0xFF);
        data[dlen + 3] = (_u8) ((mbuf->ts >> 24) & 0xFF);
        dlen += 4;
    }
    cdlen = ct_code_data(&frame[3], data, dlen);
    frame[3+cdlen] = 3+cdlen+1; // len
    frame[3+cdlen+1] = CT_EOF;

    ret = cbunp_cmd_execute(chan, frame, 3+cdlen+2);
    if (mode != CI_TR_QUEUE) { // CiWrite
        if (ret == 0) ret = 1;
    }
    return ret;
}


_s16 __cbunp_transmit_series_compat(_u8 chan, canmsg_t * mbuf, int cnt, int *err)
{
	int i, ret;

	*err = 0;
	for (i = 0; i < cnt; i++) {
		ret = cbunp_transmit(chan, CI_TR_QUEUE, &mbuf[i]);
		if (ret < 0) {
			*err = ret;
			//if (i == 0 && ret != -ECINORES)  // if (ret == -ECINORES) tr queue is full
			//	i = ret;
			break;
		}
	}
	return i;
}

_s16 cbunp_transmit_series(_u8 chan, canmsg_t * mbuf, int cnt, int *err)
{
	int ret = 0, cdlen = 0;
	_u16 tmp16 = 0;
	int dlen, dlen_save, i, k;
	_u8 frame[CT_LEN_MAX];
	_u8 data[CT_LEN_MAX];
	_u8 *flags;

	if (cdev[chan]->fmver < CHAI_VER(0, 1, 5)) {
		return __cbunp_transmit_series_compat(chan, mbuf, cnt, err);
	}
	*err = 0;
	frame[0] = CITRANSMIT_SERIES;
	frame[1] = CT_SEQ_STUB;

	dlen_save = dlen = 0;
	for (k = 0; k < cnt; k++) {
		dlen_save = dlen;    // save data length for case of overflow
		flags = &data[dlen];
		dlen += 1;
		*flags = mbuf[k].len & 0x0F;
		if (msg_iseff(&mbuf[k])) {
			*flags |= 0x10;
			data[dlen] = (_u8)(mbuf[k].id & 0xFF);
			data[dlen + 1] = (_u8)((mbuf[k].id >> 8) & 0xFF);
			data[dlen + 2] = (_u8)((mbuf[k].id >> 16) & 0xFF);
			data[dlen + 3] = (_u8)((mbuf[k].id >> 24) & 0xFF);
			dlen += 4;
		}
		else {
			data[dlen] = (_u8)(mbuf[k].id & 0xFF);
			data[dlen + 1] = (_u8)((mbuf[k].id >> 8) & 0xFF);
			dlen += 2;
		}
		if (msg_isrtr(&mbuf[k])) {
			*flags |= 0x20;
		}
		else {
			for (i = 0; i < mbuf[k].len; i++)
				data[dlen + i] = mbuf[k].data[i];
			dlen += mbuf[k].len;
		}
		if (mbuf[k].flags & FRAME_TRDELAY) { // dealy transmit frame
			*flags |= 0x40;
			data[dlen]     = (_u8)(mbuf[k].ts & 0xFF);
			data[dlen + 1] = (_u8)((mbuf[k].ts >> 8) & 0xFF);
			data[dlen + 2] = (_u8)((mbuf[k].ts >> 16) & 0xFF);
			data[dlen + 3] = (_u8)((mbuf[k].ts >> 24) & 0xFF);
			dlen += 4;
		}
		if (ct_calc_code_len(dlen) >= (CT_LEN_MAX - 4)) {// no more space for can frame
			dlen = dlen_save;
			break;
		}
		//printf("can code len = %d\n", ct_calc_code_len(dlen));
	}

/*	printf("can data content: ");
	for (i = 0; i < dlen; i++)
		printf("0x%x ", data[i]);
	printf("\n");
	*/
	cdlen = ct_code_data(&frame[2], data, dlen);
	//printf("common can code len = %d\n", cdlen);
	frame[2 + cdlen] = 2 + cdlen + 1; // len
	frame[2 + cdlen + 1] = CT_EOF;

	ret = cbunp_cmd_execute(chan, frame, 2 + cdlen + 2);
	if (ret < 0) {
		*err = ret;
		return 0;
	}
	if (cdev[chan]->cmd.dlen != 3) return -ECIGEN;
	tmp16 = (_u16)cdev[chan]->cmd.data[0] | (_u16)(((_u16)cdev[chan]->cmd.data[1] << 8) & 0xFF);
	*err = (_u16)cdev[chan]->cmd.data[3];
	*err = -(*err);
	return (int)tmp16;
}

_s16 cbunp_trstat(_u8 chan, _u16 *trqcnt)
{
    int ret = 0;
    _u32 tmp = 0;
    sja1000stat_t st;
    _u8 frame[4] = {CITRSTAT, CT_SEQ_STUB, 3, CT_EOF};

    if (cdev[chan]->fmver >= CHAI_VER(0,1,4) ) { // firmware vsersion >= 1.4
        ret = cbunp_cmd_execute(chan, frame, 4);
        if (ret < 0) return ret;
        if (cdev[chan]->cmd.dlen != 3) return -ECIGEN;
        ret = cdev[chan]->cmd.data[0];
        *trqcnt = (_u16) cdev[chan]->cmd.data[1] | (_u16) ( ((_u16) cdev[chan]->cmd.data[2] << 8) & 0xFF);
    } else { // firmware version < 1.4, emulate via chipstat (no transmit queue)
        *trqcnt = 0;
        ret = cbunp_chipstat(chan, (chipstat_t *) &st);
        if (ret < 0) return ret;
        if (st.stat & 0x04) { // TBS1 is 1 (transmit buffer is released)
            if (st.stat & 0x08) { // TCS1 is 1 (transmit complete)
                ret = CI_TR_COMPLETE_OK;
            } else {
                ret = CI_TR_COMPLETE_ABORT;
            }
        } else {
            ret = CI_TR_INCOMPLETE;
        }
    }
    return ret;
}

_s16 cbunp_trcancel(_u8 chan, _u16 *trqcnt)
{
    int ret =0;
    _u8 frame[4] = {CITRCANCEL, CT_SEQ_STUB, 3, CT_EOF};

    if (cdev[chan]->fmver < CHAI_VER(0,1,4) ) { // firmware vsersion < 1.4
        return -ECINOSYS;
    }

    ret = cbunp_cmd_execute(chan, frame, 4);
    if (ret < 0) return ret;
    if (cdev[chan]->cmd.dlen != 3) return -ECIGEN;
    ret = cdev[chan]->cmd.data[0];
    *trqcnt = (_u16) cdev[chan]->cmd.data[1] | (_u16) ( ((_u16) cdev[chan]->cmd.data[2] << 8) & 0xFF);
    return ret;
}

_s16 cbunp_chipstat(_u8 chan, chipstat_t * status)
{
    int ret =0;
    sja1000stat_t *st = (sja1000stat_t *) status;
    _u8 frame[4] = {CISTATUS, CT_SEQ_STUB, 3, CT_EOF};
    _u8 *data;


    ret = cbunp_cmd_execute(chan, frame, 4);
    if (ret < 0)
        return ret;
    if (cdev[chan]->cmd.dlen != 25)
        return -ECIGEN;

    data = cdev[chan]->cmd.data;

    st->brdnum = chans[chan].brdnum;
    st->irq = 0;
    st->baddr = 0;
    st->type = SJA1000;

    st->state = data[0];
    memcpy(&st->hovr_cnt, &data[1], 4);
    //st->hovr_cnt = cdev[chan]->hovr_cnt;
    st->sovr_cnt = cdev[chan]->sovr_cnt;
    st->mode = data[5];
    st->stat = data[6];
    st->inten = data[7];
    st->clkdiv = data[8];
    st->ecc = data[9];
    st->ewl = data[10];
    st->rxec = data[11];
    st->txec = data[12];
    st->rxmc = data[13];
    memcpy(&st->acode, &data[14], 4);
    memcpy(&st->amask, &data[18], 4);
    st->btr0 = data[22];
    st->btr1 = data[23];
    st->outctl = data[24];

    return 0;
}

_s16 cbunp_hwreset(_u8 chan)
{
    int ret =0;
    _u8 frame[4] = {CIHWRESET, CT_SEQ_STUB, 3, CT_EOF};

    ret = cbunp_cmd_execute(chan, frame, 4);
	if (ret<0) {
		return ret;
	}

	pthread_mutex_lock(&cdev[chan]->errlock);
    cdev[chan]->hovr_cnt = 0;
    cdev[chan]->sovr_cnt = 0;
	pthread_mutex_unlock(&cdev[chan]->errlock);
    return 0;
}

static _s16 _cbunp_regread(_u8 chan, _u32 offset, _u32 * val)
{
    int ret = 0, cdlen = 0;
    _u8 frame[CT_LEN_MAX];
    _u8 data[32];
    _u32 regval = 0;

    if (cdev[chan]->fmver < CHAI_VER(0,1,4) ) { // firmware vsersion < 1.4
        return -ECINOSYS;
    }

    frame[0] = CIREGREAD;
    frame[1] = CT_SEQ_STUB;
    data[0] = (_u8) ( offset        & 0xFFUL);
    data[1] = (_u8) ((offset >> 8)  & 0xFFUL);
    data[2] = (_u8) ((offset >> 16) & 0xFFUL);
    data[3] = (_u8) ((offset >> 24) & 0xFFUL);
    cdlen = ct_code_data(&frame[2], data, 4);
    if (cdlen != 5) {
        return -ECIGEN;
    }
    frame[2+cdlen] = 8; // len
    frame[8] = CT_EOF;

    ret = cbunp_cmd_execute(chan, frame, 9);
    if (ret < 0) {
        return ret;
    }
    if (cdev[chan]->cmd.dlen != 4)
        return -ECIGEN;

    regval  =  (_u32) cdev[chan]->cmd.data[3] << 24UL;
    regval |=  (_u32) cdev[chan]->cmd.data[2] << 16UL;
    regval |=  (_u32) cdev[chan]->cmd.data[1] << 8UL;
    regval |=  (_u32) cdev[chan]->cmd.data[0];

    *val = regval;
    return 0;
}

static _s16 _cbunp_regwrite(_u8 chan, _u32 offset, _u32 val)
{
    int ret = 0, cdlen = 0;
    _u8 frame[CT_LEN_MAX];
    _u8 data[32];
    _u32 regval = 0;

    if (cdev[chan]->fmver < CHAI_VER(0,1,4) ) { // firmware vsersion < 1.4
        return -ECINOSYS;
    }

    frame[0] = CIREGWRITE;
    frame[1] = CT_SEQ_STUB;
    data[0] = (_u8) ( offset        & 0xFFUL);
    data[1] = (_u8) ((offset >> 8)  & 0xFFUL);
    data[2] = (_u8) ((offset >> 16) & 0xFFUL);
    data[3] = (_u8) ((offset >> 24) & 0xFFUL);
    data[4] = (_u8) ( val        & 0xFFUL);
    data[5] = (_u8) ((val >> 8)  & 0xFFUL);
    data[6] = (_u8) ((val >> 16) & 0xFFUL);
    data[7] = (_u8) ((val >> 24) & 0xFFUL);
    cdlen = ct_code_data(&frame[2], data, 8);
    if (cdlen != 10) {
        return -ECIGEN;
    }
    frame[2+cdlen] = 13; // len
    frame[13] = CT_EOF;

    ret = cbunp_cmd_execute(chan, frame, 14);
    return ret;
}

_s16 cbunp_hwreg(_u8 chan, _u32 offset, _u32 * val, int getset)
{
	if (getset == CI_CMD_GET) {
		return _cbunp_regread(chan, offset, val);
	} else {
		return _cbunp_regwrite(chan, offset, *val);
	}
}

_s16 _cbunp_setwritetout(_u8 chan, _u16 msec)
{
    int ret = 0, cdlen = 0;
    _u8 frame[CT_LEN_MAX];  // real length is 7 bytes
    _u8 data[2];

    frame[0] = CISETWTOUT;
    frame[1] = CT_SEQ_STUB;
    data[0] = (_u8) ( msec       & 0xFF);
    data[1] = (_u8) ((msec >> 8) & 0xFF);
    cdlen = ct_code_data(&frame[2], data, 2);
    if (cdlen != 3) {
        return -ECIGEN;
    }
    frame[2+cdlen] = 6; // len
    frame[6] = CT_EOF;

    ret = cbunp_cmd_execute(chan, frame, 7);
    if (ret >= 0) {
        cdev[chan]->tout = msec;
    }
    return ret;
}

_s16 _cbunp_getwritetout(_u8 chan, _u16 * msec)
{
    *msec = cdev[chan]->tout;
    return 0;
}

_s16 cbunp_writetout(_u8 chan, _u16 * msec, int getset)
{
    if (getset == CI_CMD_GET) {
        return _cbunp_getwritetout(chan, msec);
    } else { // CI_CMD_SET
        return _cbunp_setwritetout(chan, *msec);
    }
}

_s16 cbunp_setlom(_u8 chan, _u8 mode)
{
    int ret = 0;
    _u8 frame[5] = {CISETLOM, CT_SEQ_STUB, mode, 4, CT_EOF};

    ret = cbunp_cmd_execute(chan, frame, 5);
    return ret;
}

_s16 cbunp_switch_sigs (_u8 chan, int sigflag, int onoff)
{
    if (onoff) {
        cdev[chan]->sigenable |= sigflag; //set
    } else {
        cdev[chan]->sigenable &= ~sigflag; //clear
    }
    return 0;
}

_s16 cbunp_errsgetclear(_u8 chan, canerrs_t * errs)
{
	pthread_mutex_lock(&cdev[chan]->errlock);
    memcpy(errs, &cdev[chan]->curerrs, sizeof(canerrs_t));
    memset(&cdev[chan]->curerrs, 0, sizeof(canerrs_t));
	//TODO!
//    ResetEvent(chios[chan].select_evs[CAN_EVER]);
	struct pollfd wait_object;
	u_int64_t event_value;
	int ret;
	if (chios[chan].select_evs[CAN_EVER].fd > 0) {
		wait_object.fd = chios[chan].select_evs[CAN_EVER].fd;
		wait_object.events = POLLIN;
		wait_object.revents = 0;
		ret = poll(&wait_object, 1, 0);
		if (ret < 0)
			return -1;
		if ((wait_object.revents & POLLIN) != 0) {
			ret = eventfd_read(chios[chan].select_evs[CAN_EVER].fd, &event_value);
			if (ret < 0)
				return -1;
		}
	}
	//printf("eventfd_reset: EVER\n");
	pthread_mutex_unlock(&cdev[chan]->errlock);
    return 0;
}

_s16 cbunp_rcqueresize(_u8 chan, _u16 size)
{
    int ret = 0;
    frameque_t  *new_rcq;

    if (cdev[chan]->chip_state != CAN_INIT)
        return -ECISTATE;

    new_rcq = fquealloc(size);
    if (new_rcq == NULL) {
        return -ECIMFAULT;
    }

	pthread_mutex_lock(&cdev[chan]->rcqlock);
    fquecncl(cdev[chan]->rcq);
    cdev[chan]->rcq = new_rcq;
    cbunp_rc_checkchange_state(chan);
	pthread_mutex_unlock(&cdev[chan]->rcqlock);
    return 0;
}

_s16 cbunp_rcqthreshold (_u8 chan, _u16 * thres, int getset)
{
    if (cdev[chan]->fmver < CHAI_VER(0,1,4) ) { // firmware vsersion < 1.4
        return -ECINOSYS;
    }
    if (getset == CI_CMD_GET) {
        *thres = cdev[chan]->rcq_thres;
    } else { // CI_CMD_SET
		pthread_mutex_lock(&cdev[chan]->rcqlock);
        cdev[chan]->rcq_thres = *thres;
        cbunp_rc_checkchange_state(chan);
		pthread_mutex_unlock(&cdev[chan]->rcqlock);
    }
    return 0;
}

_s16 _cbunp_set_trqthreshold(_u8 chan, _u16 thres)
{
    int ret = 0, cdlen = 0;
    _u8 frame[CT_LEN_MAX];  // real length is 7 bytes
    _u8 data[2];

    frame[0] = CISETTRTRESHOLD;
    frame[1] = CT_SEQ_STUB;
    data[0] = (_u8) ( thres       & 0xFF);
    data[1] = (_u8) ((thres >> 8) & 0xFF);
    cdlen = ct_code_data(&frame[2], data, 2);
    if (cdlen != 3) {
        return -ECIGEN;
    }
    frame[2+cdlen] = 6; // len
    frame[6] = CT_EOF;

    ret = cbunp_cmd_execute(chan, frame, 7);
    if (ret >= 0) {
        cdev[chan]->trq_thres = thres;
    }
    return ret;
}

_s16 cbunp_trqthreshold (_u8 chan, _u16 * thres, int getset)
{
    if (cdev[chan]->fmver < CHAI_VER(0,1,4) ) { // firmware vsersion < 1.4
        return -ECINOSYS;
    }

    if (getset == CI_CMD_GET) {
        *thres = cdev[chan]->trq_thres;
        return 0;
    } else { // CI_CMD_SET
        return _cbunp_set_trqthreshold(chan, *thres);
    }
}


_s16 cbunp_chan_operate(_u8 chan, int cmd)
{
    _s16 ret = 0;

    switch (cmd) {
    case CHANCMD_START:
        ret = cbunp_start(chan);
        break;
    case CHANCMD_STOP:
        ret = cbunp_stop(chan);
        break;
    case CHANCMD_HWRESET:
        ret = cbunp_hwreset(chan);
        break;
    case CHANCMD_LOM_ON:
        ret = cbunp_setlom(chan, CI_ON);
        break;
    case CHANCMD_LOM_OFF:
        ret = cbunp_setlom(chan, CI_OFF);
        break;
    default:
        ret = -EUCNOSYS;
        break;
    }
    return ret;
}

_s16 cbunp_que_operate(_u8 chan, int cmd, _u16 * p)
{
    _s16 ret = 0;

    switch (cmd) {
    case RCQUECMD_TRESH_GET:
        ret = cbunp_rcqthreshold (chan, p, CI_CMD_GET);
        break;
    case RCQUECMD_TRESH_SET:
        ret = cbunp_rcqthreshold (chan, p, CI_CMD_SET);
        break;
    case RCQUECMD_STAT:
        ret = cbunp_rcquegetcnt(chan, p);
        break;
    case RCQUECMD_CANCEL:
        ret = cbunp_rcquecancel(chan, p);
        break;
    case RCQUECMD_RESIZE:
        ret = cbunp_rcqueresize(chan, *p);
        break;
    case TRQUECMD_TRESH_GET:
        ret = cbunp_trqthreshold (chan, p, CI_CMD_GET);
        break;
    case TRQUECMD_TRESH_SET:
        ret = cbunp_trqthreshold (chan, p, CI_CMD_SET);
        break;
    case TRQUECMD_STAT:
        ret = cbunp_trstat(chan, p);
        break;
    case TRQUECMD_CANCEL:
        ret = cbunp_trcancel(chan, p);
        break;
    default:
        ret = -EUCNOSYS;
        break;
    }
    return ret;
}

struct chai_funcs cbunp_funcs = {
    /* open */ cbunp_open,
    /* close */ cbunp_close,
    /* check_chan_opened */ cbunp_check_chan_opened,
    /* check_wait_condition */ cbunp_check_wait_condition,
    /* switch_sigs */ cbunp_switch_sigs,
    /* read */ cbunp_read,
    /* transmit */ cbunp_transmit,
	/* transmit_series */ cbunp_transmit_series,
	/* chan_operate */ cbunp_chan_operate,
    /* que_operate */ cbunp_que_operate,
    /* setfilter */ cbunp_setfilter,
    /* setbaud */ cbunp_setbaud,
    /* chipstat */ cbunp_chipstat,
    /* errsgetclear */ cbunp_errsgetclear,
    /* writetout */ cbunp_writetout,
    /* hwreg */     cbunp_hwreg
};
