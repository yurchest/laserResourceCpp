/*
*  chai-lnx.c
*  CAN Hardware Abstraction Interface for Linux
*
*
*  Author: Fedor Nedeoglo, 1998-2015
*
*  Marathon Ltd. Moscow, 2013
*
*
*/

#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <sys/timeb.h>
#include <sys/eventfd.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <signal.h>
#include <string.h>
#include <poll.h>
#include <pthread.h>

#include "chai.h"
#include "unican.h"
#include "chai-lnx.h"
#include "chai-cbunp.h"

const _u32 chai_version = CHAI_VER(2, 11, 0);

//TODO:
//#pragma data_seg(".CBOARDSHARE")
int    chai_init                        = 0;
struct chai_board chboards[CI_BRD_NUMS] = { 0 };
struct chai_chan  chans[CI_CHAN_NUMS]   = { 0 };
//#pragma data_seg()

chio_t chios[CI_CHAN_NUMS] = {0};
pthread_mutex_t devlist_mutex;

#define DEVLIST_MUTEX_NAME TEXT("ChaiDevListMutex")
pthread_mutex_t chai_cblock = PTHREAD_MUTEX_INITIALIZER;

__attribute__((constructor)) void dllMain()
{
	pthread_mutexattr_t mAttr;
	pthread_mutexattr_init(&mAttr);
	pthread_mutexattr_settype(&mAttr, PTHREAD_MUTEX_RECURSIVE);
	pthread_mutex_init(&devlist_mutex, &mAttr);
}

__attribute__((destructor)) void dllFinish()
{
	pthread_mutex_lock(&devlist_mutex);
	//pid smth
	pthread_mutex_unlock(&devlist_mutex);
	pthread_mutex_destroy(&devlist_mutex);
}

void msg_zero(canmsg_t * msg)
{
    memset(msg, 0, sizeof(canmsg_t));
}

_s16 msg_isrtr(canmsg_t * msg)
{
    return (msg->flags & FRAME_RTR);
}

void msg_setrtr(canmsg_t * msg)
{
    msg->flags |= FRAME_RTR;
}

_s16 msg_iseff(canmsg_t * msg)
{
    return (msg->flags & FRAME_EFF);
}

void msg_seteff(canmsg_t * msg)
{
    msg->flags |= FRAME_EFF;
}

void msg_setdelaytr(canmsg_t * msg, _u32 mks)
{
    msg->flags = msg->flags | FRAME_TRDELAY;
    msg->ts = mks;
}


/*============================================
Internal data structures and functions
=============================================*/

const char *ci_errlist[] = {
    "success",
    "generic (not specified) error",
    "device or resourse busy",
    "memory fault",
    "function can't be called for chip in current state",
    "invalid call, function can't be called for this object",
    "invalid parameter",
    "can not access resource",
    "function or feature not implemented ",
    "input/output error",
    "no such device or object",
    "call was interrupted by event",
    "no resources",
    "time out occured"
};

struct canev_handls {
    void (*cb[CICB_NUMS]) (_s16);
    void (*cb_ex[CICB_NUMS]) (_u8, _s16, void *);
    void *args[CICB_NUMS];
};

_s32 canfds[CI_CHAN_NUMS];
struct canev_handls ci_handlers[CI_CHAN_NUMS];
_u16 wtouts[CI_CHAN_NUMS];
_u16 wtcounts[CI_CHAN_NUMS];

sigset_t chai_sigmask;

/*
 *  predefined signal numbers, from highest priority to low
 *  should be synced with signums from kernel part of sysdep.c
*/

//#define CAN_SIG_RX    SIGRTMAX-6
//#define CAN_SIG_TX    SIGRTMAX-5
//#define CAN_SIG_BOFF  SIGRTMAX-4
//#define CAN_SIG_EWL   SIGRTMAX-3
//#define CAN_SIG_HOVR  SIGRTMAX-2
//#define CAN_SIG_SOVR  SIGRTMAX-1
//#define CAN_SIG_WTOUT SIGRTMAX

static void _ln_sighandl(int sig, siginfo_t *info, void *extra)
{
	int chan;
	_s16 ev;
	int cbind;
	unsigned long tmp;

	tmp = (unsigned long) info->si_addr;
	chan = (int) tmp;
	chan &= 0xff;
	if (chan < 0 || chan >= CI_CHAN_NUMS)
		goto out;

	if (sig == CAN_SIG_RX) {
		ev = CIEV_RC;
		cbind = CICB_RX;
	} else if (sig == CAN_SIG_TX) {
		ev = CIEV_TR;
		cbind = CICB_TX;
	} else if (sig == CAN_SIG_EWL) {
		ev = CIEV_EWL;
		cbind = CICB_ERR;
	} else if (sig == CAN_SIG_BOFF) {
		ev = CIEV_BOFF;
		cbind = CICB_ERR;
	} else if (sig == CAN_SIG_HOVR) {
		ev = CIEV_HOVR;
		cbind = CICB_ERR;
	} else if (sig == CAN_SIG_SOVR) {
		ev = CIEV_SOVR;
		cbind = CICB_ERR;
	} else if (sig == CAN_SIG_WTOUT) {
		ev = CIEV_WTOUT;
		cbind = CICB_ERR;
	} else {
		goto out;
	}

	if (ci_handlers[chan].cb[cbind]) {
		ci_handlers[chan].cb[cbind] (ev);
	} else if (ci_handlers[chan].cb_ex[cbind]) {
		ci_handlers[chan].cb_ex[cbind] ((_u8) chan, (_s16) ev,
			ci_handlers[chan].args[cbind]);
	}
out:
	return;
}

void *_sighandl(void *arg)
{
	_u8 chan = *((_u8*)arg);
	free(arg);
	int sig;
	int cbind;

	if (chan >= CI_CHAN_NUMS || chan < 0)
		return -1;

	while (1) {
		if (chios[chan].cbexit_flag) {
			return 0;
		}
//		wmo = WaitForMultipleObjects(CAN_SIGNALS_N, chios[chan].cb_semas, FALSE, INFINITE);
//		printf("polling on channel: %d\n", chan);
		int rc = poll(chios[chan].cb_semas, CAN_SIGNALS_N, -1);
//		printf("polled\n");
		if (rc <= 0)
			return -1;

//		printf("Signal: %d\n", sig);
		if (chios[chan].cbexit_flag) {
			return 0;
		}

		eventfd_t ev = 0;
		for (int i = 0; i < CAN_SIGNALS_N; ++i)
			if (chios[chan].cb_semas[i].revents & POLLIN) {
				eventfd_read(chios[chan].cb_semas[i].fd, &ev);
//				printf("polled [sighandl]: %d\n", sig);
				sig = i;
				break;
			}

		pthread_mutex_lock(&chai_cblock);
		if (sig == CAN_SIG_RX) {
			ev = CIEV_RC;
			cbind = CICB_RX;
		} else if (sig == CAN_SIG_TX) {
			ev = CIEV_TR;
			cbind = CICB_TX;
		} else if (sig == CAN_SIG_EWL) {
			ev = CIEV_EWL;
			cbind = CICB_ERR;
		} else if (sig == CAN_SIG_BOFF) {
			ev = CIEV_BOFF;
			cbind = CICB_ERR;
		} else if (sig == CAN_SIG_HOVR) {
			ev = CIEV_HOVR;
			cbind = CICB_ERR;
		} else if (sig == CAN_SIG_SOVR) {
			ev = CIEV_SOVR;
			cbind = CICB_ERR;
		} else if (sig == CAN_SIG_WTOUT) {
			ev = CIEV_WTOUT;
			cbind = CICB_ERR;
		} else {
			pthread_mutex_unlock(&chai_cblock);
			continue;
		}

		if (chios[chan].ci_handlers.cb[cbind]) {
			//printf("cb debug begin ch %d\n", chan);
			chios[chan].ci_handlers.cb[cbind](ev);
			//printf("cb debug finish ch %d\n", chan);
		} else if (chios[chan].ci_handlers.cb_ex[cbind]) {
			//printf("cb_ex debug begin ch %d\n", chan);
			chios[chan].ci_handlers.cb_ex[cbind]((_u8) chan, (_s16) ev,  chios[chan].ci_handlers.args[cbind]);
			//printf("cb_ex debug finish ch %d\n", chan);
		}

		pthread_mutex_unlock(&chai_cblock);
	}
	return 0;
}

static _s16 _get_chai_errno(const _s16 unican_errno)
{
    _s16 err = (unican_errno < 0) ? -unican_errno : unican_errno;

    switch (err) {
    case 0:
        return 0;
        break;
    case EUCBUSY:
        return ECIBUSY;
        break;
    case EUCMFAULT:
        return ECIMFAULT;
        break;
    case EUCSTATE:
        return ECISTATE;
        break;
    case EUCINCALL:
        return ECIINCALL;
        break;
    case EUCINVAL:
        return ECIINVAL;
        break;
    case EUCACCES:
        return ECIACCES;
        break;
    case EUCNOSYS:
        return ECINOSYS;
        break;
    case EUCIO:
        return ECIIO;
        break;
    case EUCNODEV:
        return ECINODEV;
        break;
    case EUCINTR:
        return ECIINTR;
        break;
    case EUCRESTARTSYS:
        return ECIINTR;
        break;
    case EUCNORES:
        return ECINORES;
        break;
    case EUCTOUT:
        return ECITOUT;
        break;
    default:
        return ECIGEN;
        break;
    }
    return ECIGEN;
}

/*============================================
Basic functions
=============================================*/

#define _ci_cb_lock()     sigprocmask(SIG_BLOCK, &chai_sigmask, NULL);
#define _ci_cb_unlock()   sigprocmask(SIG_UNBLOCK, &chai_sigmask, NULL);

#define _ci_ioctl(d,cmd,parg,ioret) \
   if ( ( ioret = ioctl( d, cmd, (char *) (parg)) ) < 0) ioret = -_get_chai_errno(errno)

#define _ci_chan_operate(ch, ccmd, ioret) do {\
   _u32 cval = ccmd;\
   if ( ( ioret = ioctl( canfds[ch], CAN_IOC_CHAN_OPERATE, &cval) ) < 0) \
   ioret = -_get_chai_errno(errno); \
} while (0)

static int _ci_que_operate(_u8 chan, _u32 intype, _u32 qcmd, _u16 *param)
{
	int ret;
	canparam_t queop;

	CANOP_CMD_CONSTRUCT(&queop.p1,qcmd,intype);
	queop.p2 = *param;
	_ci_ioctl(canfds[chan], CAN_IOC_QUE_OPERATE, &queop, ret);
	*param = (_u16) queop.p2;
	return ret;
}

#define CHANOK_OR_RETURN(chan) \
	if (chan >= CI_CHAN_NUMS || canfds[chan] < 0) return -ECIINVAL

static void _ci_init_handlers(_u8 chan)
{
	int j;

	for (j = 0; j < CICB_NUMS; j++) {
		ci_handlers[chan].cb[j] = NULL;
		ci_handlers[chan].cb_ex[j] = NULL;
		ci_handlers[chan].args[j] = NULL;
	}
}

/*================================================================
Initialization
================================================================*/
_s16 ci_get_free_brdslot(void)
{
	_s16 slot = -1, i;

	for (i = 0; i < CI_BRD_NUMS; i++) {
		if (chboards[i].exist == 0) {
			slot = i;
			break;
		}
	}
	return slot;
}

_s16 ci_get_free_chanslot(void)
{
	_s16 slot = -1, i;

	for (i = 0; i < CI_CHAN_NUMS; i++) {
		if (chans[i].exist == 0) {
			slot = i;
			break;
		}
	}
	return slot;
}

_s16 ci_board_register(int brdslot, _s16 type, char *manufact, char *name, _u32 hwver,
	_s16 chip0, _s16 chip1, _s16 chip2, _s16 chip3)
{
	if (brdslot < 0)
		return -1;
	chboards[brdslot].exist = 1;
	chboards[brdslot].type = type;
	chboards[brdslot].vio = 0;
	chboards[brdslot].b.brdnum = (_u8) brdslot;
	chboards[brdslot].b.hwver = hwver;
	sprintf(chboards[brdslot].b.manufact, manufact);
	sprintf(chboards[brdslot].b.name, name);
	chboards[brdslot].b.chip[0] = chip0;
	chboards[brdslot].b.chip[1] = chip1;
	chboards[brdslot].b.chip[2] = chip2;
	chboards[brdslot].b.chip[3] = chip3;
	return 0;
}

_s16 ci_board_register_new(_s16 type, char *manufact, char *name, _u32 hwver,
	_s16 chip0, _s16 chip1, _s16 chip2, _s16 chip3)
{
	_s16 slot;

	slot = ci_get_free_brdslot();

	if (slot >= 0) {
		ci_board_register(slot, type, manufact, name, hwver,
			chip0, chip1, chip2, chip3);
	}
	return slot;
}

_s16 ci_board_free(_s16 brdnum)
{
	if (chboards[brdnum].exist == 0)
		return -1;
	chboards[brdnum].exist = 0;
	return 0;
}

_s16 ci_chan_register_new(int brdtype)
{
	_s16 slot = -1;

	slot = ci_get_free_chanslot();
	if (slot >= 0) {
		chans[slot].exist = 1;
		chans[slot].brdtype = brdtype;
		//chans[slot].locid = 0;
		chans[slot].serial[0] = '\0';
		chans[slot].wdf_devpath[0] = 0;
	}
	return slot;
}

void ci_chan_assign_to_board(_s16 chslot, _s16 brdslot)
{
	chans[chslot].brdnum = brdslot;
}

_s16 ci_chan_free(int chnum)
{
	if (chans[chnum].exist == 0)
		return -1;
	chans[chnum].exist = 0;
	return 0;
}

void ci_free_semas_evs(_u8 chan)
{
	int i;
	if (chios[chan].cb_semas)
		free(chios[chan].cb_semas);

	if (chios[chan].select_evs)
		free(chios[chan].select_evs);

//	for (i = 0; i < CAN_EVS_N; i++) {
//		if (chios[chan].select_evs[i])
//			CloseHandle(chios[chan].select_evs[i]);
//		chios[chan].select_evs[i] = NULL;
//	}
}

_s16 ci_alloc_semas_evs(_u8 chan)
{
	int i, ret = 0;
	chios[chan].cb_semas = malloc(sizeof(struct pollfd) * CAN_SIGNALS_N);
	for (i = 0; i < CAN_SIGNALS_N; i++) {
		chios[chan].cb_semas[i].fd = eventfd(0, EFD_SEMAPHORE);
		if (chios[chan].cb_semas[i].fd == 0) {
			ret = -ECINORES;
			goto err;
		}
		chios[chan].cb_semas[i].events = POLLIN;
		chios[chan].cb_semas[i].revents = 0;
	}
	chios[chan].select_evs = malloc(sizeof(struct pollfd) * CAN_EVS_N);
	for (i = 0; i < CAN_EVS_N; i++) {
		if (i == CAN_EVWR)
			chios[chan].select_evs[i].fd = eventfd(1, 0);
		else
			chios[chan].select_evs[i].fd = eventfd(0, 0);
		if (chios[chan].select_evs[i].fd == 0) {
			ret = -ECINORES;
			goto err;
		}
		chios[chan].select_evs[i].events = POLLIN;
		chios[chan].select_evs[i].revents = 0;
	}
	return ret;

err:
	ci_free_semas_evs(chan);
	return ret;
}

void ci_init_handlers(_u8 chan)
{
	int j;

	for(j=0; j<CICB_NUMS; j++) {
		chios[chan].ci_handlers.cb[j] = NULL;
		chios[chan].ci_handlers.cb_ex[j] = NULL;
		chios[chan].ci_handlers.args[j] = NULL;
	}
}

_s16 ci_start_cbthread(_u8 chan)
{
	int ret = 0;

	ci_init_handlers(chan);
	chios[chan].cbexit_flag = 0;
//	chios[chan].cbthread = (HANDLE) _beginthreadex(NULL, 0, _sighandl, (void *) chan, 0, NULL);
	pthread_attr_t attr;
	pthread_attr_init(&attr);
	//printf("creating thread\n");
//	printf("STARTING THREAD ON CHAN: %d\n", chan);
	int *arg = malloc(sizeof(*arg));
	*arg = chan;
	ret = pthread_create(&chios[chan].cbthread, NULL, _sighandl, arg);
	//printf("thread created\n");
	if (ret) {
		return -ECINORES;
	}
	return 0;
}

void ci_stop_cbthread(_u8 chan)
{
	pthread_mutex_lock(&chai_cblock);
	ci_init_handlers(chan);
	pthread_mutex_unlock(&chai_cblock);

	chios[chan].cbexit_flag = 1;
//	ReleaseSemaphore(chios[chan].cb_semas[CAN_SIG_WTOUT], 1, NULL);
	eventfd_write(chios[chan].cb_semas[CAN_SIG_WTOUT].fd, 1);
//	printf("eventfd_write: SIG_WTOUT\n");

//	WaitForSingleObject(chios[chan].cbthread, INFINITE);
//	CloseHandle(chios[chan].cbthread);
	pthread_join(chios[chan].cbthread, NULL);
}


void ci_print_boards_chans(void)
{
	int i,j;

	//printf("BOARDS:\n");
	for (i=0; i<CI_BRD_NUMS; i++) {
		if (chboards[i].exist) {
			//printf ("Board %d(%d) type %d %s: ", i, chboards[i].b.brdnum, chboards[i].type, chboards[i].b.name );
			for (j=0;j<4;j++) {
				if (chboards[i].b.chip[j] >= 0) {
					//printf("chan(%d) %d ", j, chboards[i].b.chip[j]);
				}
			}
			//printf("\n");
		}
	}
	//printf("CHANNELS:\n");
	for (i=0; i<CI_CHAN_NUMS; i++) {
		if (chans[i].exist) {
			//printf("chan %d: brdnum=%d brdtype=%d\n", i, chans[i].brdnum, chans[i].brdtype);
		}
	}
}

void ci_init_chai_funcs(void)
{
	int i;

	for (i = 0; i < CI_CHAN_NUMS; i++) {
		if (chans[i].exist) {
			switch (chans[i].brdtype) {
			case CAN_BUS_USB_NP: case CAN_BUS_USB_NPS:
				chios[i].cifuncs = &cbunp_funcs;
		//printf("FUNCS INITED");
				break;
			default:
				// chios[i].cifuncs = &wdf_funcs;
				break;
			}
		}
	}
}

#define CHANOK_OR_RETURN(chan) \
    if (chan >= CI_CHAN_NUMS || canfds[chan] < 0) return -ECIINVAL

_s16 CiInit(void)
{
	int i, j;

	for (i = 0; i < CI_CHAN_NUMS; i++) {
		chios[i].canfd = (int)INVALID_HANDLE_VALUE;
		ci_init_handlers(i);
		chios[i].cb_semas = NULL;
		chios[i].select_evs = NULL;
		chios[i].cbexit_flag = 0;
	}
	cbunp_init();

	pthread_mutex_lock(&devlist_mutex);
	if (chai_init == 0) {
		for (i = 0; i < CI_CHAN_NUMS; i++) {
			chans[i].brdtype = -1;
			chans[i].exist = 0;
			chans[i].pid = 0;
		}
		for (i = 0; i < CI_BRD_NUMS; i++) {
			chboards[i].exist = 0;
			chboards[i].vio = 0;
			chboards[i].fwver = 0;
			for (j=0; j<4; j++)
				chboards[i].b.chip[j] = -1;
		}
	//find_wdf_isa_pci_devices();
		chai_init = 1;
	}
	int cbunp_devices_count = find_cbunp_devices();
	if (cbunp_devices_count < 0) { // CAN-bus-USBnp should be called last always
		printf("find_cbunp_devices() failed\n");
	}
	ci_init_chai_funcs();

	if (cbunp_devices_count == 0) {
		struct sigaction act;

		for (i = 0; i < CI_CHAN_NUMS; i++) {
			canfds[i] = -1;
			ci_init_handlers(i);
		}
		act.sa_sigaction = _ln_sighandl;
		sigemptyset(&chai_sigmask);
		sigaddset(&chai_sigmask, (int)CAN_SIG_RX);
		sigaddset(&chai_sigmask, (int)CAN_SIG_EWL);
		sigaddset(&chai_sigmask, (int)CAN_SIG_BOFF);
		sigaddset(&chai_sigmask, (int)CAN_SIG_HOVR);
		sigaddset(&chai_sigmask, (int)CAN_SIG_SOVR);
		sigaddset(&chai_sigmask, (int)CAN_SIG_WTOUT);

		memcpy(&act.sa_mask, &chai_sigmask, sizeof(sigset_t));
		act.sa_flags = SA_SIGINFO | SA_RESTART;

		if (sigaction((int)CAN_SIG_RX, &act, NULL) == -1) {
			//perror("CHAI FATAL: sigaction");
			return -ECIGEN;
		}
		if (sigaction((int)CAN_SIG_EWL, &act, NULL) == -1) {
			//perror("CHAI FATAL: sigaction");
			return -ECIGEN;
		}
		if (sigaction((int)CAN_SIG_BOFF, &act, NULL) == -1) {
			//perror("CHAI FATAL: sigaction");
			return -ECIGEN;
		}
		if (sigaction((int)CAN_SIG_HOVR, &act, NULL) == -1) {
			//perror("CHAI FATAL: sigaction");
			return -ECIGEN;
		}
		if (sigaction((int)CAN_SIG_SOVR, &act, NULL) == -1) {
			//perror("CHAI FATAL: sigaction");
			return -ECIGEN;
		}
		if (sigaction((int)CAN_SIG_WTOUT, &act, NULL) == -1) {
			//perror("CHAI FATAL: sigaction");
			return -ECIGEN;
		}
	}
	pthread_mutex_unlock(&devlist_mutex);
	ci_print_boards_chans();
	return 0;
}

_s16 CiOpen(_u8 chan, _u8 flags)
{
	_s16 ret = 0;
	if (chans[chan].brdtype != CAN_BUS_USB_NP) {
		char fname[16];
		_u32 drvflag = 0;

		if (chan >= CI_CHAN_NUMS)
			return -ECIINVAL;
		if (canfds[chan] >= 0)
			return -ECIBUSY;

		snprintf(fname, 15, "/dev/can%d", chan);
		if (flags & CIO_BLOCK)
			canfds[chan] = open(fname, O_RDWR);
		else
			canfds[chan] = open(fname, O_RDWR | O_NONBLOCK);
		if (canfds[chan] < 0) {
			ret = -_get_chai_errno(errno);
			goto out;
		}

		if ((flags & CIO_CAN11) && (flags & CIO_CAN29)) {
			drvflag = CANMODE_29ON;
			if (ioctl(canfds[chan], CAN_IOC_SETMODE, &drvflag) < 0)
				goto err;
		} else if (flags & CIO_CAN29) {
			drvflag = CANMODE_29ON;
			if (ioctl(canfds[chan], CAN_IOC_SETMODE, &drvflag) < 0)
				goto err;
			drvflag = CANMODE_11OFF;
			if (ioctl(canfds[chan], CAN_IOC_SETMODE, &drvflag) < 0)
				goto err;
		}
		wtouts[chan] = CI_WRITE_TIMEOUT_DEF;
		goto out;

	err:
		ret = -_get_chai_errno(errno);
		close(canfds[chan]);
		canfds[chan] = -1;
	}
	else {
		if (chan >= CI_CHAN_NUMS)
			return -ECIINVAL;
		pthread_mutex_lock(&devlist_mutex);

		if (chans[chan].exist == 0) {
			ret = -ECINODEV;
			goto out;
		}

		ret = chios[chan].cifuncs->open(chan, flags);
		if (ret >= 0) {
			chans[chan].pid = getpid();
		}
		pthread_mutex_unlock(&devlist_mutex);
	}
out:
	return ret;
}

_s16 CiClose(_u8 chan)
{
	if (chans[chan].brdtype != CAN_BUS_USB_NP) {
		CHANOK_OR_RETURN(chan);

		close(canfds[chan]);
		canfds[chan] = -1;
		_ci_init_handlers(chan);
		return 0;
	}
	else {
		_s16 ret = 0;

		if (chan >= CI_CHAN_NUMS)
			return -ECIINVAL;

		pthread_mutex_lock(&devlist_mutex);
		if (chans[chan].exist == 0) {
			ret =  -ECINODEV;
			goto out;
		}
		ret = chios[chan].cifuncs->close(chan);
		if (ret >= 0) {
			chans[chan].pid = 0;
		}
	out:
		pthread_mutex_unlock(&devlist_mutex);
		return ret;
	}
}

_s16 CiStart(_u8 chan)
{
	CHANOK_OR_RETURN(chan);
	if (chans[chan].brdtype != CAN_BUS_USB_NP) {
		int ret;
		_ci_chan_operate(chan, CHANCMD_START, ret);
		return ret;
	}
	else
		return chios[chan].cifuncs->chan_operate(chan, CHANCMD_START);
}

_s16 CiStop(_u8 chan)
{
	CHANOK_OR_RETURN(chan);
	if (chans[chan].brdtype != CAN_BUS_USB_NP) {
		int ret;
		_ci_chan_operate(chan, CHANCMD_STOP, ret);
		return ret;
	}
	else
		return chios[chan].cifuncs->chan_operate(chan, CHANCMD_STOP);
}

_s16 CiSetFilter(_u8 chan, _u32 acode, _u32 amask)
{
	CHANOK_OR_RETURN(chan);
	if (chans[chan].brdtype != CAN_BUS_USB_NP) {
		int ret;
		canparam_t f;

		f.p1 = acode;
		f.p2 = amask;
		_ci_ioctl(canfds[chan], CAN_IOC_SET_FILTER, &f, ret);
		return ret;
	} else
		return chios[chan].cifuncs->setfilter(chan, acode, amask);
}

_s16 CiSetBaud(_u8 chan, _u8 bt0, _u8 bt1)
{
	CHANOK_OR_RETURN(chan);
	if (chans[chan].brdtype != CAN_BUS_USB_NP) {
		int ret;
		canparam_t cbaud;
		cbaud.p1 = bt0;
		cbaud.p2 = bt1;
		_ci_ioctl(canfds[chan], CAN_IOC_SET_BAUDRATE, &cbaud, ret);
		return ret;
	} else
		return chios[chan].cifuncs->setbaud(chan, bt0, bt1);
}

_s16 CiWrite(_u8 chan, canmsg_t * mbuf, _s16 cnt)
{
	CHANOK_OR_RETURN(chan);
	if (chans[chan].brdtype != CAN_BUS_USB_NP) {
		int ret = 0, ret1 = 0;
		struct pollfd pfd;
		_u16 trqcnt;

		_ci_ioctl(canfds[chan], CAN_IOC_WRITE, mbuf,ret);
		if (ret > 0 && wtouts[chan] > 0) {
			// wait
			pfd.fd = canfds[chan];
			pfd.events = 0;
			pfd.revents = 0;
			pfd.events = POLLOUT;
			_ci_cb_lock();
			ret1 = poll(&pfd, 1, (int) wtouts[chan]);
			if (ret1 < 0 || ret1 == 0) {    // poll error or timeout
				if (_ci_que_operate(chan, CANOP_POUT, TRQUECMD_CANCEL, &trqcnt) != CI_TR_COMPLETE_OK) {
					ioctl(canfds[chan], CAN_IOC_RISE_WTOUT, 0);
					ret = -ECIIO;
				} else {
					ret = 1;
				}
			}
			_ci_cb_unlock();
		}
		return ret;
	}
	else
		return chios[chan].cifuncs->transmit(chan, CI_TR_DIRECT, mbuf);
}

_s16 CiRead(_u8 chan, canmsg_t * mbuf, _s16 cnt)
{
	CHANOK_OR_RETURN(chan);
	if (chans[chan].brdtype != CAN_BUS_USB_NP) {
		int ret;
		canparam_t b;
		b.p1 = (unsigned long) mbuf;
		b.p2 = cnt;
		_ci_ioctl(canfds[chan], CAN_IOC_READ, &b, ret);
		return ret;
	}
	else
		return chios[chan].cifuncs->read(chan, mbuf, cnt);
}


#ifdef CHAI_STATUS
/*===========================================
Status and info functions
============================================*/

_u32 CiGetLibVer()
{
    return chai_version;
}

_u32 CiGetDrvVer()
{
    int ret;
    int fd;
    _u32 ver = 0;

    fd = open("/dev/unican", O_RDWR);
    if (fd < 0)
        return 0;
    ret = ioctl(fd, CAN_IOC_GETVER, &ver);
    close(fd);
    if (ret < 0)
        return 0;
    return ver;
}

_s16 CiChipStat(_u8 chan, chipstat_t * stat)
{
	CHANOK_OR_RETURN(chan);
	if (chans[chan].brdtype != CAN_BUS_USB_NP) {
		int ret;
		_ci_ioctl(canfds[chan], CAN_IOC_GET_STATUS, stat, ret);
		return ret;
	}
	else
		return chios[chan].cifuncs->chipstat(chan, stat);
}


_s16 CiBoardInfo(canboard_t * binfo)
{
	int ret = 0;
	if (chboards[binfo->brdnum].type != CAN_BUS_USB_NP) {
		int fd;

		if (binfo->brdnum >= CI_BRD_NUMS)
			return -ECIINVAL;
		fd = open("/dev/unican", O_RDWR);
		if (fd < 0) {
			return -_get_chai_errno(errno);
		}
		ret = ioctl(fd, CAN_IOC_GETBRDINFO, binfo);
		close(fd);
		if (ret < 0)
			return -_get_chai_errno(errno);

		return 0;
	}
	else {
		if (binfo->brdnum >= CI_BRD_NUMS)
			return -ECIINVAL;

		pthread_mutex_lock(&devlist_mutex);

		if (chboards[binfo->brdnum].exist == 0) {
			ret = -ECINODEV;
			goto out;
		}
		memcpy(binfo, &chboards[binfo->brdnum].b, sizeof(canboard_t));
out:
		pthread_mutex_unlock(&devlist_mutex);
		return ret;
	}
}

_s16 CiGetFirmwareVer(_u8 chan, _u32 * ver)
{
    *ver = 0;
    return 0;
}

_s16 CiBoardGetSerial(_u8 brdnum, char *sbuf, _u16 bufsize)
{
	if (chboards[brdnum].type != CAN_BUS_USB_NP) {
		if (brdnum >= CI_BRD_NUMS)
			return -ECIINVAL;
		return -ECINOSYS;
	}
	else {
		int len, ret = 0;

		if (brdnum >= CI_BRD_NUMS)
			return -ECIINVAL;

		pthread_mutex_lock(&devlist_mutex);
		if (chboards[brdnum].exist == 0) {
			ret = -ECINODEV;
			goto out;
		}

		switch (chboards[brdnum].type) {
		case CAN_BUS_USB_NP: case CAN_BUS_USB_NPS:
			len = (int) strlen(chans[chboards[brdnum].b.chip[0]].serial) - 1; //skip " A" or " B" in FTDI serial
			if (bufsize < (len +1)) len = bufsize-1;
			strncpy(sbuf, chans[chboards[brdnum].b.chip[0]].serial, len);
			sbuf[len] = '\0';
			break;
		default:
			ret = -ECINOSYS;
			break;
		}
out:
		pthread_mutex_unlock(&devlist_mutex);
		return ret;
	}
}

#endif

#ifdef CHAI_EXTRA
/*============================================
Extra functions
=============================================*/
_s16 CiTransmit(_u8 chan, canmsg_t * mbuf)
{
	CHANOK_OR_RETURN(chan);
	if (chans[chan].brdtype != CAN_BUS_USB_NP) {
		int ret;
		_ci_ioctl(canfds[chan], CAN_IOC_TRANSMIT, mbuf, ret);
		return ret;
	}
	else
		return chios[chan].cifuncs->transmit(chan, CI_TR_QUEUE, mbuf);
}

typedef struct {
	int evtype;
	int canwait_ind;
} wdfwait_t;


_s16 CiWaitEvent(canwait_t * cw, int cwcount, int tout)
{
	if (chans[cw[0].chan].brdtype != CAN_BUS_USB_NP) {
		struct pollfd pfd[CI_CHAN_NUMS];
		int ret, i;

		if (cwcount > CI_CHAN_NUMS)
			return -ECIINVAL;
		for (i = 0; i < cwcount; i++) {
			if (cw[i].chan >= CI_CHAN_NUMS || canfds[cw[i].chan] < 0)
				return -ECIINVAL;
			pfd[i].fd = canfds[cw[i].chan];
			pfd[i].events = 0;
			pfd[i].revents = 0;
			if (cw[i].wflags & CI_WAIT_RC)
				pfd[i].events |= POLLIN;
			if (cw[i].wflags & CI_WAIT_TR)
				pfd[i].events |= POLLOUT;
			if (cw[i].wflags & CI_WAIT_ER)
				pfd[i].events |= POLLPRI;
		}
		if ((ret = poll(pfd, cwcount, (int) tout)) < 0) {
			return -_get_chai_errno(errno);
		}
		if (ret <= 0)
			return ret;             //timeout or error
		ret = 0;
		for (i = 0; i < cwcount; i++) {
			cw[i].rflags = 0;
			if (pfd[i].revents & POLLIN) {
				cw[i].rflags |= CI_WAIT_RC;
				ret |= (0x1 << i);
			}
			if (pfd[i].revents & POLLOUT) {
				cw[i].rflags |= CI_WAIT_TR;
				ret |= (0x1 << i);
			}
			if (pfd[i].revents & POLLPRI) {
				cw[i].rflags |= CI_WAIT_ER;
				ret |= (0x1 << i);
			}
		}
		return ret;
	}
	else {
		int ret, i;
		int wtcnt;
		wdfwait_t wt[CI_CHAN_NUMS * CAN_EVS_N] = { 0 };
		struct pollfd whand[CI_CHAN_NUMS * CAN_EVS_N];
		int signalled;
		int sigind;
		int has_wflags = 0;

		if (cwcount > CI_CHAN_NUMS)
			return -ECIINVAL;
		wtcnt = 0;
		for (i = 0; i < cwcount; i++) {
			cw[i].rflags = 0;
			if (cw[i].chan >= CI_CHAN_NUMS)
				return -ECIINVAL;
			ret = chios[cw[i].chan].cifuncs->check_wait_condition(cw[i].chan, cw[i].wflags);
			if (ret < 0) return ret;
			has_wflags = 0;
			if (cw[i].wflags & CI_WAIT_RC) {
				wt[wtcnt].canwait_ind = i;
				wt[wtcnt].evtype = CAN_EVRD;
				whand[wtcnt] = chios[cw[i].chan].select_evs[wt[wtcnt].evtype];
				wtcnt++;
				has_wflags = 1;
			}
			if (cw[i].wflags & CI_WAIT_ER) {
				wt[wtcnt].canwait_ind = i;
				wt[wtcnt].evtype = CAN_EVER;
				whand[wtcnt] = chios[cw[i].chan].select_evs[wt[wtcnt].evtype];
				wtcnt++;
				has_wflags = 1;
			}
			if (cw[i].wflags & CI_WAIT_TR) {
				wt[wtcnt].canwait_ind = i;
				wt[wtcnt].evtype = CAN_EVWR;
				whand[wtcnt] = chios[cw[i].chan].select_evs[wt[wtcnt].evtype];
				wtcnt++;
				has_wflags = 1;
			}
			if (!has_wflags)
				return -ECIINVAL;
		}
		int rc = poll(whand, wtcnt, tout);
		if (rc == 0) {
			printf("CiWaitEvent: TIMEOUT\n");
			return 0;               //timeout
		}
		if (rc < 0) {
			printf("chai dbg: WaitForMultipleObjects failed\n");
			return -ECIGEN;
		}
		for (int i = 0; i < wtcnt; ++i)
			if (whand[i].revents & POLLIN) {
				signalled = i;
				break;
			}

		ret = 0;
		for (sigind = signalled; sigind < wtcnt; sigind++) {
			if (poll(&whand[sigind], 1, 0) <= 0)
				continue;
			eventfd_t ev = 0;
			if (!(whand[sigind].revents & POLLIN))
				continue;
//			printf("polled [CiWaitEvent]: %d\n", wt[sigind].evtype);
			eventfd_read(whand[sigind].fd, &ev);
			cw[wt[sigind].canwait_ind].rflags = 0;
			switch (wt[sigind].evtype) {
			case CAN_EVRD:
				cw[wt[sigind].canwait_ind].rflags |= CI_WAIT_RC;
				ret |= (0x1 << wt[sigind].canwait_ind);
				break;
			case CAN_EVER:
				cw[wt[sigind].canwait_ind].rflags |= CI_WAIT_ER;
				ret |= (0x1 << wt[sigind].canwait_ind);
				break;
			case CAN_EVWR:
				cw[wt[sigind].canwait_ind].rflags |= CI_WAIT_TR;
				ret |= (0x1 << wt[sigind].canwait_ind);
				break;
			default:
				break;
			}
		}
		return ret;
	}
}

_s16 CiErrsGetClear(_u8 chan, canerrs_t * errs)
{
	CHANOK_OR_RETURN(chan);
	if (chans[chan].brdtype != CAN_BUS_USB_NP) {
		int ret;
		_ci_ioctl(canfds[chan], CAN_IOC_GET_ERRS, errs, ret);
		return ret;
	}
	else
		return chios[chan].cifuncs->errsgetclear(chan, errs);
}

_s16 CiHwReset(_u8 chan)
{
	CHANOK_OR_RETURN(chan);
	if (chans[chan].brdtype != CAN_BUS_USB_NP) {
		int ret;
		_ci_chan_operate(chan, CHANCMD_HWRESET, ret);
		return ret;
	}
	else
		return chios[chan].cifuncs->chan_operate(chan, CHANCMD_HWRESET);
}

_s16 CiSetLom(_u8 chan, _u8 mode)
{
	CHANOK_OR_RETURN(chan);
	if (chans[chan].brdtype != CAN_BUS_USB_NP) {
		int ret;
		if (mode) {
			_ci_chan_operate(chan, CHANCMD_LOM_ON, ret);
		} else {
			_ci_chan_operate(chan, CHANCMD_LOM_OFF, ret);
		}
		return ret;
	}
	else {
		if (mode)
			return chios[chan].cifuncs->chan_operate(chan, CHANCMD_LOM_ON);
		else
			return chios[chan].cifuncs->chan_operate(chan, CHANCMD_LOM_OFF);
	}
}

_s16 CiWriteTout(_u8 chan, _s16 getset, _u16 * msec)
{
	CHANOK_OR_RETURN(chan);
	if (chans[chan].brdtype != CAN_BUS_USB_NP) {
		_ci_cb_lock();
		if (getset == CI_CMD_GET) {
			*msec = wtouts[chan];
		}
		else {                   // CI_CMD_SET
			wtouts[chan] = *msec;
		}
		_ci_cb_unlock();
		return 0;
	}
	else
		return chios[chan].cifuncs->writetout(chan, msec, getset);
}

void CiStrError(_s16 cierrno, char *buf, _s16 n)
{
	if (cierrno < 0)
		cierrno = -cierrno;
	if (cierrno > 13)
		cierrno = ECIGEN;

	strncpy(buf, ci_errlist[cierrno], n);
	buf[n - 1] = '\0';
	return;
}

void CiPerror(_s16 cierrno, const char *s)
{
    char buf[128];
    CiStrError(cierrno, buf, 127);
    fprintf(stderr, "%s: %s\n", s, buf);
    return;
}

_s16 CiRcQueResize(_u8 chan, _u16 size)
{
	CHANOK_OR_RETURN(chan);

	if (chans[chan].brdtype != CAN_BUS_USB_NP)
		return _ci_que_operate(chan, CANOP_PIN, RCQUECMD_RESIZE, &size);
	else
		return chios[chan].cifuncs->que_operate(chan, RCQUECMD_RESIZE, &size);
}

_s16 CiRcQueGetCnt(_u8 chan, _u16 * rcqcnt)
{
	CHANOK_OR_RETURN(chan);

	if (chans[chan].brdtype != CAN_BUS_USB_NP)
		return _ci_que_operate(chan, CANOP_POUT, RCQUECMD_STAT, rcqcnt);
	else
		return chios[chan].cifuncs->que_operate(chan, RCQUECMD_STAT, rcqcnt);
}

_s16 CiRcQueCancel(_u8 chan, _u16 * rcqcnt)
{
	CHANOK_OR_RETURN(chan);

	if (chans[chan].brdtype != CAN_BUS_USB_NP)
		return _ci_que_operate(chan, CANOP_POUT, RCQUECMD_CANCEL, rcqcnt);
	else
		return chios[chan].cifuncs->que_operate(chan, RCQUECMD_CANCEL, rcqcnt);
}

_s16 CiTrStat(_u8 chan, _u16 * trqcnt)
{
	CHANOK_OR_RETURN(chan);

	if (chans[chan].brdtype != CAN_BUS_USB_NP)
		return _ci_que_operate(chan, CANOP_POUT, TRQUECMD_STAT, trqcnt);
	else
		return chios[chan].cifuncs->que_operate(chan, TRQUECMD_STAT, trqcnt);
}


_s16 CiTrCancel(_u8 chan, _u16 * trqcnt)
{
	CHANOK_OR_RETURN(chan);

	if (chans[chan].brdtype != CAN_BUS_USB_NP)
		return _ci_que_operate(chan, CANOP_POUT, TRQUECMD_CANCEL, trqcnt);
	else
		return chios[chan].cifuncs->que_operate(chan, TRQUECMD_CANCEL, trqcnt);
}

_s16 CiTrQueThreshold(_u8 chan, _s16 getset, _u16 * thres)
{
	CHANOK_OR_RETURN(chan);
	if (chans[chan].brdtype != CAN_BUS_USB_NP) {
		if (getset == CI_CMD_GET) {
			return _ci_que_operate(chan, CANOP_POUT, TRQUECMD_TRESH_GET, thres);
		} else {
			return _ci_que_operate(chan, CANOP_PIN, TRQUECMD_TRESH_SET, thres);
		}
	}
	else {
		if (getset == CI_CMD_GET)
			return chios[chan].cifuncs->que_operate(chan, TRQUECMD_TRESH_GET, thres);
		else
			return chios[chan].cifuncs->que_operate(chan, TRQUECMD_TRESH_SET, thres);
	}
}

_s16 CiRcQueThreshold(_u8 chan, _s16 getset, _u16 * thres)
{
	CHANOK_OR_RETURN(chan);
	if (chans[chan].brdtype != CAN_BUS_USB_NP) {
		if (getset == CI_CMD_GET) {
			return _ci_que_operate(chan, CANOP_POUT, RCQUECMD_TRESH_GET, thres);
		} else {
			return _ci_que_operate(chan, CANOP_PIN, RCQUECMD_TRESH_SET, thres);
		}
	}
	else {
		if (getset == CI_CMD_GET)
			return chios[chan].cifuncs->que_operate(chan, RCQUECMD_TRESH_GET, thres);
		else
			return chios[chan].cifuncs->que_operate(chan, RCQUECMD_TRESH_SET, thres);
	}
}

#endif

#ifdef CHAI_BETA
/*==================================================================
Beta release (experimental) functions: could be changed or removed
====================================================================*/

_s16 CiRegRead(_u8 chan, _u32 offset, _u32 * val)
{
	CHANOK_OR_RETURN(chan);
	if (chans[chan].brdtype != CAN_BUS_USB_NP) {
		canparam_t reg;
		int ret;

		reg.p1 = offset;
		reg.p2 = 0;

		_ci_ioctl(canfds[chan], CAN_IOC_REG_READ, &reg, ret);
		*val = reg.p2;
		return ret;
	}
	else
		return chios[chan].cifuncs->hwreg(chan, offset, val, CI_CMD_GET);
}

_s16 CiRegWrite(_u8 chan, _u32 offset, _u32 val)
{
	CHANOK_OR_RETURN(chan);
	if (chans[chan].brdtype != CAN_BUS_USB_NP) {
		canparam_t reg;
		int ret;
		reg.p1 = offset;
		reg.p2 = val;
		_ci_ioctl(canfds[chan], CAN_IOC_REG_WRITE, &reg, ret);
		return ret;
	}
	else
		return chios[chan].cifuncs->hwreg(chan, offset, &val, CI_CMD_SET);
}

can_waitobj_t * CiSysWaitObjGet(_u8 chan)
{
	if (chans[chan].brdtype != CAN_BUS_USB_NP) {
		if (chan >= CI_CHAN_NUMS || canfds[chan] < 0)
			return NULL;
		return &canfds[chan];
	}
	else {
		//TODO!!
		if (chan >= CI_CHAN_NUMS)
			return NULL;
		pthread_mutex_lock(&devlist_mutex);
		if (chans[chan].exist == 0) {
			pthread_mutex_unlock(&devlist_mutex);
			return NULL;
		}
		pthread_mutex_unlock(&devlist_mutex);
		if ( chios[chan].cifuncs->check_chan_opened (chan) < 0)
			return NULL;

		return &chios[chan].select_evs;
	}
}

_s16 CiPnpDevListRescan(void)
{
	pthread_mutex_lock(&devlist_mutex);
	find_cbunp_devices();
	ci_init_chai_funcs();
	pthread_mutex_unlock(&devlist_mutex);
	return 0;
}

_s16 CiTransmitSeries(_u8 chan, canmsg_t * mbuf, int cnt, int *chaierr)
{
	CHANOK_OR_RETURN(chan);
	if (chans[chan].brdtype != CAN_BUS_USB_NP) {
		int ret, i;
		for(i=0; i<cnt; i++) {
			_ci_ioctl(canfds[chan], CAN_IOC_TRANSMIT, mbuf, ret);
			if (ret < 0) {
				if ( i==0 ) return  ret;
				else return i;
			}
		}
		return i;
	}
	else
		return chios[chan].cifuncs->transmit_series(chan, mbuf, cnt, chaierr);
}


#endif                          //CHAI_BETA

#ifdef CHAI_BACKCOMPAT
// backward compatibility functions, in new code use CiWriteTout
_s16 CiSetWriteTout(_u8 chan, _u16 msec)
{
    _u16 val = msec;

    return CiWriteTout(chan, CI_CMD_SET, &val);
}

_s16 CiGetWriteTout(_u8 chan, _u16 * msec)
{
    _s16 ret = 0;

    ret = CiWriteTout(chan, CI_CMD_GET, msec);
    return ret;
}

// backward compatibility, in new code use
// CiRcQueResize, CiRcQueCancel, CiRcQueGetCnt
_s16 CiQueResize(_u8 chan, _u16 size)
{
    return CiRcQueResize(chan, size);

}

_s16 CiRcQueEmpty(_u8 chan)
{
	_u16 rcqcnt;

	return CiRcQueCancel(chan, &rcqcnt);
}

_s32 CiRcGetCnt(_u8 chan)
{
    _u16 cnt = 0;
    _s32 ret = 0;

    ret = CiRcQueGetCnt(chan, &cnt);
    if (ret < 0)
        return (_s32) ret;
    return (_s32) cnt;
}


// backward compatibility functions, in new code use CiSetLom
_s16 CiSJA1000SetLom(_u8 chan)
{
    return CiSetLom(chan, 1);
}

_s16 CiSJA1000ClearLom(_u8 chan)
{
    return CiSetLom(chan, 0);
}

_s16 CiHwRead(_u8 chan, _s16 offset, _u8 * val)
{
    return -ECINOSYS;
}

_s16 CiHwWrite(_u8 chan, _s16 offset, _u8 * val)
{
    return -ECINOSYS;
}

// backward compatibility functions, in new code use CiWaitEvent

static _s16 _ci_set_cb(int ex_flag, _u8 chan, _u8 ev, void (*ci_cb) (_s16),
    void (*ci_cbex) (_u8, _s16, void *), void *arg)
{
	CHANOK_OR_RETURN(chan);
	if (chans[chan].brdtype != CAN_BUS_USB_NP) {
			int cbind;
			canparam_t cparam;

			switch (ev) {
			case CIEV_RC:
				cparam.p1 = ERR_SIG_FLAG;
				cbind = CICB_RX;
				break;
			case CIEV_CANERR:
				cparam.p1 = RX_SIG_FLAG;
				cbind = CICB_ERR;
				break;
			default:
				return -ECIINVAL;
				break;
			}
			cparam.p2 = CI_OFF;
			ioctl(canfds[chan], CAN_IOC_SW_SIGNAL, &cparam);     // clear
			if (ex_flag) {
				ci_handlers[chan].cb_ex[cbind] =
					(void (*)(_u8, _s16, void *)) ci_cbex;
				if (ci_cbex) {
					ci_handlers[chan].args[cbind] = arg;
					ci_handlers[chan].cb[cbind] = NULL; // switch off basic cb
				} else {
					ci_handlers[chan].args[cbind] = NULL;
				}
			} else {
				ci_handlers[chan].cb[cbind] = (void (*)(_s16)) ci_cb;
				if (ci_cb) {
					ci_handlers[chan].cb_ex[cbind] = NULL;      // switch off extended cb
					ci_handlers[chan].args[cbind] = NULL;
				}
			}
			if (ci_handlers[chan].cb_ex[cbind] || ci_handlers[chan].cb[cbind]) {
				cparam.p2 = CI_ON;
				ioctl(canfds[chan], CAN_IOC_SW_SIGNAL, &cparam); // set
			}
	}
	else {
		int sflag, cbind;

		switch (ev) {
		case CIEV_RC:
			sflag = RX_SIG_FLAG;
			cbind = CICB_RX;
			break;
		case CIEV_CANERR:
			sflag = ERR_SIG_FLAG;
			cbind = CICB_ERR;
			break;
		default:
			return -ECIINVAL;
			break;
		}

		pthread_mutex_lock(&chai_cblock);
		chios[chan].cifuncs->switch_sigs(chan, sflag, 0); // switch off
		if (ex_flag) {
			chios[chan].ci_handlers.cb_ex[cbind] = (void (*)(_u8, _s16, void*)) ci_cbex;
			if (ci_cbex) {
				chios[chan].ci_handlers.args[cbind] = arg;
				chios[chan].ci_handlers.cb[cbind] = NULL; // switch off basic cb
			} else {
				chios[chan].ci_handlers.args[cbind] = NULL;
			}
		} else {
			chios[chan].ci_handlers.cb[cbind] = (void (*)(_s16)) ci_cb;
			if (ci_cb) {
				chios[chan].ci_handlers.cb_ex[cbind] = NULL; // switch off extended cb
				chios[chan].ci_handlers.args[cbind] = NULL;
			}
		}
		if (chios[chan].ci_handlers.cb_ex[cbind] || chios[chan].ci_handlers.cb[cbind] ) {
			chios[chan].cifuncs->switch_sigs(chan, sflag, 1); // switch on
		}
		pthread_mutex_unlock(&chai_cblock);
		return 0;
	}
}

_s16 CiSetCB(_u8 chan, _u8 ev, void (*ci_cb) (_s16))
{
	return _ci_set_cb(0, chan, ev, ci_cb, NULL, NULL);
}

_s16 CiSetCBex(_u8 chan, _u8 ev, void (*ci_cb_ex) (_u8, _s16, void *),
    void *udata)
{
    return _ci_set_cb(1, chan, ev, NULL, ci_cb_ex, udata);
}

_s16 CiCB_lock(void)
{
	pthread_mutex_lock(&chai_cblock);
    return (_s16) _ci_cb_lock();
}

_s16 CiCB_unlock(void)
{
	pthread_mutex_unlock(&chai_cblock);
    return (_s16) _ci_cb_unlock();
}

// always returns false
// (hovr and sovr was removed from canmsg_t.flags)
_s16 msg_ishovr(canmsg_t * msg)
{
    return 0;                   // false
}

_s16 msg_issovr(canmsg_t * msg)
{
    return 0;                   // false
}

#endif //CHAI_BACKCOMPAT
