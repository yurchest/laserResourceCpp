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
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <signal.h>
#include <string.h>
#include <poll.h>

#include "chai.h"
#include "sysdep.h"
#include "unican.h"

const _u32 chai_version = CHAI_VER(2, 11, 0);

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

#define CAN_SIG_RX    SIGRTMAX-6
#define CAN_SIG_TX    SIGRTMAX-5
#define CAN_SIG_BOFF  SIGRTMAX-4
#define CAN_SIG_EWL   SIGRTMAX-3
#define CAN_SIG_HOVR  SIGRTMAX-2
#define CAN_SIG_SOVR  SIGRTMAX-1
#define CAN_SIG_WTOUT SIGRTMAX


static void _sighandl(int sig, siginfo_t * info, void *extra)
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

_s16 CiInit(void)
{
    int i;
    struct sigaction act;

    for (i = 0; i < CI_CHAN_NUMS; i++) {
        canfds[i] = -1;
        _ci_init_handlers(i);
    }
    act.sa_sigaction = _sighandl;
    sigemptyset(&chai_sigmask);
    sigaddset(&chai_sigmask, CAN_SIG_RX);
    sigaddset(&chai_sigmask, CAN_SIG_EWL);
    sigaddset(&chai_sigmask, CAN_SIG_BOFF);
    sigaddset(&chai_sigmask, CAN_SIG_HOVR);
    sigaddset(&chai_sigmask, CAN_SIG_SOVR);
    sigaddset(&chai_sigmask, CAN_SIG_WTOUT);

    memcpy(&act.sa_mask, &chai_sigmask, sizeof(sigset_t));
    act.sa_flags = SA_SIGINFO | SA_RESTART;

    if (sigaction(CAN_SIG_RX, &act, NULL) == -1) {
        perror("CHAI FATAL: sigaction");
        return -ECIGEN;
    }
    if (sigaction(CAN_SIG_EWL, &act, NULL) == -1) {
        perror("CHAI FATAL: sigaction");
        return -ECIGEN;
    }
    if (sigaction(CAN_SIG_BOFF, &act, NULL) == -1) {
        perror("CHAI FATAL: sigaction");
        return -ECIGEN;
    }
    if (sigaction(CAN_SIG_HOVR, &act, NULL) == -1) {
        perror("CHAI FATAL: sigaction");
        return -ECIGEN;
    }
    if (sigaction(CAN_SIG_SOVR, &act, NULL) == -1) {
        perror("CHAI FATAL: sigaction");
        return -ECIGEN;
    }
    if (sigaction(CAN_SIG_WTOUT, &act, NULL) == -1) {
        perror("CHAI FATAL: sigaction");
        return -ECIGEN;
    }
    return 0;
}

_s16 CiOpen(_u8 chan, _u8 flags)
{
    char fname[16];
    int ret = 0;
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
out:
    return ret;
}

_s16 CiClose(_u8 chan)
{
	CHANOK_OR_RETURN(chan);

    close(canfds[chan]);
    canfds[chan] = -1;
    _ci_init_handlers(chan);
    return 0;
}

_s16 CiStart(_u8 chan)
{
	int ret;
	
	CHANOK_OR_RETURN(chan);

    _ci_chan_operate(chan, CHANCMD_START, ret);
    return ret;
}

_s16 CiStop(_u8 chan)
{
	int ret;
	
	CHANOK_OR_RETURN(chan);

    _ci_chan_operate(chan, CHANCMD_STOP, ret);
    return ret;
}

_s16 CiSetFilter(_u8 chan, _u32 acode, _u32 amask)
{
	int ret;
    canparam_t f;

	CHANOK_OR_RETURN(chan);

    f.p1 = acode;
    f.p2 = amask;
    _ci_ioctl(canfds[chan], CAN_IOC_SET_FILTER, &f, ret);
    return ret;
}

_s16 CiSetBaud(_u8 chan, _u8 bt0, _u8 bt1)
{
	int ret;
    canparam_t cbaud;

	CHANOK_OR_RETURN(chan);

    cbaud.p1 = bt0;
    cbaud.p2 = bt1;
    _ci_ioctl(canfds[chan], CAN_IOC_SET_BAUDRATE, &cbaud, ret);
    return ret;
}

_s16 CiWrite(_u8 chan, canmsg_t * mbuf, _s16 cnt)
{
    int ret = 0, ret1 = 0;
    struct pollfd pfd;
    _u16 trqcnt;

	CHANOK_OR_RETURN(chan);

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

_s16 CiRead(_u8 chan, canmsg_t * mbuf, _s16 cnt)
{
	int ret;
    canparam_t b;

	CHANOK_OR_RETURN(chan);

    b.p1 = (unsigned long) mbuf;
    b.p2 = cnt;
    _ci_ioctl(canfds[chan], CAN_IOC_READ, &b, ret);
    return ret;
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
	int ret;
	
	CHANOK_OR_RETURN(chan);

    _ci_ioctl(canfds[chan], CAN_IOC_GET_STATUS, stat, ret);
    return ret;
}


_s16 CiBoardInfo(canboard_t * binfo)
{
    int ret = 0;
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

_s16 CiGetFirmwareVer(_u8 chan, _u32 * ver) 
{
    *ver = 0;
    return 0;
}

_s16 CiBoardGetSerial(_u8 brdnum, char *sbuf, _u16 bufsize)
{
    if (brdnum >= CI_BRD_NUMS)
        return -ECIINVAL;
    return -ECINOSYS;
}

#endif

#ifdef CHAI_EXTRA
/*============================================
Extra functions
=============================================*/
_s16 CiTransmit(_u8 chan, canmsg_t * mbuf) 
{
	int ret;
	
 	CHANOK_OR_RETURN(chan);

    _ci_ioctl(canfds[chan], CAN_IOC_TRANSMIT, mbuf, ret);
    return ret;
}


_s16 CiWaitEvent(canwait_t * cw, int cwcount, int tout)
{
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

_s16 CiErrsGetClear(_u8 chan, canerrs_t * errs)
{
    int ret;
    
	CHANOK_OR_RETURN(chan);

    _ci_ioctl(canfds[chan], CAN_IOC_GET_ERRS, errs, ret);
    return ret; 
}

_s16 CiHwReset(_u8 chan)
{
	int ret;
	
	CHANOK_OR_RETURN(chan);
	
    _ci_chan_operate(chan, CHANCMD_HWRESET, ret);

    return ret;
}

_s16 CiSetLom(_u8 chan, _u8 mode)
{
	int ret;

	CHANOK_OR_RETURN(chan);

    if (mode) {
        _ci_chan_operate(chan, CHANCMD_LOM_ON, ret);
    } else {
        _ci_chan_operate(chan, CHANCMD_LOM_OFF, ret);
	}
    return ret;
}

_s16 CiWriteTout(_u8 chan, _s16 getset, _u16 * msec) 
{
	CHANOK_OR_RETURN(chan);

    _ci_cb_lock();
    if (getset == CI_CMD_GET) {
        *msec = wtouts[chan];
    } else {                   // CI_CMD_SET
        wtouts[chan] = *msec;
    }
    _ci_cb_unlock();

    return 0;
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
	
	return _ci_que_operate(chan, CANOP_PIN, RCQUECMD_RESIZE, &size);
}

_s16 CiRcQueGetCnt(_u8 chan, _u16 * rcqcnt) 
{
	CHANOK_OR_RETURN(chan);
	    
	return _ci_que_operate(chan, CANOP_POUT, RCQUECMD_STAT, rcqcnt);
}

_s16 CiRcQueCancel(_u8 chan, _u16 * rcqcnt)
{
	CHANOK_OR_RETURN(chan);
    
	return _ci_que_operate(chan, CANOP_POUT, RCQUECMD_CANCEL, rcqcnt);
}

_s16 CiTrStat(_u8 chan, _u16 * trqcnt)
{
	CHANOK_OR_RETURN(chan);
    
	return _ci_que_operate(chan, CANOP_POUT, TRQUECMD_STAT, trqcnt);
}


_s16 CiTrCancel(_u8 chan, _u16 * trqcnt) 
{
	CHANOK_OR_RETURN(chan);

    return _ci_que_operate(chan, CANOP_POUT, TRQUECMD_CANCEL, trqcnt);
}

_s16 CiTrQueThreshold(_u8 chan, _s16 getset, _u16 * thres) 
{
	CHANOK_OR_RETURN(chan);
    
    if (getset == CI_CMD_GET) {
	    return _ci_que_operate(chan, CANOP_POUT, TRQUECMD_TRESH_GET, thres);
    } else {
	    return _ci_que_operate(chan, CANOP_PIN, TRQUECMD_TRESH_SET, thres);
	}
}

_s16 CiRcQueThreshold(_u8 chan, _s16 getset, _u16 * thres) 
{
	CHANOK_OR_RETURN(chan);
    
    if (getset == CI_CMD_GET) {
	    return _ci_que_operate(chan, CANOP_POUT, RCQUECMD_TRESH_GET, thres);
    } else {
	    return _ci_que_operate(chan, CANOP_PIN, RCQUECMD_TRESH_SET, thres);
	}
}

#endif

#ifdef CHAI_BETA
/*==================================================================
Beta release (experimental) functions: could be changed or removed
====================================================================*/ 

_s16 CiRegRead(_u8 chan, _u32 offset, _u32 * val)
{
    canparam_t reg;
    int ret;

	CHANOK_OR_RETURN(chan);

    reg.p1 = offset;
    reg.p2 = 0;

    _ci_ioctl(canfds[chan], CAN_IOC_REG_READ, &reg, ret);
    *val = reg.p2;
    return ret;
}

_s16 CiRegWrite(_u8 chan, _u32 offset, _u32 val)
{
    canparam_t reg;
    int ret;
    
	CHANOK_OR_RETURN(chan);

    reg.p1 = offset;
    reg.p2 = val;

    _ci_ioctl(canfds[chan], CAN_IOC_REG_WRITE, &reg, ret);
    return ret;
}

can_waitobj_t * CiSysWaitObjGet(_u8 chan)
{
    if (chan >= CI_CHAN_NUMS || canfds[chan] < 0) 
        return NULL; 

    return &canfds[chan];
}

_s16 CiPnpDevListRescan(void)
{
    return 0;
}

_s16 CiTransmitSeries(_u8 chan, canmsg_t * mbuf, int cnt)
{
	int ret, i;
	
 	CHANOK_OR_RETURN(chan);
    
    for(i=0; i<cnt; i++) {
        _ci_ioctl(canfds[chan], CAN_IOC_TRANSMIT, mbuf, ret);
        if (ret < 0) {
            if ( i==0 ) return  ret;
            else return i;
        }
    }
    return i;
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

_s16 _ci_set_cb(int ex_flag, _u8 chan, _u8 ev, void (*ci_cb) (_s16),
    void (*ci_cbex) (_u8, _s16, void *), void *arg)
{
    int cbind;
    canparam_t cparam;

	CHANOK_OR_RETURN(chan);

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
    return 0;
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
    return (_s16) _ci_cb_lock();
}

_s16 CiCB_unlock(void)
{
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
