#ifndef CHAILNX_H
#define CHAILNX_H

#include <wchar.h>

#include "WinTypes.h"
#include <sys/signal.h>

enum {
	CAN_ERR_BOFF = 0,
	CAN_ERR_EWL,
	CAN_ERR_HWOVR,
	CAN_ERR_SWOVR,
	CAN_ERR_WTOUT
};

enum {
	CAN_ERRS_N = (CAN_ERR_WTOUT + 1)
};

#define CAN_ERRSIGS_BASE 2
enum can_signal_t {
	CAN_SIG_RX    = 0,
	CAN_SIG_TX    = 1,
	CAN_SIG_BOFF  = CAN_ERR_BOFF  + CAN_ERRSIGS_BASE,
	CAN_SIG_EWL   = CAN_ERR_EWL   + CAN_ERRSIGS_BASE,
	CAN_SIG_HOVR  = CAN_ERR_HWOVR + CAN_ERRSIGS_BASE,
	CAN_SIG_SOVR  = CAN_ERR_SWOVR + CAN_ERRSIGS_BASE,
	CAN_SIG_WTOUT = CAN_ERR_WTOUT + CAN_ERRSIGS_BASE
};


enum {
	CAN_SIGNALS_N = (CAN_SIG_WTOUT + 1)
};

enum {
	CAN_EVRD = 0,
	CAN_EVWR,
	CAN_EVER
};

enum {
	CAN_EVS_N = (CAN_EVER + 1)
};

struct chai_board {
	int exist;
	int type;
	_u32 fwver;   // firmware version for CAN-bus-USBnp devices
	_u32 vio;     // identifier for PCI/ISA boards (in reality it is a virtual base address of first chip)
	canboard_t b;
};

struct chai_chan {
	int exist;
	int brdtype;
	int brdnum;
	DWORD pid;
	wchar_t wdf_devpath[512]; // for WDF devices
	char serial[64]; // for CAN-bus-USBnp
	//_u32 locid; // for CAN-bus-USBnp
};

struct cev_handls {
	void (*cb[CICB_NUMS]) (_s16);
	void (*cb_ex[CICB_NUMS]) (_u8, _s16, void *);
	void *args[CICB_NUMS];
};

struct chai_funcs {
	_s16 (*open) (_u8 chan, _u8 flags);
	_s16 (*close) (_u8 chan);
	_s16 (*check_chan_opened) (_u8 chan);
	_s16 (*check_wait_condition) (_u8 chan,  _u8 wait_flags);
	_s16 (*switch_sigs) (_u8 chan, int sigflag, int onoff);
	_s16 (*read) (_u8 chan, canmsg_t * mbuf, _s16 cnt);
	_s16 (*transmit) (_u8 chan, _u8 mode, canmsg_t * mbuf);
	_s16 (*transmit_series) (_u8 chan, canmsg_t * mbuf, int cnt, int *err);
	_s16 (*chan_operate) (_u8 chan, int cmd);
	_s16 (*que_operate) (_u8 chan, int cmd, _u16 * p);
	_s16 (*setfilter) (_u8 chan, _u32 acode, _u32 amask);
	_s16 (*setbaud) (_u8 chan, _u8 bt0, _u8 bt1);
	_s16 (*chipstat) (_u8 chan, chipstat_t * stat);
	_s16 (*errsgetclear) (_u8 chan, canerrs_t * errs);
	_s16 (*writetout) (_u8 chan, _u16 * msec, int getset);
	_s16 (*hwreg) (_u8 chan, _u32 offset, _u32 * val, int getset);
};

typedef struct {
	int canfd;
	struct cev_handls ci_handlers;
	struct pollfd *cb_semas; //[CAN_SIGNALS_N]
	int semaSig;
	struct pollfd *select_evs; //[CAN_EVS_N];
	int    cbexit_flag;
	pthread_t cbthread;
	struct chai_funcs *cifuncs;
} chio_t;


/*
*  Transmit modes
*/
#define CI_TR_DIRECT    0x0
#define CI_TR_QUEUE     0x1

extern _s16 ci_get_free_brdslot(void);
extern _s16 ci_get_free_chanslot(void);
extern _s16 ci_board_register_new(_s16 type, char *manufact, char *name, _u32 hwver,
	_s16 chip0, _s16 chip1, _s16 chip2, _s16 chip3);
extern _s16 ci_board_free(_s16 brdnum);
extern _s16 ci_chan_register_new(int brdtype);
extern void ci_chan_assign_to_board(_s16 chslot, _s16 brdslot);
extern _s16 ci_chan_free(int chnum);
extern _s16 ci_alloc_semas_evs(_u8 chan);
extern void ci_free_semas_evs(_u8 chan);
extern _s16 ci_start_cbthread(_u8 chan);
extern void ci_stop_cbthread(_u8 chan);
extern void *_sighandl(void *arg);
extern struct chai_board chboards[];
extern struct chai_chan  chans[];
extern chio_t chios[];
extern pthread_mutex_t chai_cblock;

#endif // CHAILNX_H
