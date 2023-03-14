/*
*  unicandrv.c
*  Uniform CAN (Unican)
* 
*  Author: Fedor Nedeoglo, 1998-2015
*
*  Marathon Ltd. Moscow, 2015
*/

#include "chai.h"
#include "sysdep.h"
#include "unican.h"

const _u32 unican_version = UNICAN_VER(11, 0, 0);

static canque_t *__quealloc(_u16 size) 
{
    canque_t * q;

    if ((q = sdep_malloc(sizeof(canque_t))) == NULL)
        return NULL;
    q->d = sdep_malloc(sizeof(canmsg_t) * size);
    if (q->d == NULL) {
        sdep_free(q);
        return NULL;
    }
    q->sloc = 0;
    q->rloc = 0;
    q->count = 0;
    q->size = size;
    return q;
}

static int __quecncl(canque_t * q) 
{
    if (!q)
        return -1;
    if (q->d)
        sdep_free(q->d);
    sdep_free(q);
    return 0;
}

static inline int __queempty(canque_t * q) 
{
    q->sloc = 0;
    q->rloc = 0;
    q->count = 0;
    return 0;
}

static inline int __questor(canque_t * q, canmsg_t * msg) 
{
    if (q->count == q->size) {
        return -1;             // queue is full
    }
    sdep_memcpy(q->d + q->sloc, msg, sizeof(canmsg_t));
    q->count++;
    q->sloc += 1;
    if (q->sloc >= q->size)
        q->sloc = 0;
    return 0;
}

static inline int __queretr(canque_t * q, canmsg_t * msg) 
{
    if (q->count == 0) {
        return -1;             // queue is empty 
    }
    sdep_memcpy(msg, q->d + q->rloc, sizeof(canmsg_t));
    q->count--;
    q->rloc += 1;
    if (q->rloc >= q->size)
        q->rloc = 0;
    return 0;
}

static int can_quealloc(struct can_dev * chip, _u16 size, _u8 type) 
{
    canque_t * q;

    if (sdep_atomic_get(&chip->state) != CAN_INIT)
        return -EUCSTATE;
    if (type == CIQUE_RC)
        q = chip->rc.q;
    else if (type == CIQUE_TR)
        q = chip->tr.q;
    else
        return -EUCINVAL;

    if (q != NULL)
        return -EUCBUSY;

    q = __quealloc(size);
    if (q == NULL)
        return -EUCMFAULT;

    if (type == CIQUE_RC)
        chip->rc.q = q;
    else
        chip->tr.q = q;
    return 0;
}

static int can_quecncl(struct can_dev * chip, _u8 type) 
{
    canque_t * q;

    if (type == CIQUE_RC) {
        q = chip->rc.q;
    } else if (type == CIQUE_TR) {
        q = chip->tr.q;
    } else {
        return -EUCINVAL;
    }
    if (q == NULL)
        return -EUCNODEV;
    __quecncl(q);
    q = NULL;
    return 0;
}

static int can_queempty(struct can_dev * chip, _u8 type) 
{
    canque_t * q;

    if (type == CIQUE_RC) {
        q = chip->rc.q;
    } else if (type == CIQUE_TR) {
        q = chip->tr.q;
    } else {
        return -EUCINVAL;
    }

    if (q == NULL)
        return -EUCNODEV;

    __queempty(q);
    return 0;
}


void __can_init_can_tr_struct(cantr_t * tr) 
{
    sdep_lock_init(&tr->lock);
    tr->trd_flag = TR_DELAY_FLAG_CLEAR;
    tr->q_thres = CIQUE_TR_THRESHOLD_DEF;
    tr->q = NULL;
}

void __can_init_can_rc_struct(canrc_t * rc) 
{
    sdep_lock_init(&rc->lock);
    rc->time_start = sdep_getts();
    rc->q_thres = CIQUE_RC_THRESHOLD_DEF;
    rc->q = NULL;
}

int can_open(struct can_dev *chip) 
{
    int ret = 0;

    chip->hwovr_cnt = 0;
    chip->swovr_cnt = 0;
    chip->signals = 0;

    chip->flags = SFF_ACCEPT_FLAG;     /* accept 11bit frames only */
    sdep_atomic_set(&chip->state, CAN_INIT);
    __can_init_can_tr_struct(&chip->tr);
    __can_init_can_rc_struct(&chip->rc);

    if ((ret = can_quealloc(chip, CIQUE_DEFSIZE_RC, CIQUE_RC)) < 0)
        return ret;

    if ((ret = can_quealloc(chip, CIQUE_DEFSIZE_TR, CIQUE_TR)) < 0)
        return ret;

    memset(chip->errs, 0, sizeof(_u16) * CAN_ERRS_N);
    sdep_lock_init(&chip->err_lock);
    chip->cops->init(chip);
    return 0;
}

void can_release(struct can_dev *chip) 
{
    SDEP_LOCK_DECLARE_FLAGS(flags);

    sdep_lock(&chip->tr.lock, &flags);
    if (chip->tr.trd_flag) {
        sdep_trdtimer_stop(chip);
        chip->tr.trd_flag = TR_DELAY_FLAG_CLEAR;
    }
    chip->tr.q->count = 0;
    sdep_unlock(&chip->tr.lock, &flags);
    chip->cops->release(chip);
    can_quecncl(chip, CIQUE_RC);
    can_quecncl(chip, CIQUE_TR);
    sdep_lock_free(&chip->err_lock);
    sdep_lock_free(&chip->rc.lock);
    sdep_lock_free(&chip->tr.lock);
}

int can_read(struct can_dev *chip, char *buf, int count) 
{
    canmsg_t tmp_msg;
    int ret;
    int readed = 0;
    canmsg_t * addr = (canmsg_t *) buf;
    SDEP_LOCK_DECLARE_FLAGS(flags);

    if (sdep_atomic_get(&chip->state) != CAN_RUNNING)
        return -EUCSTATE;

    if (count <= 0)
        return -EUCINVAL;

    if (chip->rc.q == NULL)
        return -EUCNODEV;

    while (readed < count) {
        sdep_lock(&chip->rc.lock, &flags);
        ret = __queretr(chip->rc.q, &tmp_msg);
        sdep_unlock(&chip->rc.lock, &flags);
        if (ret >= 0) {
            if (copy_to_user
                ((void *) &addr[readed], (void *) &tmp_msg,
                sizeof(canmsg_t))) 
            {
                readed = -EUCMFAULT;
                goto out;
            }
            readed++;
        } else {
            goto out;
        }
    }
out:
    return readed;
}


//
// transmit whithout tr queue
//
int can_write(struct can_dev * chip, canmsg_t * frame) 
{
    int ret = 0;
    SDEP_LOCK_DECLARE_FLAGS(flags);

    if (sdep_atomic_get(&chip->state) != CAN_RUNNING)
        return -EUCSTATE;

    if (frame->flags & FRAME_EFF) {
        if (!(chip->flags & EFF_ACCEPT_FLAG))
            return -EUCSTATE;
    } else {
        if (!(chip->flags & SFF_ACCEPT_FLAG))
            return -EUCSTATE;
    }

    // if transmit queue is not empty then CiTransmit is in use,
    // if chip->trd_flag is set, then delay transmition is in progress,
    // so CiWrite should not run
    sdep_lock(&chip->tr.lock, &flags);
	if (chip->tr.q_thres != chip->tr.q->size) {
		sdep_unlock(&chip->tr.lock, &flags);
        return -EUCINVAL;
	}
    if (chip->tr.q->count || chip->tr.trd_flag) {
		sdep_unlock(&chip->tr.lock, &flags);
        return -EUCBUSY;
    }
    ret = chip->cops->transmit(chip, frame);
    sdep_unlock(&chip->tr.lock, &flags);
    if (ret < 0) {
        return -EUCIO;
    }
    return 1;
}

void can_delay_transmition_handler(struct can_dev *chip)
{
    SDEP_LOCK_DECLARE_FLAGS(flags);

    sdep_lock(&chip->tr.lock, &flags);
    if (chip->tr.trd_flag) {
        chip->tr.trd_flag = TR_DELAY_FLAG_CLEAR;
        chip->cops->transmit(chip, &chip->tr.trd_frame);
    }
    sdep_unlock(&chip->tr.lock, &flags);
}

static inline void can_delay_transmit(struct can_dev *chip, canmsg_t * frame) 
{
    chip->tr.trd_frame = *frame;
    chip->tr.trd_flag = TR_DELAY_FLAG_SET;
    sdep_trdtimer_start(chip, frame->ts);
}

int can_transmit(struct can_dev *chip, canmsg_t * frame) 
{
    SDEP_LOCK_DECLARE_FLAGS(flags);
    int ret;

    if (sdep_atomic_get(&chip->state) != CAN_RUNNING)
        return -EUCSTATE;
    if (frame->flags & FRAME_EFF) {
        if (!(chip->flags & EFF_ACCEPT_FLAG))
            return -EUCSTATE;
    } else {
        if (!(chip->flags & SFF_ACCEPT_FLAG))
            return -EUCSTATE;
    }
    if (frame->flags & FRAME_TRDELAY && frame->ts == 0)
        frame->flags &= ~FRAME_TRDELAY;
    ret = 0;
    sdep_lock(&chip->tr.lock, &flags);
    if (chip->tr.q->count == 0 && !chip->tr.trd_flag
        && chip->cops->transmit_status(chip) != CI_TR_INCOMPLETE) {
            if (!(frame->flags & FRAME_TRDELAY)) {
                chip->cops->transmit(chip, frame);
            } else {               // delay transmit frame
                can_delay_transmit(chip, frame);
            }
    } else {
        if (__questor(chip->tr.q, frame) < 0) {
            ret = -EUCNORES;
        }
    }
    sdep_unlock(&chip->tr.lock, &flags);
    return ret;
}


int can_set_baud(struct can_dev * chip, _u32 bt0, _u32 bt1) 
{
    if (sdep_atomic_get(&chip->state) != CAN_INIT)
        return -EUCSTATE;

    return chip->cops->set_baud(chip, bt0, bt1);
}


int can_set_filter(struct can_dev * chip, _u32 acode, _u32 amask) 
{
    if (sdep_atomic_get(&chip->state) != CAN_INIT)
        return -EUCSTATE;
    return chip->cops->set_filter(chip, acode, amask);
}

int can_get_status(struct can_dev * chip, chipstat_t * st) 
{
    return chip->cops->get_status(chip, st);
}

int can_get_clear_errs(struct can_dev * chip, canerrs_t * errs) 
{
    SDEP_LOCK_DECLARE_FLAGS(flags);

    sdep_lock(&chip->err_lock, &flags);
    errs->ewl   = chip->errs[CAN_ERR_EWL];
    errs->boff  = chip->errs[CAN_ERR_BOFF];
    errs->hwovr = chip->errs[CAN_ERR_HWOVR];
    errs->swovr = chip->errs[CAN_ERR_SWOVR];
    errs->wtout = chip->errs[CAN_ERR_WTOUT];
    memset(chip->errs, 0, sizeof(_u16) * CAN_ERRS_N);
    sdep_unlock(&chip->err_lock, &flags);
    return 0;
}

int can_switch_sigs(struct can_dev * chip, int sigflag, int onoff) 
{
    if (sdep_atomic_get(&chip->state) != CAN_INIT)
        return -EUCSTATE;
    if (sigflag != ERR_SIG_FLAG && sigflag != RX_SIG_FLAG)
        return -EUCINVAL;

    if (onoff) {
        chip->signals |= ((_u8) sigflag);
    } else {
        chip->signals &= ~((_u8) sigflag);
    }
    return 0;
}

int can_set_mode(struct can_dev * chip, int mode) 
{
    if (sdep_atomic_get(&chip->state) != CAN_INIT)
        return -EUCSTATE;
    switch (mode) {
    case CANMODE_11ON:
        chip->flags |= SFF_ACCEPT_FLAG;
        break;

    case CANMODE_11OFF:
        chip->flags &= ~SFF_ACCEPT_FLAG;
        break;

    case CANMODE_29ON:
        chip->flags |= EFF_ACCEPT_FLAG;
        break;

    case CANMODE_29OFF:
        chip->flags &= ~EFF_ACCEPT_FLAG;
        break;

    default:
        return -EUCINVAL;
        break;
    }
    return 0;
}

static void can_stop(struct can_dev *chip) 
{
    SDEP_LOCK_DECLARE_FLAGS(flags);

    sdep_lock(&chip->tr.lock, &flags);
    if (chip->tr.trd_flag) {
        sdep_trdtimer_stop(chip);
    }
    sdep_unlock(&chip->tr.lock, &flags);
    chip->cops->stop(chip);
    sdep_atomic_set(&chip->state, CAN_INIT);
}

static void can_start(struct can_dev *chip) 
{
    SDEP_LOCK_DECLARE_FLAGS(flags);
    canmsg_t frame;

    sdep_lock(&chip->tr.lock, &flags);
    chip->cops->start(chip);
    if (chip->tr.trd_flag) {
        sdep_trdtimer_start(chip, chip->tr.trd_frame.ts);
    } else if (chip->cops->transmit_status(chip) != CI_TR_INCOMPLETE) {
        if (chip->tr.q->count > 0) {
            __queretr(chip->tr.q, &frame);
            chip->cops->transmit(chip, &frame);
        }
    }
    sdep_atomic_set(&chip->state, CAN_RUNNING);
    sdep_unlock(&chip->tr.lock, &flags);
}

static void can_hwreset(struct can_dev *chip) 
{
    chip->cops->hwreset(chip);
    chip->swovr_cnt = 0;
    chip->hwovr_cnt = 0;
}

static int can_set_lom(struct can_dev * chip, int onoff) 
{
    if (chip->type != SJA1000)
        return -EUCNOSYS;
    if (sdep_atomic_get(&chip->state) != CAN_INIT)
        return -EUCSTATE;
    if (onoff) {
        chip->cops->set_lom(chip, 1);
    } else {
        chip->cops->set_lom(chip, 0);
    }
    return 0;
}

int can_chan_operate(struct can_dev * chip, int cmd)
{
    int ret = 0;

    switch (cmd) {
	case CHANCMD_START:
	    can_start(chip);
	    break;
	case CHANCMD_STOP:
	    can_stop(chip);
	    break;
	case CHANCMD_HWRESET:
	    can_hwreset(chip);
	    break;
	case CHANCMD_LOM_ON:
	    ret = can_set_lom(chip, CI_ON);
	    break;
	case CHANCMD_LOM_OFF:
	    ret = can_set_lom(chip, CI_OFF);
	    break;
	default:
	    ret = -EUCNOSYS;
	    break;
	}
    return ret;
}


#define can_trcancel(chip,ptrqcnt,pret) do {\
    _s16 trstat;\
    SDEP_LOCK_DECLARE_FLAGS(flags);\
    *(pret) = CI_TRCANCEL_NOTRANSMISSION;\
    sdep_lock(&chip->tr.lock, &flags);\
    if (chip->tr.trd_flag) {\
        sdep_trdtimer_stop(chip);\
        chip->tr.trd_flag = TR_DELAY_FLAG_CLEAR;\
        *(pret) = CI_TRCANCEL_DELAYABORTED;\
    };\
    *(ptrqcnt) = (_u16) chip->tr.q->count;\
    chip->tr.q->count = 0;\
    if (chip->cops->transmit_status(chip) == CI_TR_INCOMPLETE) {\
        trstat = chip->cops->transmit_cancel(chip);\
        if (trstat == CI_TR_COMPLETE_ABORT)\
            *(pret) = CI_TRCANCEL_ABORTED;\
        else\
            *(pret) = CI_TRCANCEL_TRANSMITTED;\
    };\
    sdep_unlock(&chip->tr.lock, &flags);\
  } while(0)

#define can_trstat(chip,ptrqcnt,pret) do {\
    SDEP_LOCK_DECLARE_FLAGS(flags);\
    sdep_lock(&(chip)->tr.lock, &flags);\
    *(ptrqcnt) = (_u16) (chip)->tr.q->count;\
    if ((chip)->tr.trd_flag) {\
        *(pret) = CI_TR_DELAY;\
    } else {\
        *(pret) = chip->cops->transmit_status(chip);\
    };\
    sdep_unlock(&chip->tr.lock, &flags);\
  } while(0)

static int can_rcque_resize(struct can_dev *chip, _u16 size) 
{
    int ret;
    SDEP_LOCK_DECLARE_FLAGS(flags);

    if (sdep_atomic_get(&chip->state) != CAN_INIT)
        return -EUCSTATE;

    sdep_lock(&chip->rc.lock, &flags);
    can_quecncl(chip, CIQUE_RC);
    ret = can_quealloc(chip, (_u16) size, CIQUE_RC);
    if (chip->rc.q_thres > chip->rc.q->size)
        chip->rc.q_thres = chip->rc.q->size;
    sdep_unlock(&chip->rc.lock, &flags);
    return ret;
}

static int can_rcque_empty(struct can_dev * chip, _u16 * rcqcnt) 
{
    int ret;
    SDEP_LOCK_DECLARE_FLAGS(flags);

    sdep_lock(&chip->rc.lock, &flags);
    *rcqcnt = (_u16) chip->rc.q->count;
    ret = can_queempty(chip, CIQUE_RC);
    sdep_unlock(&chip->rc.lock, &flags);
    return ret;
}

static int can_rcque_getcnt(struct can_dev * chip, _u16 * rcqcnt) 
{
    SDEP_LOCK_DECLARE_FLAGS(flags);

    if (chip->rc.q == NULL)
        return -EUCINVAL;

    sdep_lock(&chip->rc.lock, &flags);
    *rcqcnt = (_u16) chip->rc.q->count;
    sdep_unlock(&chip->rc.lock, &flags);
    return 0;
}

static int can_rcq_threshold(struct can_dev * chip, _u16 * thres, int getset) 
{
    SDEP_LOCK_DECLARE_FLAGS(flags);

    if (getset == CI_CMD_GET) {
        *thres = chip->rc.q_thres;
    } else {                   // SET
        if (sdep_atomic_get(&chip->state) != CAN_INIT)
            return -EUCSTATE;
        sdep_lock(&chip->rc.lock, &flags);
        if (*thres <= chip->rc.q->size)
            chip->rc.q_thres = *thres;
        else
            chip->rc.q_thres = chip->rc.q->size;
        sdep_unlock(&chip->rc.lock, &flags);
    }
    return 0;
}

static int can_trq_threshold(struct can_dev * chip, _u16 * thres, int getset) 
{
    SDEP_LOCK_DECLARE_FLAGS(flags);

    if (getset == CI_CMD_GET) {
        *thres = chip->tr.q_thres;
    } else {                   // SET
        if (sdep_atomic_get(&chip->state) != CAN_INIT)
            return -EUCSTATE;
        sdep_lock(&chip->tr.lock, &flags);
        if (*thres <= chip->tr.q->size)
            chip->tr.q_thres = *thres;
        else
            chip->tr.q_thres = chip->tr.q->size;
        sdep_unlock(&chip->tr.lock, &flags);
    }
    return 0;
}


int can_queue_operate(struct can_dev * chip, int cmd, _u16 * p) 
{
	int ret = 0;
	
    switch (cmd) {
	case RCQUECMD_TRESH_GET:
	    ret = can_rcq_threshold(chip, p, CI_CMD_GET); //POUT
	    break;
	case RCQUECMD_TRESH_SET:
	    ret = can_rcq_threshold(chip, p, CI_CMD_SET); //PIN
	    break;
	case RCQUECMD_STAT:
		ret = can_rcque_getcnt(chip, p); //POUT
	    break;
	case RCQUECMD_CANCEL:
	    ret = can_rcque_empty(chip, p); //POUT
	    break;
	case RCQUECMD_RESIZE:
	    ret = can_rcque_resize(chip, *p); //PIN
	    break;
	case TRQUECMD_TRESH_GET:
	    ret = can_trq_threshold(chip, p, CI_CMD_GET); //POUT
	    break;
	case TRQUECMD_TRESH_SET:
	    ret = can_trq_threshold(chip, p, CI_CMD_SET); //PIN
	    break;
	case TRQUECMD_STAT:
		can_trstat(chip,p,&ret); //POUT
	    break;
	case TRQUECMD_CANCEL:
		can_trcancel(chip,p,&ret); //POUT
	    break;
	default:
	    ret = -EUCNOSYS;
	    break;
	}
    return ret;
}

int can_hwreg(struct can_dev * chip, int getset, _u32 offset, _u32 *val)
{
    if (getset == CI_CMD_GET) {
        *val = chip->hread(chip, (int) offset);
    } else {                    // SET
        chip->hwrite(chip, (int) offset, *val);
    }
    return 0;
}

void can_post_error(struct can_dev * chip, int errn, _u16 errcnt)
{
    SDEP_LOCK_DECLARE_FLAGS(flags);

    sdep_lock(&chip->err_lock, &flags);
    if (errn == CAN_ERR_SWOVR) chip->swovr_cnt += errcnt;
    else if (errn == CAN_ERR_HWOVR) chip->hwovr_cnt += errcnt;
    chip->errs[errn] += errcnt;
    sdep_unlock(&chip->err_lock, &flags);
    sdep_wakeup_err(chip);
    if (chip->signals & ERR_SIG_FLAG) {
        while(errcnt--) 
            sdep_notify_user(chip, errn + CAN_ERRSIGS_BASE);
    }
}

_u16 can_isr(struct can_dev * chip, _u16 irqn, canmsg_t *frame) 
{
    SDEP_LOCK_DECLARE_FLAGS(flags);
    int ret;

    PDEBUG("enter to isr, irqn = %d\n", irqn);

    if (irqn & INT_RC) {
        PDEBUG("RC_DATA_INT occured...\n");
            // store frame in RC queue
            sdep_lock(&chip->rc.lock, &flags);
            ret = __questor(chip->rc.q, frame);
            sdep_unlock(&chip->rc.lock, &flags);
            if (ret >= 0) {
                sdep_wakeup_recieve(chip);
                if (chip->signals & RX_SIG_FLAG)
                    sdep_notify_user(chip, CAN_EV_RX);
            } else {
                can_post_error(chip, CAN_ERR_SWOVR, 1);
                PERROR("no space left on RX buffer, dropping frame.\n");
            }
        PDEBUG("RC_DATA_INT complete \n");
    }

    if (irqn & INT_TR) {
        PDEBUG("TR_DATA_INT occured...\n");
        if (chip->cops->transmit_status(chip) != CI_TR_INCOMPLETE) {
            sdep_lock(&chip->tr.lock, &flags);
            if (__queretr(chip->tr.q, frame) >= 0) {
                if (frame->flags & FRAME_TRDELAY) {
                    can_delay_transmit(chip, frame);
                } else {
                    chip->cops->transmit(chip, frame);
                }
            }
            sdep_unlock(&chip->tr.lock, &flags);
        } else {
            PERROR("CAN ISR TR: hw transmit buffer is locked!!!!!\n");
        }
        sdep_wakeup_transmit(chip);
        PDEBUG("TR_DATA_INT complete \n");
    }
    if (irqn & INT_HOVR) {
        can_post_error(chip, CAN_ERR_HWOVR, 1);
        PWARN("hardware data overrun (HOVR)\n");
    }

    if (irqn & INT_EWL) {
        can_post_error(chip, CAN_ERR_EWL, 1);
        PWARN 
            ("i/o error counter exceeds the limit of 96 (EWL)\n");
    }

    if (irqn & INT_BOFF) {
        can_stop(chip);
        sdep_lock(&chip->tr.lock, &flags);
        can_queempty(chip, CIQUE_TR);
        sdep_unlock(&chip->tr.lock, &flags);
        can_post_error(chip, CAN_ERR_BOFF, 1);
        PERROR 
            ("i/o error counter exceeds the limit of 255 (BUS OFF)\n");
    }
    
    return irqn;
}
