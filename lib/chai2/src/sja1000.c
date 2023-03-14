/*
*  sja1000.c
*  Phillips SJA1000 chip operations for Unican
*
*  Author: Fedor Nedeoglo, 1998-2016
*
*  Marathon Ltd. Moscow, 2016
*
*/

#include <chai.h>
#include <sysdep.h>
#include <unican.h>

#include <sja1000.h>


#define _trbuf_released(chip) ( chip->hread((chip),SR) & 0x04 )
#define _tr_complete(chip) ( chip->hread((chip),SR) & 0x08 )
#define _set_transmitreq(chip) chip->hwrite((chip),CMR, 0x01)
#define _aborttr(chip) chip->hwrite((chip),CMR, 0x02)
#define _release_rcbuf(chip) chip->hwrite((chip),CMR, 0x04)
#define _disableall_intrs(chip) chip->hwrite((chip),IER, 0x0)
#define _enable_intrs(chip) chip->hwrite((chip),IER, 0x1f)
#define _clear_dataovr(chip) chip->hwrite((chip),CMR, 0x08)




//#define sja1000_is_ints_enabled(pchip)  (int) chip->hread((pchip), IER)

static void _swto_PeliCAN(struct can_dev *chip) 
{
    _u32 cdr = chip->hread(chip, CDR);

    cdr |= CBIT_7;
    chip->hwrite(chip, CDR, cdr);
} 

static inline void _set_singlefilter(struct can_dev *chip) 
{
    _u32 mode = chip->hread(chip, MOD);

    mode |= CBIT_3;
    chip->hwrite(chip, MOD, mode);
} 

static inline void _set_resetreq(struct can_dev *chip) 
{
    _u32 mode;

    while (((mode = chip->hread(chip, MOD)) & CBIT_0) == 0) {
        mode |= CBIT_0;
        chip->hwrite(chip, MOD, mode);
    }
}

static inline void _clear_resetreq(struct can_dev *chip) 
{
    _u32 mode;

    while (((mode = chip->hread(chip, MOD)) & CBIT_0) != 0) {
        mode &= ~CBIT_0;
        chip->hwrite(chip, MOD, mode);
    }
}


void sja1000_stall(struct can_dev *chip) 
{
    _u32 reg;

    chip->hreset(chip);

    // switch to PeliCAN mode
    reg = chip->hread(chip, CDR);
    reg |= CBIT_7;
    chip->hwrite(chip, CDR, reg);
    // stop chip;
    reg = chip->hread(chip, MOD);
    reg |= CBIT_0;
    chip->hwrite(chip, MOD, reg);

    _disableall_intrs(chip);
} 

void sja1000_stop(struct can_dev *chip) 
{
    _disableall_intrs(chip);
    _set_resetreq(chip);
} 

void sja1000_start(struct can_dev *chip) 
{
    _enable_intrs(chip);
    _clear_resetreq(chip);
} 

_s16 sja1000_transmit_status(struct can_dev *chip) 
{
    _u32 stat = chip->hread(chip, SR);

    if (stat & 0x04) {         // transmit buffer is released
        if (stat & 0x08) {      // transmit complete
            return CI_TR_COMPLETE_OK;   // complete - success
        } else {
            return CI_TR_COMPLETE_ABORT;       // complete - abort
        }

    }
    return CI_TR_INCOMPLETE;
}

_s16 sja1000_set_baudrate(struct can_dev * chip, _u32 bt0, _u32 bt1) 
{
    _u32 mod = chip->hread(chip, MOD);

    if (!(mod & CBIT_0))
        return -1;
    chip->hwrite(chip, BTR0, bt0);
    chip->hwrite(chip, BTR1, bt1);
    return 0;
}


_s16 sja1000_set_filter(struct can_dev * chip, _u32 acode, _u32 amask) 
{
    _u32 mod = chip->hread(chip, MOD);
    _u32 mask = ~(amask);

    if (!(mod & CBIT_0))
        return -1;

    if (chip->flags & SFF_ACCEPT_FLAG) {       // standart frame format 
        chip->hwrite(chip, AMR_BASE, (_u8) ((mask >> 3) & 0xff));
        chip->hwrite(chip, AMR_BASE + 1,
            (_u8) ((mask << 5) & 0xff) | 0x1f);
        chip->hwrite(chip, AMR_BASE + 2, 0xff);    // don't care
        chip->hwrite(chip, AMR_BASE + 3, 0xff);     // don't care
        chip->hwrite(chip, ACR_BASE, (_u8) ((acode >> 3) & 0xff));
        chip->hwrite(chip, ACR_BASE + 1,
            (_u8) ((acode << 5) & 0xff) | 0x1f);
    } else {                   // extended frame format
        chip->hwrite(chip, AMR_BASE, (_u8) ((mask >> 21) & 0xff));
        chip->hwrite(chip, AMR_BASE + 1, (_u8) ((mask >> 13) & 0xff));
        chip->hwrite(chip, AMR_BASE + 2, (_u8) ((mask >> 5) & 0xff));
        chip->hwrite(chip, AMR_BASE + 3,
            (_u8) (((mask << 3) & 0xff) | 0x7));
        chip->hwrite(chip, ACR_BASE, (_u8) ((acode >> 21) & 0xff));
        chip->hwrite(chip, ACR_BASE + 1,
            (_u8) ((acode >> 13) & 0xff));
        chip->hwrite(chip, ACR_BASE + 2, (_u8) ((acode >> 5) & 0xff));
        chip->hwrite(chip, ACR_BASE + 3, (_u8) ((acode << 3) & 0xff));
    }
    return 0;
}


_s16 sja1000_set_lom(struct can_dev * chip, int onoff) 
{
    _u32 mode = chip->hread(chip, MOD);

    if (onoff) {
        mode |= CBIT_1;
    } else {
        mode &= ~CBIT_1;
    }

    chip->hwrite(chip, MOD, mode);
    return 0;
}

void sja1000_hardreset(struct can_dev *chip) 
{
    _u32 mode_save = chip->hread(chip, MOD);

    chip->hreset(chip);
    _set_resetreq(chip);
    _swto_PeliCAN(chip);
    _set_singlefilter(chip);
    if (!(mode_save & CBIT_0))
        _clear_resetreq(chip);
}

void sja1000_init(struct can_dev *chip) 
{
    int retries = 0;

    chip->hreset(chip);
    _swto_PeliCAN(chip);

    for (retries = 0; retries < 32; retries++) {
        _release_rcbuf(chip);
    }

    sja1000_stop(chip);

    // set output control to 0x5e 
    chip->hwrite(chip, OCR, 0x5e);
    _set_singlefilter(chip);

    // Don't use filtering 
    sja1000_set_filter(chip, 0x0UL, 0x0UL);
    sja1000_set_baudrate(chip, BCI_500K);
}

void sja1000_release(struct can_dev *chip) 
{
    _disableall_intrs(chip);
    _set_resetreq(chip);
} 


static inline void
    sja1000_load_frame(struct can_dev *chip, canmsg_t * frame) 
{
    _u32 txinfo = 0;
    _u32 id = 0;
    int i;

    if (frame->len > 8)
        frame->len = 8;
    txinfo = frame->len & 0xf;
    if (frame->flags & FRAME_EFF) {    // extended frame format
        txinfo |= CBIT_7;       // mark frame as extended
        id = frame->id << 3;
        chip->hwrite(chip, TXB_BASE + 1, (_u8) ((id >> 24) & 0xff));
        chip->hwrite(chip, TXB_BASE + 2, (_u8) ((id >> 16) & 0xff));
        chip->hwrite(chip, TXB_BASE + 3, (_u8) ((id >> 8) & 0xff));
        chip->hwrite(chip, TXB_BASE + 4, (_u8) (id & 0xff));
        if (frame->flags & FRAME_RTR) {
            txinfo |= CBIT_6;
        } else {
            for (i = 0; i < frame->len; i++)
                chip->hwrite(chip, TXB_BASE + i + 5, frame->data[i]);
        }
    } else {                   // standart frame format
        id = frame->id << 5;
        chip->hwrite(chip, TXB_BASE + 1, (_u8) ((id >> 8) & 0xff));
        chip->hwrite(chip, TXB_BASE + 2, (_u8) (id & 0xff));
        if (frame->flags & FRAME_RTR) {
            txinfo |= CBIT_6;
        } else {
            for (i = 0; i < frame->len; i++)
                chip->hwrite(chip, TXB_BASE + i + 3, frame->data[i]);
        }
    }
    chip->hwrite(chip, TXB_BASE, txinfo);

#ifdef DEBUG_CANDRV
    PDEBUG("sja1000_load_frame() - id=0x%lx ", frame->id);
    if (frame->flags & FRAME_EFF)
        PRINT("EFF ");
    else
        PRINT("SFF ");
    PRINT("flags=0x%x\n", frame->flags);
#endif
}



_s16 sja1000_transmit_cancel(struct can_dev *chip) 
{
    _aborttr(chip);
    while (1) {
        if (_trbuf_released(chip))
            break;
    }
    return sja1000_transmit_status(chip);
}


/* return 0 on success, -1 on error (busy) */ 
_s16 sja1000_transmit(struct can_dev * chip, canmsg_t * frame) 
{

    if (!_trbuf_released(chip)) {
        return -1;
    }
    sja1000_load_frame(chip, frame);
    _set_transmitreq(chip);
    return 0;
}

_s16 sja1000_get_status(struct can_dev * chip, chipstat_t * stat) 
{
    _s32 i;
    sja1000stat_t * st = (sja1000stat_t *) stat;

    st->type = chip->type;
#ifdef LINUX
    st->brdnum = chip->brd->num; // in windows this field is filled by chai.dll at user level
#endif
    if (sdep_atomic_get(&chip->state) == CAN_INIT) {
        st->state = CAN_INIT;
    } else {
        st->state = CAN_RUNNING;
    }
    st->irq = chip->irqn;
    st->baddr = chip->base_address;
    st->hovr_cnt = chip->hwovr_cnt;
    st->sovr_cnt = chip->swovr_cnt;

    st->mode = chip->hread(chip, MOD);
    st->stat = chip->hread(chip, SR);
    st->inten = chip->hread(chip, IER);
    st->clkdiv = chip->hread(chip, CDR);
    st->ecc = chip->hread(chip, ECC);
    st->ewl = chip->hread(chip, EWLR);
    st->rxec = chip->hread(chip, RXERR);
    st->txec = chip->hread(chip, TXERR);
    st->rxmc = chip->hread(chip, RMC);
    st->amask = 0;
    for (i = 0; i <= 2; i++) {
        st->amask =
            (st->amask | (_u32) chip->hread(chip, AMR_BASE + i)) << 8;
    }
    st->amask = st->amask | (_u32) chip->hread(chip, AMR_BASE + 3);

    st->acode = 0;
    for (i = 0; i <= 2; i++) {
        st->acode =
            (st->acode | (_u32) chip->hread(chip, ACR_BASE + i)) << 8;
    }
    st->acode = st->acode | (_u32) chip->hread(chip, ACR_BASE + 3);

    st->btr0 = chip->hread(chip, BTR0);
    st->btr1 = chip->hread(chip, BTR1);
    st->outctl = chip->hread(chip, OCR);
    return 0;
}

/* returns: 0 - success, <0 - no frame to retrieve */ 
static inline _s16 sja1000_retrieve(struct can_dev *chip, canmsg_t * frame) 
{
    _u32 rxinfo = 0;
    _s32 i;

    if (!(chip->hread(chip, SR) & 0x01))
        return -1;
    frame->flags = 0;
    frame->id = 0;

    rxinfo = chip->hread(chip, RXB_BASE);
    frame->len = rxinfo & 0xf;
    if (frame->len > 8)
        frame->len = 8;
    if (rxinfo & CBIT_7) {     // extended frame format
        if (!(chip->flags & EFF_ACCEPT_FLAG)) {
            _release_rcbuf(chip);
            return -1;
        }
        frame->flags |= FRAME_EFF;
        frame->id = 0;
        for (i = 1; i <= 3; i++) {
            frame->id =
                (frame->
                id | (_u32) chip->hread(chip, RXB_BASE + i)) << 8;
        }
        frame->id =
            (frame->id | (_u32) chip->hread(chip, RXB_BASE + 4)) >> 3;

        if (rxinfo & CBIT_6) {
            frame->flags |= FRAME_RTR;
        } else {
            for (i = 0; i < frame->len; i++)
                frame->data[i] = chip->hread(chip, RXB_BASE + 5 + i);
        }
    } else {                   // standart frame format
        if (!(chip->flags & SFF_ACCEPT_FLAG)) {
            _release_rcbuf(chip);
            return -1;
        }
        frame->id = ((_u32) chip->hread(chip, RXB_BASE + 1)) << 8;
        frame->id =
            (frame->id | (_u32) chip->hread(chip, RXB_BASE + 2)) >> 5;
        if (rxinfo & CBIT_6) {
            frame->flags |= FRAME_RTR;
        } else {
            for (i = 0; i < frame->len; i++)
                frame->data[i] = chip->hread(chip, RXB_BASE + 3 + i);
        }
    }
    frame->ts = (_u32) ( (_u64) sdep_getts() - (_u64) chip->rc.time_start);

    _release_rcbuf(chip);

#ifdef DEBUG_CANDRV
    PDEBUG("sja1000_retrieve_frame() - id=0x%lx ", frame->id);
    if (frame->flags & FRAME_EFF)
        PRINT("EFF ");
    else
        PRINT("SFF ");
    PRINT("flags=0x%x\n", frame->flags);
#endif
    return 0;
}


_u16 sja1000_fast_isr(struct can_dev *chip, canmsg_t *rcframe) 
{
    _u32 int_reg, stat_reg;
    _u16 ret = 0;

    int_reg = chip->hread(chip, IR);
    stat_reg = chip->hread(chip, SR);

    if (int_reg & 0x01) {
        if (sja1000_retrieve(chip, rcframe) >= 0) {
            ret |= INT_RC;
        }
    }
    if (int_reg & 0x02)
        ret |= INT_TR;
    if (int_reg & 0x08) {
        ret |= INT_HOVR;
        _clear_dataovr(chip);
    }
    if (int_reg & 0x04) {
        if (stat_reg & 0x40)
            ret |= INT_EWL;
        if (stat_reg & 0x80)
            ret |= INT_BOFF;
    }
    if (int_reg & 0x10)
        ret |= INT_WUP;
    return ret;
}

struct chip_operations sja1000_cops = { 
    /* stall */           sja1000_stall, 
    /* init: */           sja1000_init, 
    /* release: */        sja1000_release, 
    /* hwreset: */        sja1000_hardreset, 
    /* stop:    */        sja1000_stop, 
    /* start:   */        sja1000_start, 
    /* fast_isr: */       sja1000_fast_isr,
    /* transmit: */       sja1000_transmit, 
    /* transmit_status */ sja1000_transmit_status, 
    /* transmit_cancel */ sja1000_transmit_cancel, 
    /* set_baud:   */     sja1000_set_baudrate, 
    /* set_filter:   */   sja1000_set_filter, 
    /* get_status: */     sja1000_get_status, 
    /* set_lom: */        sja1000_set_lom 
};

