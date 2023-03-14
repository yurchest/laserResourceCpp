#ifdef __KERNEL__
#include <chai.h>
#include <sysdep.h>
#include <unican.h>
#else
#include <stdio.h>
#include <chai.h>
#include <unican.h>
#endif                          //__KERNEL__

#ifdef WIN32
#define snprintf _snprintf
#endif

static void byte2bin(char *buf, unsigned char byte)
{
    int i;

    buf[0] = '\0';
    for (i = 0; i < 8; i++) {
        if ((byte >> i) & 0x1)
            buf[8 - i - 1] = '1';
        else
            buf[8 - i - 1] = '0';
    }
    buf[8] = '\0';              /* add trailing zero */
}

_s16 CiChipStatToStr(chipstat_t * status, chstat_desc_t * desc)
{
    int i;
    char buf[128];
    sja1000stat_t *st = (sja1000stat_t *) status;

    for (i = 0; i < CI_CHSTAT_STRNUM; i++) {
        desc->name[i][0] = '\0';
        desc->val[i][0] = '\0';
    }
    snprintf(desc->name[0], CI_CHSTAT_MAXLEN, "CAN state");
    if (st->state == CAN_INIT)
        snprintf(desc->val[0], CI_CHSTAT_MAXLEN, "INIT");
    else
        snprintf(desc->val[0], CI_CHSTAT_MAXLEN, "RUNNING");
    snprintf(desc->name[1], CI_CHSTAT_MAXLEN, "base addr");
    snprintf(desc->val[1], CI_CHSTAT_MAXLEN, "0x%lx", (unsigned long) st->baddr);
    snprintf(desc->name[2], CI_CHSTAT_MAXLEN, "irq");
    snprintf(desc->val[2], CI_CHSTAT_MAXLEN, "%d", st->irq);
    snprintf(desc->name[3], CI_CHSTAT_MAXLEN, "hardware ovr");
    snprintf(desc->val[3], CI_CHSTAT_MAXLEN, "%lu", (unsigned long) st->hovr_cnt);
    snprintf(desc->name[4], CI_CHSTAT_MAXLEN, "software ovr");
    snprintf(desc->val[4], CI_CHSTAT_MAXLEN, "%lu", (unsigned long) st->sovr_cnt);
    switch (st->type) {
    case SJA1000:
        snprintf(desc->name[5], CI_CHSTAT_MAXLEN, "CAN chip");
        snprintf(desc->val[5], CI_CHSTAT_MAXLEN, "SJA1000");
        byte2bin(buf, st->mode);
        snprintf(desc->name[6], CI_CHSTAT_MAXLEN, "MOD");
        snprintf(desc->val[6], CI_CHSTAT_MAXLEN, "0x%.2hx (%s)", st->mode,
            buf);
        byte2bin(buf, st->stat);
        snprintf(desc->name[7], CI_CHSTAT_MAXLEN, "ST");
        snprintf(desc->val[7], CI_CHSTAT_MAXLEN, "0x%.2hx (%s)", st->stat,
            buf);
        byte2bin(buf, st->inten);
        snprintf(desc->name[8], CI_CHSTAT_MAXLEN, "IER");
        snprintf(desc->val[8], CI_CHSTAT_MAXLEN, "0x%.2hx (%s)", st->inten,
            buf);
        byte2bin(buf, st->ecc);
        snprintf(desc->name[9], CI_CHSTAT_MAXLEN, "ECC");
        snprintf(desc->val[9], CI_CHSTAT_MAXLEN, "0x%.2hx (%s)", st->ecc,
            buf);
        byte2bin(buf, st->ewl);
        snprintf(desc->name[10], CI_CHSTAT_MAXLEN, "EWLR");
        snprintf(desc->val[10], CI_CHSTAT_MAXLEN, "0x%.2hx (%s)", st->ewl,
            buf);
        byte2bin(buf, st->rxec);
        snprintf(desc->name[11], CI_CHSTAT_MAXLEN, "RXERR");
        snprintf(desc->val[11], CI_CHSTAT_MAXLEN, "0x%.2hx (%s)", st->rxec,
            buf);
        byte2bin(buf, st->txec);
        snprintf(desc->name[12], CI_CHSTAT_MAXLEN, "TXERR");
        snprintf(desc->val[12], CI_CHSTAT_MAXLEN, "0x%.2hx (%s)", st->txec,
            buf);
        byte2bin(buf, st->rxmc);
        snprintf(desc->name[13], CI_CHSTAT_MAXLEN, "RMC");
        snprintf(desc->val[13], CI_CHSTAT_MAXLEN, "0x%.2hx (%s)", st->rxmc,
            buf);
        snprintf(desc->name[14], CI_CHSTAT_MAXLEN, "acode");
        snprintf(desc->val[14], CI_CHSTAT_MAXLEN, "0x%.8lx", (unsigned long) st->acode);
        snprintf(desc->name[15], CI_CHSTAT_MAXLEN, "amask");
        snprintf(desc->val[15], CI_CHSTAT_MAXLEN, "0x%.8lx", (unsigned long) st->amask);
        byte2bin(buf, st->btr0);
        snprintf(desc->name[16], CI_CHSTAT_MAXLEN, "BTR0");
        snprintf(desc->val[16], CI_CHSTAT_MAXLEN, "0x%.2hx (%s)", st->btr0,
            buf);
        byte2bin(buf, st->btr1);
        snprintf(desc->name[17], CI_CHSTAT_MAXLEN, "BTR1");
        snprintf(desc->val[17], CI_CHSTAT_MAXLEN, "0x%.2hx (%s)", st->btr1,
            buf);
        byte2bin(buf, st->outctl);
        snprintf(desc->name[18], CI_CHSTAT_MAXLEN, "OCR");
        snprintf(desc->val[18], CI_CHSTAT_MAXLEN, "0x%.2hx (%s)",
            st->outctl, buf);
        byte2bin(buf, st->clkdiv);
        snprintf(desc->name[19], CI_CHSTAT_MAXLEN, "CDR");
        snprintf(desc->val[19], CI_CHSTAT_MAXLEN, "0x%.2hx (%s)",
            st->clkdiv, buf);
        break;
    default:
        break;
    }
    return 0;
}
