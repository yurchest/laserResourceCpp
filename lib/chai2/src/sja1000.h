#ifndef __SJA1000_H
#define __SJA1000_H

#define CBIT_0  0x01
#define CBIT_1  0x02
#define CBIT_2  0x04
#define CBIT_3  0x08
#define CBIT_4  0x10
#define CBIT_5  0x20
#define CBIT_6  0x40
#define CBIT_7  0x80

#define MOD        0            /* mode, rw in both mode, except some bits */
#define CMR        1            /* command, wo in both mode */
#define SR         2            /* status, ro in both mode */
#define IR         3            /* interrupt, ro in both mode */
#define IER        4            /* interrupt enable, rw in both mode */
#define BTR0       6            /* btr0, ro in operating, rw in reset */
#define BTR1       7            /* btr1, ro in operating, rw in reset */
#define OCR        8            /* output control, ro in operating, rw in reset */
#define ALC        11           /* arbitration lost capture, ro in both mode */
#define ECC        12           /* error code capture, ro in both mode */
#define EWLR       13           /* error warning limit, ro in operating, rw in reset */
#define RXERR      14           /* RX error counter, ro in operating, rw in reset */
#define TXERR      15           /* TX error counter, ro in operating, rw in reset */
#define RMC        29           /* RX message counter, ro in both mode */
#define CDR        31           /* clock divider, rw in both mode , except some bits */

#define RXB_BASE   16           /* ro in operating mode [16-28] */
#define TXB_BASE   16           /* wo in operating mode [16-28] */
#define ACR_BASE   16           /* rw in reset mode [16-19] */
#define AMR_BASE   20           /* rw in reset mode [20-23] */

#endif                          /* __SJA1000_H */
