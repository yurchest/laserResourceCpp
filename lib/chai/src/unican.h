/*
*  unican.h - Uniform CAN Driver (Unican)
*
*
*  Author: Fedor Nedeoglo, 1998-2016
*
*  Marathon Ltd. Moscow, 2016
*/  

#ifndef __UNICAN_H
#define __UNICAN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "sysdep.h"

#define CI_CHANS_PER_BOARD 4


#define SFF_ACCEPT_FLAG 0x08
#define EFF_ACCEPT_FLAG 0x10

#define ERR_SIG_FLAG 0x08
#define  RX_SIG_FLAG 0x02
#define  TX_SIG_FLAG 0x01

    // names of CAN hardware interrupts bits
#define INT_RC        0x1
#define INT_TR        0x2
#define INT_HOVR      0x4
#define INT_EWL       0x8
#define INT_BOFF      0x10
#define INT_WUP       0x20

#define CANMODE_11ON  1
#define CANMODE_11OFF 2
#define CANMODE_29ON  3
#define CANMODE_29OFF 4


enum { 
    CICB_RX = 0, 
    CICB_TX, 
    CICB_ERR, 
    CICB_NUMS 
};

enum {
	CHANCMD_START = 0,
	CHANCMD_STOP,
	CHANCMD_HWRESET,
	CHANCMD_LOM_ON,
	CHANCMD_LOM_OFF,
	RCQUECMD_TRESH_GET,
	RCQUECMD_TRESH_SET,
	RCQUECMD_STAT,
	RCQUECMD_CANCEL,
	RCQUECMD_RESIZE,
	TRQUECMD_TRESH_GET,
	TRQUECMD_TRESH_SET,
	TRQUECMD_STAT,
	TRQUECMD_CANCEL
};

#ifdef _KERNEL_


#define CHIP_TYPES_NUM 8


    typedef struct {
        canmsg_t * d;
        _u16 sloc;
        _u16 rloc;
        _u16 size;
        _u16 count;
    } canque_t;



    struct chip_operations {
        void (*stall) (struct can_dev *);
        void (*init) (struct can_dev *);
        void (*release) (struct can_dev *);
        void (*hwreset) (struct can_dev *);
        void (*stop) (struct can_dev *);
        void (*start) (struct can_dev *);
        _u16 (*fast_isr) (struct can_dev *chip, canmsg_t *);
        _s16(*transmit) (struct can_dev *, canmsg_t *);
        _s16(*transmit_status) (struct can_dev *);
        _s16(*transmit_cancel) (struct can_dev *);
        _s16(*set_baud) (struct can_dev *, _u32 bt0, _u32 bt1);
        _s16(*set_filter) (struct can_dev *, _u32 acode, _u32 amask);
        _s16(*get_status) (struct can_dev *, chipstat_t *);
        _s16(*set_lom) (struct can_dev *, int onoff);
    };

    enum { 
        TR_DELAY_FLAG_CLEAR = 0, 
        TR_DELAY_FLAG_SET = 1 
    };

    typedef struct {
        _u8 trd_flag;           // delay transmit flag
        _u16 q_thres;           // trqueue treshold
        sdep_timer_t trd_timer; // delayed transmition timer
        canque_t *q;            // tr queue (fifo)
        canmsg_t trd_frame;     // delay trasmit frame buffer
        sdep_lock_t lock;
    } cantr_t;


    typedef struct {
        _u16 q_thres;           // rcqueue treshold
        _u64 time_start;        // starting point for time stamp
        canque_t *q;            // rc queue (fifo)
        sdep_lock_t lock;
    } canrc_t;


#ifdef LINUX
// defined in sysdep.h
struct can_board;
#endif


    struct can_dev {
        // old backward compatibility fields
        _s32 irqn;
        _u32 base_address;
        _u32 hwovr_cnt;
        _u32 swovr_cnt;
        
        // io resources 
        void * iores;

        _u32 (*hread)  (struct can_dev *chip, _u32 shift);
        void (*hwrite) (struct can_dev *chip, _u32 shift, _u32 val);
        void (*hreset) (struct can_dev *chip);
        struct chip_operations *cops;
        
        sdep_atomic_t opened;
        sdep_atomic_t state;   // CAN_RUNNING, CAN_INIT
    
        _u16 flags;         /* 2 - transmit progress
                               3 - accept 11bit messages
                               4 - accept 29bit messages
                               */
                               
        _u16 signals;       /* 0 - CAN_SIG_TX enable
                               1 - CAN_SIG_RX enable
                               2 - CAN_SIG_BOFF enable
                               3 - CAN_SIG_HOVR enable
                               4 - CAN_SIG_EWL enable 
                               */
                              
        _s16 type;            // type of chip
        
        _u16 errs[CAN_ERRS_N];


        sdep_lock_t err_lock;
        sdep_mutex_t mutex;
        cantr_t tr;
        canrc_t rc;

#ifdef LINUX
        struct list_head list;
        struct can_board *brd;
        _s32 minor;
        struct proc_dir_entry *pentry; // proc fs members
        struct task_struct *task;
        wait_queue_head_t wait_rq;
        wait_queue_head_t wait_wq;
        wait_queue_head_t wait_eq;
#endif
    };


// iores should be allocated on heap memory
#define CANDEV_IORES_GET(pchip,typeofres) ((typeofres *)((pchip)->iores))

#define CANDEV_IORES_ALLOC(typeofres) (typeofres *) sdep_malloc(sizeof(typeofres))


/*
#define CANDEV_DEV_IORES_ASSIGN(pchip,piores,phread,phwrite,phreset)\
        do {\
            (pchip)->iores = (void *)(piores);\
            (pchip)->hread = (phread);\
            (pchip)->hwrite = (phwrite);\
            (pchip)->hreset = (phreset);\
             } while 0
*/

// EXPORTS
    extern struct chip_operations sja1000_cops;
    extern void can_delay_transmition_handler(struct can_dev *chip);

    // from unicandrv.c
    extern const _u32 unican_version;
    extern int can_open(struct can_dev *chip);
    extern void can_release(struct can_dev *chip);
    extern _u16 can_isr(struct can_dev * chip, _u16 irqn, canmsg_t *frame);
    // from unicandrv.c for ioctl
    extern int can_chan_operate(struct can_dev * chip, int cmd);
    extern int can_queue_operate(struct can_dev * chip, int cmd, _u16 * p);
    extern int can_read(struct can_dev *chip, char *buf, int count);
    extern int can_write(struct can_dev *chip, canmsg_t * frame);
    extern int can_transmit(struct can_dev *chip, canmsg_t * frame);
    extern int can_set_baud(struct can_dev *chip, _u32 bt0, _u32 bt1);
    extern int can_set_filter(struct can_dev *chip, _u32 acode, _u32 amask);
    extern int can_get_status(struct can_dev *chip, chipstat_t * st);
    extern int can_get_clear_errs(struct can_dev *chip, canerrs_t * errs);
    extern int can_switch_sigs(struct can_dev *chip, int sig_type, int onoff);
    extern int can_set_mode(struct can_dev *chip, int mode);
    extern int can_hwreg(struct can_dev * chip, int getset, _u32 offset, _u32 *val);
    extern void can_post_error(struct can_dev * chip, int errn, _u16 errcnt);

#endif  /* __KERNEL__ */

#ifdef __cplusplus
}
#endif

#endif  /* __UNICAN_H */
