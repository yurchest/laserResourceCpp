/*
*  sysdep.h 
*  Linux
*
*  Author: Fedor Nedeoglo, 1998-2015
*
*  Marathon Ltd. Moscow, 2015
*
*/

#ifndef __SYSDEP_H
#define __SYSDEP_H



typedef struct {
    unsigned long p1;
    unsigned long p2;
} canparam_t;

/*
*  Codes for CAN Driver ioctl() call.
*/
#define CAN_IOC_MAGIC 'u'

#define CAN_IOC_WRITE              _IO(CAN_IOC_MAGIC, 0)
#define CAN_IOC_TRANSMIT           _IO(CAN_IOC_MAGIC, 1)
#define CAN_IOC_READ               _IO(CAN_IOC_MAGIC, 2)
#define CAN_IOC_SET_BAUDRATE       _IO(CAN_IOC_MAGIC, 3)
#define CAN_IOC_SET_FILTER         _IO(CAN_IOC_MAGIC, 4)
#define CAN_IOC_GET_ERRS           _IO(CAN_IOC_MAGIC, 5)
#define CAN_IOC_GET_STATUS         _IO(CAN_IOC_MAGIC, 6)
#define CAN_IOC_CHAN_OPERATE       _IO(CAN_IOC_MAGIC, 7)
#define CAN_IOC_QUE_OPERATE        _IO(CAN_IOC_MAGIC, 8)
#define CAN_IOC_SW_SIGNAL          _IO(CAN_IOC_MAGIC, 9)
#define CAN_IOC_SETMODE            _IO(CAN_IOC_MAGIC, 10)
#define CAN_IOC_GETVER             _IO(CAN_IOC_MAGIC, 11)
#define CAN_IOC_REG_READ           _IO(CAN_IOC_MAGIC, 12)
#define CAN_IOC_REG_WRITE          _IO(CAN_IOC_MAGIC, 14)

#define CAN_IOC_RISE_WTOUT         _IO(CAN_IOC_MAGIC, 29)
#define CAN_IOC_GETBRDINFO         _IO(CAN_IOC_MAGIC, 30)

// unican error codes 
#include <linux/errno.h>
#define EUCOK         0            /* success */
#define EUCGEN        1
#define EUCBUSY       EBUSY        /* device or resourse busy */
#define EUCMFAULT     EFAULT       /* memory fault */
#define EUCSTATE      EBADRQC      /* function can't be called for chip in current state */
#define EUCINCALL     EBADR        /* invalid call, function can't be called for this object */
#define EUCINVAL      EINVAL       /* invalid parameter */
#define EUCACCES      EACCES       /* can not access resource */
#define EUCNOSYS      ENOSYS       /* function or feature not implemented */
#define EUCIO         EIO          /* input/output error */
#define EUCNODEV      ENODEV       /* no such device */
#define EUCINTR       EAGAIN       /* call was interrupted by event */
#define EUCRESTARTSYS ERESTART
#define EUCNOTTY      ENOTTY
#define EUCNORES      ENOBUFS
#define EUCTOUT       ETIME


#define CANOP_PIN   0x1UL
#define CANOP_POUT  0x2UL

#define CANOP_CMD_CONSTRUCT(pout,cmd,outflag) \
  *(pout) =  ( (unsigned long) (cmd) & 0xFFFFUL ) | ((unsigned long) (outflag) << 16)

#define CANOP_CMD_DECONSTRUCT(pcmd,poutflag) do {\
  *(poutflag) = (unsigned long) (*(pcmd) >> 16);\
  *(pcmd) = (unsigned long) (*(pcmd) & 0xFFFFUL);\
} while (0)



//
//  Kernel only part
//
#ifdef __KERNEL__
#include <linux/module.h>
#include <linux/version.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/spinlock.h>
#include <linux/ioport.h>
#include <linux/slab.h>
#include <linux/vmalloc.h>
#include <linux/ioctl.h>
#include <linux/wait.h>
#include <linux/poll.h>
#include <linux/fs.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/types.h>
#include <linux/pci.h>
#include <linux/time.h>
#include <linux/cdev.h>
#ifdef CONFIG_PROC_FS
#include <linux/proc_fs.h>
#endif

#include <linux/version.h>
/*
#ifndef KERNEL_VERSION
#define KERNEL_VERSION(a,b,c) ((a)*65536+(b)*256+(c))
#endif
*/

#if LINUX_VERSION_CODE > KERNEL_VERSION(4, 11, 0)
#include <linux/sched/signal.h>
#endif

#include <asm/io.h>
#if LINUX_VERSION_CODE > KERNEL_VERSION(3, 3, 0)
#include <asm/switch_to.h>
#else
#include <asm/system.h>
#endif
#include <asm/segment.h>
#include <asm/uaccess.h>

#define CAN_MAJOR 121     /* major number for unican character Linux driver */
#define UNICAN_MAJOR 122  /* major number for CiBoardInfo() function */


#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,29)
#include <linux/semaphore.h>
#endif

#ifndef _u64
#define _u64 unsigned long long
#endif

struct can_dev;

struct unican {
    
    // linked lists of devs, boards and drivers
    struct list_head  devs;
    struct list_head  boards;
    struct list_head  drivers;
    
    // character interfaces
    struct cdev       unican_cdev;
    struct cdev       can_cdev;
};

struct can_board_driver;

struct can_board {
    _s32 num;
    _u32 hwver;
    struct can_dev *chip[4];
    struct can_board_driver *drv;
    void   *priv;   // private data for driver 
    struct list_head list;
};

struct can_board_driver {
    char *name;
    char *manufact;
    int (*find_init_all) (struct unican *ucn);
    void (*remove) (struct unican *ucn, struct can_board *brd);
    
    struct list_head list;
};

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

enum {
    CAN_EV_RX = 0,
    CAN_EV_TX = 1
};

#define CAN_ERRSIGS_BASE 2

enum {
    CAN_SIGNALS_N = (CAN_ERRSIGS_BASE + CAN_ERRS_N)
};

////////////////////////////
// Atomic integer ops
//

#define sdep_atomic_t atomic_t
#define sdep_atomic_set(pavar, val) do { atomic_set( pavar, (int) val); mb(); } while(0)
#define sdep_atomic_get(pavar) atomic_read(pavar)

////////////////////////////
// kernel transmit delay timer
//
typedef unsigned long sdep_timerarg_t;

typedef struct timer_list sdep_timer_t;

//extern void sdep_trdtimer_start(struct can_dev *chip, _u32 mks);
//extern void sdep_trdtimer_stop(struct can_dev *chip);
#define sdep_trdtimer_start(pchip,mks) do { (pchip)->tr.trd_timer.expires = jiffies + ((mks)/1000UL)*HZ/1000UL; add_timer(&(pchip)->tr.trd_timer);} while(0)
#define sdep_trdtimer_stop(pchip) del_timer_sync(&(pchip)->tr.trd_timer)
/* ======================================================================
Print and debug
====================================================================== */
#ifdef DEBUG_CANDRV
#define PDEBUG(fmt,arg...) do { printk("unican: "); printk(fmt,##arg); } while(0)
#define PWARN(fmt,arg...) do { printk("unican WARNING: "); printk(fmt,##arg); } while(0)
#else
#define PDEBUG(fmt,arg...)
#define PWARN(fmt,arg...)
#endif
#define PERROR(fmt,arg...) do { printk("unican ERROR: "); printk(fmt,##arg); } while(0)
#define PRINT(fmt,arg...) do { printk(fmt,##arg); } while(0)
#define PINFO(fmt,arg...) do { printk("unican: "); printk(fmt,##arg); } while(0)
/* ======================================================================
locks
====================================================================== */
typedef struct mutex sdep_mutex_t;
#define sdep_mutex_init(pmutex)   mutex_init(pmutex)
#define sdep_mutex_lock(pmutex)   mutex_lock(pmutex)
#define sdep_mutex_unlock(pmutex) mutex_unlock(pmutex)

#define SDEP_LOCK_DECLARE_FLAGS(f) unsigned long f
typedef spinlock_t sdep_lock_t;

#define sdep_lock_init(plock)  spin_lock_init(plock)
#define sdep_lock(plock, pflags) spin_lock_irqsave((plock), *(pflags))
#define sdep_unlock(plock, pflags) spin_unlock_irqrestore((plock), *(pflags))
#define sdep_lock_free(plock)

#define sdep_udelay(usec) udelay(usec)

/* ======================================================================
memory funcs
====================================================================== */
#define sdep_malloc(size) kmalloc((size_t) (size), GFP_KERNEL)
#define sdep_free(p) kfree(p)
#define sdep_memcpy(pdest, psrc, size) memcpy((pdest), (psrc), (size_t) (size))
#define sdep_memset(pdest, val, size) memset((pdest), (val), (size_t) (size))

extern void sdep_notify_user(struct can_dev *chip, int ev_n);
extern _u64 sdep_getts(void);

extern int sdep_transmit_check_state(struct can_dev *chip);
//extern void sdep_wakeup_transmit(struct can_dev *chip);
#define sdep_wakeup_transmit(pchip) if (sdep_transmit_check_state(pchip) > 0) wake_up_interruptible(&(pchip)->wait_wq)

extern int sdep_recieve_check_state(struct can_dev *chip);
//extern void sdep_wakeup_recieve(struct can_dev *chip);
#define sdep_wakeup_recieve(pchip) if (sdep_recieve_check_state(pchip) > 0) wake_up_interruptible(&(pchip)->wait_rq)

extern int sdep_error_check_state(struct can_dev *chip);
#define sdep_wakeup_err(pchip) wake_up_interruptible(&((pchip)->wait_eq))

// from unicandrv-lnx.c
extern struct unican uc;

// from boards.c
extern struct can_dev *   unican_dev_find_by_minor(struct unican *ucn, _s32 minor);
extern struct can_board * unican_board_find_by_num(struct unican *ucn, _s32 num);
extern struct can_dev *   unican_dev_alloc(_s16 type, _s32 irq, _u32 base_address, void *iores,
                _u32 (*hread)  (struct can_dev *chip, _u32 shift),
                void (*hwrite) (struct can_dev *chip, _u32 shift, _u32 val),
                void (*hreset) (struct can_dev *chip)
                );
extern struct can_board * unican_board_alloc_register(struct unican *ucn, 
                   struct can_board_driver *drv,
                   struct can_dev *chip0,
                   struct can_dev *chip1,
                   struct can_dev *chip2,
                   struct can_dev *chip3,
                   _u32 hwver,
                   void *pdata
                   );
extern int  unican_driver_register (struct unican *ucn, struct can_board_driver *drv);
extern int  unican_init(struct unican *ucn);
extern void unican_cleanup(struct unican *ucn);

#endif /*__KERNEL__*/

#endif /*__SYSDEP_H*/
