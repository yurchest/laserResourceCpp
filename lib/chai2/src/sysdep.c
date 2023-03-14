/*
*  sysdep.c
*  Unican Linux
*
*  Author: Fedor Nedeoglo, 1998-2015
*
*  Marathon Ltd. Moscow, 2015
*
*/

#include <chai.h>
#include <sysdep.h>
#include <unican.h>

#ifdef LINUX

int can_sigs[CAN_SIGNALS_N] = {
    /* RX */     SIGRTMAX - 6,
    /* TX */     SIGRTMAX - 5,
    /* BOFF */   SIGRTMAX - 4,
    /* EWL */    SIGRTMAX - 3,
    /* HOVR */   SIGRTMAX - 2,
    /* SOVR */   SIGRTMAX - 1,
    /* WTOUT */  SIGRTMAX
};

void sdep_notify_user(struct can_dev *chip, int ev_n)
{
    siginfo_t si;

    si.si_signo = can_sigs[ev_n];
    si.si_errno = 0;
    si.si_code = SI_KERNEL;
    si.si_pid = 0;
    si.si_uid = 0;
    si.si_addr = (void *) ((uintptr_t) chip->minor);
    send_sig_info(can_sigs[ev_n], &si, chip->task);
}

inline _u64 sdep_getts(void)
{
    struct timeval t;

    do_gettimeofday(&t);
    return (_u64) ( (_u64) t.tv_sec * 1000000ULL + (_u64) t.tv_usec);
}

// returns: 0 - sleep, 1 - wakeup
int sdep_transmit_check_state(struct can_dev *chip)
{

    if (chip->tr.q_thres == chip->tr.q->size) {
        if (chip->tr.q->count == 0
            && chip->cops->transmit_status(chip) != CI_TR_INCOMPLETE
            && !chip->tr.trd_flag) {
                return 1;
        }
    } else {
        if ((chip->tr.q->size - chip->tr.q->count) >= chip->tr.q_thres) {
            return 1;
        }
    }
    return 0;
}



// returns: 0 - sleep, 1 - wakeup
int sdep_recieve_check_state(struct can_dev *chip)
{
    if (chip->rc.q->count >= chip->rc.q_thres) {
        return 1;
    }
    return 0;
}

// returns: 0 - sleep, 1 - wakeup
int sdep_error_check_state(struct can_dev *chip)
{
    if (chip->errs[CAN_ERR_BOFF] != 0 || chip->errs[CAN_ERR_EWL] != 0
        || chip->errs[CAN_ERR_SWOVR] != 0 || chip->errs[CAN_ERR_HWOVR] != 0
        || chip->errs[CAN_ERR_WTOUT] != 0) {

            return 1;
    }
    return 0;
}

#endif /*LINUX*/
