/*
*  unicandrv-lnx.c
*  Uniform CAN (Unican) for Linux kernel
*
*  Author: Fedor Nedeoglo, 1998-2016
*
*  Marathon Ltd. Moscow, 2016
*/


#include "chai.h"
#include "sysdep.h"
#include "unican.h"


const char drvname_fmt[] =
    "Uniform CAN Interface Driver (unican) v. %d.%d%d 2016\n";

struct unican uc;

#if LINUX_VERSION_CODE >= KERNEL_VERSION(4,15,0)

static void lnx_timer_thunk(struct timer_list* kernel_timer)
{
	//container_of(my_ptr, struct_name, field_name);
	cantr_t *trp = container_of(kernel_timer, cantr_t, trd_timer);
	struct can_dev *chip = container_of(trp, struct can_dev, tr);

    can_delay_transmition_handler(chip);
}

int sdep_trdtimer_create(struct can_dev *chip) 
{
	timer_setup(&chip->tr.trd_timer, lnx_timer_thunk, 0);

    return 0;
}

#else

int sdep_trdtimer_create(struct can_dev *chip) 
{

    init_timer(&chip->tr.trd_timer);
    chip->tr.trd_timer.data = (unsigned long) chip;
    chip->tr.trd_timer.function = (void (*)(unsigned long)) can_delay_transmition_handler;
	
    return 0;
}

#endif

static int lnxcan_open(struct inode *inode, struct file *file)
{
    int ret = 0;
    int err;
    unsigned int minor = MINOR(inode->i_rdev);
    struct can_dev *chip;

    chip = unican_dev_find_by_minor(&uc, minor);
    if (chip == NULL)
        return -EUCNODEV;

    sdep_mutex_lock(&chip->mutex);
    if (sdep_atomic_get(&chip->opened)) {
        PDEBUG("unican_open - channel is busy\n");
        ret = -EUCBUSY;
        goto out;
    }

    init_waitqueue_head(&(chip->wait_rq));      /* init reading wait queue */
    init_waitqueue_head(&(chip->wait_wq));      /* init writing wait queue */
    init_waitqueue_head(&(chip->wait_eq));      /* init error wait queue */

    chip->task = current;
    err = can_open(chip);
    if (err < 0) {
        PDEBUG("unican_open can_open() failed\n");
        ret = err;
        goto out;
    }
    sdep_trdtimer_create(chip);
	file->private_data = (void *) chip;
    sdep_atomic_set(&chip->opened, 1);
out:
    sdep_mutex_unlock(&chip->mutex);
    if (ret >= 0) {
        PDEBUG("driver open - minor %d\n", chip->minor);
    }
    return ret;
}

static int lnxcan_release(struct inode *inode, struct file *file)
{
    struct can_dev *chip = (struct can_dev *) file->private_data;
	
    if (chip == NULL)
        return -EUCNODEV;

    sdep_mutex_lock(&chip->mutex);

    if (sdep_atomic_get(&chip->opened) == 0) {
        sdep_mutex_unlock(&chip->mutex);
        return -EUCINVAL;
    }
    PDEBUG("driver close - minor %d\n", chip->minor);

    sdep_atomic_set(&chip->opened, 0);
    can_release(chip);

    sdep_mutex_unlock(&chip->mutex);
    return 0;
}

#ifdef HAVE_UNLOCKED_IOCTL
static long
    lnxcan_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
#else
static int
    lnxcan_ioctl(struct inode *inode, struct file *file, unsigned int cmd,
    unsigned long arg)
#endif
{
    int ret = 0;
    chipstat_t st;
    canmsg_t wframe;
    canerrs_t errs;
    canparam_t cparam;
    _u32 offset, qinoutf, val = 0;
    struct can_dev *chip = (struct can_dev *) file->private_data;


    if (_IOC_TYPE(cmd) != CAN_IOC_MAGIC)
        return -ENOTTY;

    sdep_mutex_lock(&chip->mutex);
    switch (cmd) {

    case CAN_IOC_READ: //PINOUT
        PDEBUG("can_ioctl CAN_READ\n");
        if (copy_from_user
            ((void *) &cparam, (void *) arg, sizeof(canparam_t))) {
                ret = -EUCMFAULT;
                break;
        }
        ret = (int) can_read(chip, (char *) cparam.p1, (int) cparam.p2);
        break;

    case CAN_IOC_WRITE: //PIN
        PDEBUG("can_ioctl CAN_WRITE\n");
        if (copy_from_user
            ((void *) &wframe, (void *) arg, sizeof(canmsg_t))) {
                ret = -EUCMFAULT;
                break;
        }
        ret = (int) can_write(chip, &wframe);
        break;

    case CAN_IOC_TRANSMIT: //PIN
        PDEBUG("can_ioctl CAN_TRANSMIT\n");
        if (copy_from_user
            ((void *) &wframe, (void *) arg, sizeof(canmsg_t))) {
                ret = -EUCMFAULT;
                break;
        }
        ret = (int) can_transmit(chip, &wframe);
        break;

    case CAN_IOC_SET_BAUDRATE: //PIN
        PDEBUG("can_ioctl CAN_SET_BAUDRATE\n");
        if (copy_from_user
            ((void *) &cparam, (void *) arg, sizeof(canparam_t))) {
                ret = -EUCMFAULT;
                break;
        }
        ret = can_set_baud(chip, (_u32) cparam.p1, (_u32) cparam.p2);
        break;

    case CAN_IOC_SET_FILTER: //PIN
        PDEBUG("can_ioctl CAN_SET_FILTER\n");
        if (copy_from_user
            ((void *) &cparam, (void *) arg, sizeof(canparam_t))) {
                ret = -EUCMFAULT;
                break;
        }
        ret = can_set_filter(chip, cparam.p1, cparam.p2);
        break;

    case CAN_IOC_GET_STATUS: //POUT
        PDEBUG("can_ioctl CAN_GET_STATUS\n");
        ret = can_get_status(chip, &st); 
        if (ret >= 0) {
            if (copy_to_user
                ((void *) arg, (void *) &st, sizeof(chipstat_t))) {
                    ret = -EUCMFAULT;
                    break;
            }
        }
        break;

    case CAN_IOC_GET_ERRS: //POUT
        PDEBUG("can_ioctl CAN_GET_ERRS\n");
        ret = can_get_clear_errs(chip, &errs);
        if (ret >= 0) {
            if (copy_to_user
                ((void *) arg, (void *) &errs, sizeof(canerrs_t))) {
                    ret = -EUCMFAULT;
                    break;
            }
        }
        break;

    case CAN_IOC_CHAN_OPERATE: //PIN
        PDEBUG("can_ioctl CAN_CHAN_OPEARTE\n");
        if (copy_from_user((void *) &val, (void *) arg, sizeof(_u32))) {
            ret = -EUCMFAULT;
            break;
        }
        ret = can_chan_operate(chip, (int) val);
        break;

    case CAN_IOC_QUE_OPERATE:
        PDEBUG("can_ioctl CAN_QUE_OPERATE\n");
        if (copy_from_user
            ((void *) &cparam, (void *) arg, sizeof(canparam_t))) {
                ret = -EUCMFAULT;
                break;
        }
        CANOP_CMD_DECONSTRUCT(&cparam.p1, &qinoutf);
        ret = can_queue_operate(chip, (int) cparam.p1, (_u16 *) &cparam.p2);
        if (ret >= 0 && (qinoutf & CANOP_POUT)) {
            if (copy_to_user
                ((void *) arg, (void *) &cparam, sizeof(canparam_t))) {
                    ret = -EUCMFAULT;
                    break;
            }
        }
        break;

    case CAN_IOC_SW_SIGNAL: //PIN
        PDEBUG("can_ioctl CAN_SW_SIGNAL\n");
        if (copy_from_user((void *) &cparam, (void *) arg, sizeof(canparam_t))) {
            ret = -EUCMFAULT;
            break;
        }
        ret = can_switch_sigs(chip, (int) cparam.p1, (int) cparam.p2);
        break;


    case CAN_IOC_SETMODE:  //PIN
        PDEBUG("can_ioctl CAN_SETMODE\n");
        if (copy_from_user((void *) &val, (void *) arg, sizeof(_u32))) {
            ret = -EUCMFAULT;
            break;
        }
        ret = can_set_mode(chip, (int) val);
        break;


    case CAN_IOC_REG_READ: //PINOUT
        PDEBUG("can_ioctl CAN_REG_READ\n");
        if (copy_from_user
            ((void *) &cparam, (void *) arg, sizeof(canparam_t))) {
                ret = -EUCMFAULT;
                break;
        }
        offset = (_u32) cparam.p1;
        ret = can_hwreg(chip, CI_CMD_GET, offset, &val);
        cparam.p2 = (unsigned long) val;
        if (copy_to_user((void *) arg, (void *) &cparam, sizeof(canparam_t))) {
            ret = -EUCMFAULT;
            break;
        }
        break;

    case CAN_IOC_REG_WRITE: //PIN
        PDEBUG("can_ioctl CAN_REG_WRITE\n");
        if (copy_from_user
            ((void *) &cparam, (void *) arg, sizeof(canparam_t))) {
                ret = -EUCMFAULT;
                break;
        }
        offset = (_u32) cparam.p1; 
        val = (_u32) cparam.p2;
        ret = can_hwreg(chip, CI_CMD_SET, offset, &val);
        break;

    case CAN_IOC_RISE_WTOUT:
        ret =0;
        can_post_error(chip, CAN_ERR_WTOUT, 1);
        break;

    default:
        PDEBUG("can_ioctl UNKNOWN COMMAND\n");
        ret = -EUCNOTTY;       /* set to ENOTTY (not EINVAL) according to POSIX */
        break;
    }
    sdep_mutex_unlock(&chip->mutex);

    return ret;
}

static unsigned int lnxcan_poll(struct file *file, poll_table * wait)
{
    unsigned int mask = 0;
    struct can_dev *chip = (struct can_dev *) file->private_data;


    sdep_mutex_lock(&chip->mutex);

    poll_wait(file, &chip->wait_rq, wait);
    poll_wait(file, &chip->wait_wq, wait);
    poll_wait(file, &chip->wait_eq, wait);

    // read
    if (sdep_recieve_check_state(chip) > 0)
        mask |= POLLIN | POLLRDNORM;

    // write, transmit
    if (sdep_transmit_check_state(chip) > 0)
        mask |= POLLOUT | POLLWRNORM;

    // CAN errors
    if (sdep_error_check_state(chip) > 0)
        mask |= POLLPRI;

    sdep_mutex_unlock(&chip->mutex);
    return mask;
}

static struct file_operations can_fops = {
	owner:           THIS_MODULE,
	poll:            lnxcan_poll,
#ifdef HAVE_UNLOCKED_IOCTL
	unlocked_ioctl:  lnxcan_ioctl,
#else
	ioctl:  	 lnxcan_ioctl,
#endif
	open:            lnxcan_open,
	release:         lnxcan_release
};

static int lnxcan_brdinfo_open(struct inode *inode, struct file *file)
{
    PDEBUG("brdinfo_unican_open called\n");
    return 0;
}

static int lnxcan_brdinfo_release(struct inode *inode, struct file *file)
{
    PDEBUG("brdinfo_unican_open release\n");
    return 0;
}

static int lnxcan_brdinfo_get(canboard_t * bi)
{
    struct can_board *brd;
    int i;
    
    brd = unican_board_find_by_num(&uc, bi->brdnum);

    if (brd == NULL)
        return -EUCNODEV;
    
    bi->brdnum      = brd->num;
    bi->hwver       = brd->hwver;

    strncpy(bi->name, brd->drv->name, CI_BRDSTR_SIZE);
    bi->name[CI_BRDSTR_SIZE - 1] = '\0';

    strncpy(bi->manufact, brd->drv->manufact, CI_BRDSTR_SIZE);
    bi->manufact[CI_BRDSTR_SIZE - 1] = '\0';

    for (i=0;i<CI_CHANS_PER_BOARD;i++) {
        if(brd->chip[i]) {
            bi->chip[i] = brd->chip[i]->minor;
        } else {
            bi->chip[i] = -1;
        }
    }

    return 0;
}

#ifdef HAVE_UNLOCKED_IOCTL
static long
    lnxcan_brdinfo_ioctl(struct file *file, unsigned int cmd,
    unsigned long arg)
#else
static int
    lnxcan_brdinfo_ioctl(struct inode *inode, struct file *file,
    unsigned int cmd, unsigned long arg)
#endif
{
    canboard_t bi;
    int ret = 0;

    PDEBUG("brdinfo_unican_ioctl called\n");
    if (_IOC_TYPE(cmd) != CAN_IOC_MAGIC)
        return -ENOTTY;

    switch (cmd) {
    case CAN_IOC_GETVER:
        if (copy_to_user
            ((void *) arg, (void *) &unican_version, sizeof(_u32))) {
                ret = -EUCMFAULT;
                break;
        }
        ret = 0;
        break;

    case CAN_IOC_GETBRDINFO:
        if (copy_from_user
            ((void *) &bi, (void *) arg, sizeof(canboard_t))) {
                ret = -EUCMFAULT;
                break;
        }
        ret = lnxcan_brdinfo_get(&bi);
        if (ret >= 0) {
            if (copy_to_user
                ((void *) arg, (void *) &bi, sizeof(canboard_t))) {
                    ret = -EUCMFAULT;
                    break;
            }
        }
        break;
    default:
        ret = -EUCNOTTY;        // set to ENOTTY (not EINVAL) according to POSIX
        break;
    }
    return ret;
}

static struct file_operations unican_fops = {
	owner:           THIS_MODULE,
#ifdef HAVE_UNLOCKED_IOCTL
	unlocked_ioctl:  lnxcan_brdinfo_ioctl,
#else
	ioctl:           lnxcan_brdinfo_ioctl,
#endif
	open:            lnxcan_brdinfo_open,
	release:         lnxcan_brdinfo_release
};

/*================================================================
Init functions
================================================================*/

int setup_proc_interface(void);
void cleanup_proc_interface(void);

static char *chip_name[] = {
    "unknown CAN-controller",
    "SJA1000"
};

static char *numbers[] = {
    "first",
    "second",
    "third",
    "fourth"
};


static int _count_print_candevs(struct unican *ucn) 
{
    int i;
    struct can_board *brd;
    int foundchip = 0, foundboard = 0;
    
    list_for_each_entry(brd, &ucn->boards, list) {
        foundboard++;
        PINFO("%s board:\n", brd->drv->name);
        for (i = 0; i < CI_CHANS_PER_BOARD; i++) {
            if (brd->chip[i]) {
                foundchip++;
                PRINT("  %-7s chip %s at: 0x%lx, Irq %d, channel(minor) %d\n",
                    numbers[i], chip_name[brd->chip[i]->type],
                    (unsigned long) brd->chip[i]->base_address,
                    brd->chip[i]->irqn, brd->chip[i]->minor);
            }
        }
    }
    
    if (foundchip > 0) {
        PINFO("total - %d CAN controllers on %d boards\n", foundchip,
            foundboard);
    } else {
        PINFO("no controllers found, driver is removed\n");
        return 0;
    }
    return foundchip;    
}




static int __init init_canmodule(void)
{
    int ret;
    int foundchip = 0;

    PRINT(drvname_fmt, VERMAJ(unican_version), VERMIN(unican_version),
        VERSUB(unican_version));

    unican_init(&uc);
    foundchip = _count_print_candevs(&uc); 

    if (foundchip > 0) {
        ret = register_chrdev_region(MKDEV(UNICAN_MAJOR, 0), 1, "unican");
        if (ret) {
            PERROR("can't register major number %d.\n", UNICAN_MAJOR);
            goto err;
        }
        ret =
            register_chrdev_region(MKDEV(CAN_MAJOR, 0), CI_CHAN_NUMS,
            "can");
        if (ret) {
            PERROR("can't register major number %d.\n", CAN_MAJOR);
            unregister_chrdev_region(MKDEV(UNICAN_MAJOR, 0), 1);
            goto err;
        }

        cdev_init(&uc.unican_cdev, &unican_fops);
        uc.unican_cdev.owner = THIS_MODULE;
        uc.unican_cdev.ops = &unican_fops;
        ret = cdev_add(&uc.unican_cdev, MKDEV(UNICAN_MAJOR, 0), 1);
        if (ret) {
            PERROR("can't add cdev interface for major number %d.\n",
                UNICAN_MAJOR);
            unregister_chrdev_region(MKDEV(UNICAN_MAJOR, 0), 1);
            unregister_chrdev_region(MKDEV(CAN_MAJOR, 0), CI_CHAN_NUMS);
            goto err;
        }

        cdev_init(&uc.can_cdev, &can_fops);
        uc.can_cdev.owner = THIS_MODULE;
        uc.can_cdev.ops = &can_fops;
        ret = cdev_add(&uc.can_cdev, MKDEV(CAN_MAJOR, 0), CI_CHAN_NUMS);
        if (ret) {
            PERROR("can't add cdev interface for major number %d.\n",
                CAN_MAJOR);
            unregister_chrdev_region(MKDEV(UNICAN_MAJOR, 0), 1);
            unregister_chrdev_region(MKDEV(CAN_MAJOR, 0), CI_CHAN_NUMS);
            cdev_del(&uc.unican_cdev);
            goto err;
        }
    } else {
        ret = -1;
        goto err;
    }


#ifdef CONFIG_PROC_FS

    if (setup_proc_interface() < 0) {
        cleanup_proc_interface();
        PERROR("can not setup proc fs, driver is removed\n");
        ret = -1;
        goto errcleanup;
    }
#endif
    return 0;
err:
//    spi_unregister_driver(&spican_driver);

    return ret;
errcleanup:
    unregister_chrdev_region(MKDEV(UNICAN_MAJOR, 0), 1);
    unregister_chrdev_region(MKDEV(CAN_MAJOR, 0), CI_CHAN_NUMS);
    cdev_del(&uc.unican_cdev);
    cdev_del(&uc.can_cdev);
    unican_cleanup(&uc);
    return ret;
}

static void __exit cleanup_canmodule(void)
{
    PDEBUG("cleanup_module called\n");

    unregister_chrdev_region(MKDEV(UNICAN_MAJOR, 0), 1);
    unregister_chrdev_region(MKDEV(CAN_MAJOR, 0), CI_CHAN_NUMS);
    cdev_del(&uc.unican_cdev);
    cdev_del(&uc.can_cdev);

#ifdef CONFIG_PROC_FS
    cleanup_proc_interface();
#endif
    unican_cleanup(&uc);
}

module_init(init_canmodule);
module_exit(cleanup_canmodule);

/*================================================================
Proc fs functions
================================================================*/

#ifdef CONFIG_PROC_FS
static struct proc_dir_entry *candir, *candrv_info;

//extern _s16 CiChipStatToStr(chipstat_t * status, chstat_desc_t * desc);

#if LINUX_VERSION_CODE < KERNEL_VERSION(3,10,0)
#define PROC_PRINT(fmt,arg...) do { len += sprintf(page + len,fmt,##arg); } while(0)
#else
#define PROC_PRINT(fmt,arg...) do { seq_printf (sfile,fmt,##arg); } while(0)
#endif


#if LINUX_VERSION_CODE < KERNEL_VERSION(3,10,0)
static int proc_unican_show(char *page, char **start, off_t off, int count, 
                            int *eof, void *data)
#else
#include <linux/seq_file.h>
static int proc_unican_show(struct seq_file *sfile, void *not_used)
#endif
{
	
	int len = 0;
    int i;
    struct can_board *brd;
	int foundchip = 0, foundboard = 0;

	PROC_PRINT(drvname_fmt, VERMAJ(unican_version),
		        VERMIN(unican_version), VERSUB(unican_version));

    list_for_each_entry(brd, &uc.boards, list) {
        foundboard++;
        PROC_PRINT( "%s board (%s):\n",
                    brd->drv->name,
                    brd->drv->manufact);
        for (i = 0; i < CI_CHANS_PER_BOARD; i++) {
            if (brd->chip[i]) {
                foundchip++;
                PROC_PRINT("  %-7s controller %s at: 0x%lx, Irq %d, channel(minor) %d\n",
                            numbers[i],
                            chip_name[brd->chip[i]->type],
                            (unsigned long) brd->chip[i]->base_address,
                            brd->chip[i]->irqn, 
                            brd->chip[i]->minor);
            }
        }
    }

	PROC_PRINT("\ntotal - %d CAN controllers on %d boards\n",
		        foundchip, foundboard);
#if LINUX_VERSION_CODE < KERNEL_VERSION(3,10,0)
	*eof = 1;
#endif
	return len;
}

extern _s16 CiChipStatToStr(chipstat_t * status, chstat_desc_t * desc);

#if LINUX_VERSION_CODE < KERNEL_VERSION(3,10,0)
static int proc_can_show(char *page, char **start, off_t off, int count, 
                         int *eof, void *data)
#else
static int proc_can_show(struct seq_file *sfile, void *not_used)
#endif

{
    int len = 0, i;
    chstat_desc_t *desc;
    chipstat_t st;
#if LINUX_VERSION_CODE < KERNEL_VERSION(3,10,0)
    struct can_dev *chip = (struct can_dev *) data;
#else
    struct can_dev *chip = (struct can_dev *) sfile->private;
#endif

    if (chip == NULL)
        return 0;
    sdep_mutex_lock(&chip->mutex);

    PROC_PRINT("%s %s on %s (%s) ", numbers[chip->minor],
        chip_name[chip->type],
        chip->brd->drv->name,
        chip->brd->drv->manufact);

    PROC_PRINT( "at minor %d\n", chip->minor);
    PROC_PRINT( "driver state:  ");
    if (sdep_atomic_get(&chip->opened))
        PROC_PRINT( "opened\n");
    else
        PROC_PRINT( "closed\n");

    PROC_PRINT( "base addr   : 0x%lx\n", (unsigned long) chip->base_address);
    PROC_PRINT( "irq         : %d\n", chip->irqn);

    if (sdep_atomic_get(&chip->opened)) {
        if (chip->cops->get_status(chip, &st) < 0) {
            PROC_PRINT("error getting status of CAN cantroller at minor %d\n",
                chip->minor);
            goto out;
        }
        desc = sdep_malloc(sizeof(chstat_desc_t));
        if (desc) {
            CiChipStatToStr(&st, desc);
            for (i = 0; i < CI_CHSTAT_STRNUM; i++) {
                if (desc->name[i][0] == '\0')
                    continue;
                if (strcmp(desc->name[i], "base addr") != 0
                    && strcmp(desc->name[i], "irq") != 0) {
                        PROC_PRINT( "%-12s: %s\n", desc->name[i], desc->val[i]);
                }
            }
            sdep_free(desc);
        }
    }

out:
    sdep_mutex_unlock(&chip->mutex);
#if LINUX_VERSION_CODE < KERNEL_VERSION(3,10,0)
    *eof = 1;
#endif
    return len;
}

#undef PROC_PRINT


#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,10,0)

static int proc_unican_open(struct inode *inode, struct file *file)
{
	return single_open(file, proc_unican_show, NULL);
}

static const struct file_operations proc_unican_fops = {
	.owner = THIS_MODULE,
	.open = proc_unican_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};


static int proc_can_open(struct inode *inode, struct file *file)
{
	return single_open(file, proc_can_show, PDE_DATA(inode));
}

static const struct file_operations proc_can_fops = {
	.owner = THIS_MODULE,
	.open = proc_can_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

#endif // if LINUX_VERSION_CODE >= KERNEL_VERSION(3,10,0)

int setup_proc_interface(void)
{
    struct can_dev *chip;
    char pname[7];

    candir = proc_mkdir("can", NULL);
    if (candir == NULL)
        return -1;
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,30)
    candir->owner = THIS_MODULE;
#endif

#if LINUX_VERSION_CODE < KERNEL_VERSION(3,10,0)
    candrv_info =
        create_proc_read_entry("unican", 0444, candir, proc_unican_show,
        NULL);
#else
	candrv_info = 
		proc_create_data("unican", 0444, candir, &proc_unican_fops, NULL);
#endif	
    if (candrv_info == NULL)
        return -1;
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,30)
    candrv_info->owner = THIS_MODULE;
#endif
	
    list_for_each_entry(chip, &uc.devs, list) {
        sprintf(pname, "%d", chip->minor);
#if LINUX_VERSION_CODE < KERNEL_VERSION(3,10,0)
        chip->pentry =
            create_proc_read_entry(pname, 0444, candir, proc_can_show, (void *) chip);
#else
        chip->pentry =
            proc_create_data(pname, 0444, candir, &proc_can_fops, (void *) chip);
#endif
        if (chip->pentry == NULL)
            return -1;
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,30)
        chip->pentry->owner = THIS_MODULE;
#endif
    }
    return 0;
}

void cleanup_proc_interface(void)
{
    struct can_dev *chip;
    char pname[7];


    list_for_each_entry(chip, &uc.devs, list) {
        sprintf(pname, "%d", chip->minor);
        remove_proc_entry(pname, candir);
    }

    remove_proc_entry("unican", candir);
    remove_proc_entry("can", NULL);
}

#endif                          //CONFIG_PROC_FS


MODULE_AUTHOR("Fedor Nedeoglo");
MODULE_DESCRIPTION("Uniform CAN-bus Interface Driver, Marathon Ltd.");
MODULE_LICENSE("GPL");
