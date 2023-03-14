/*
*  boards.c
*  boards module for Unican
*
*  Author: Fedor Nedeoglo, 2016
*
*  Marathon Ltd. Moscow, 2016
*
*/

#include "chai.h"
#include "sysdep.h"
#include "unican.h"

struct can_dev * unican_dev_find_by_minor(struct unican *ucn, _s32 minor)
{
    struct can_dev *chip;

    list_for_each_entry(chip, &ucn->devs, list) {
        if (chip->minor == minor)
            return chip;
    }
    return NULL;
}

struct can_board * unican_board_find_by_num(struct unican *ucn, _s32 num)
{
    struct can_board *brd;

    list_for_each_entry(brd, &ucn->boards, list) {
        if (brd->num == num)
            return brd;
    }
    return NULL;
}

// iores should be allocated on the heap 
struct can_dev * unican_dev_alloc(_s16 type, _s32 irq, _u32 base_address, void *iores,
                _u32 (*hread)  (struct can_dev *chip, _u32 shift),
                void (*hwrite) (struct can_dev *chip, _u32 shift, _u32 val),
                void (*hreset) (struct can_dev *chip)
                )
{
    struct can_dev *chip;

    chip = sdep_malloc(sizeof(struct can_dev));
    if (chip == NULL) {
        PERROR("can't allocate memory for struct can_dev\n");
        return NULL;
    }
    memset(chip, 0, sizeof(struct can_dev));
    sdep_mutex_init(&chip->mutex);
    sdep_atomic_set(&chip->opened, 0);
    chip->rc.q = NULL;
    chip->tr.q = NULL;
    chip->iores = NULL;
    chip->type = type;
    chip->base_address = base_address;
    chip->irqn = irq;
    
    chip->iores =  iores;
    chip->hread =  hread;
    chip->hwrite = hwrite;
    chip->hreset = hreset;
    
    switch (type) {

    case SJA1000:
        chip->cops = &sja1000_cops;
        break;

    default:
        sdep_free(chip);
        chip = NULL;
        break;
    }
    
    return chip;
}


static struct can_board * __board_alloc(_u32 hwver, void *pdata)
{
    struct can_board *brd;

    brd = sdep_malloc(sizeof(struct can_board));
    if (brd == NULL) {
        PERROR("can't allocate memory for struct can_board\n");
        return NULL;
    }
    memset(brd, 0, sizeof(struct can_board));
    brd->hwver = hwver;
    brd->priv  = pdata;

    return brd;
}


static void __dev_register(struct unican *ucn,
                          struct can_dev *chip)
{
    int devnums;
    struct can_dev *dev;

    if (chip) {
        INIT_LIST_HEAD(&chip->list);

        list_add_tail(&(chip->list), &ucn->devs);
        devnums = 0;
        list_for_each_entry(dev, &ucn->devs, list) {
            devnums++;
        }
        chip->minor = devnums -1;
    }
}

struct can_board *
unican_board_alloc_register(struct unican *ucn, 
                   struct can_board_driver *drv,
                   struct can_dev *chip0,
                   struct can_dev *chip1,
                   struct can_dev *chip2,
                   struct can_dev *chip3,
                   _u32 hwver,
                   void *pdata)

{
    int i, brdnums;
    struct can_board *tmp;
    struct can_board *brd = __board_alloc(hwver, pdata);

    if (!brd) 
       return brd;
    brd->drv = drv;
    brd->chip[0] = chip0;
    brd->chip[1] = chip1;
    brd->chip[2] = chip2;
    brd->chip[3] = chip3;
    for (i=0; i<CI_CHANS_PER_BOARD; i++) {
        if(brd->chip[i]) {
            __dev_register(ucn, brd->chip[i]);
            brd->chip[i]->brd = brd;
        }
    }

    INIT_LIST_HEAD(&brd->list);

    list_add_tail(&(brd->list), &ucn->boards);
    brdnums = 0;
    list_for_each_entry(tmp, &ucn->boards, list) {
        brdnums++;
    }
    brd->num = brdnums -1;

    return brd;
}


// example definition of can_board_driver    
// ------------------------------------------
// struct can_board_driver cbpci_drv = {
//          .name          = "CAN-bus-PCI",
//          .manufact      = "Marathon Ltd.",
//          .find_init_all = cbpci_find_init_all,
//          .remove        = cbpci_remove,
//          };
// ret = unican_driver_register (&ucn, &cbpci_drv); 
// if (ret < 0) { //error... };

int unican_driver_register (struct unican *ucn, struct can_board_driver *drv)
{
    INIT_LIST_HEAD(&drv->list);
 
    list_add_tail(&(drv->list), &ucn->drivers);
    return 0;
}


static void __dev_remove_free(struct unican *ucn, struct can_dev *chip)
{
    struct can_dev *chip_iter;
    struct list_head *pos, *q;
 
    list_for_each_safe(pos, q, &ucn->devs) {
        chip_iter = list_entry(pos, struct can_dev, list);
        if (chip_iter == chip) {
            list_del(pos);
            if (chip_iter->iores) 
                sdep_free(chip_iter->iores);            
            sdep_free(chip_iter);
            chip = NULL;
        }
    }
}

static void __board_free(struct unican *ucn, struct can_board *brd)
{
    int i;
    
    for (i = 0; i < CI_CHANS_PER_BOARD; i++) {
        if (brd->chip[i]) {
            __dev_remove_free(ucn, brd->chip[i]);
        }
    }
    sdep_free(brd);
    brd = NULL;
}

static void __boards_cleanup(struct unican *ucn)
{
    struct can_board *brd;
    struct list_head *pos, *q;

    // remove boards and candevs
    list_for_each_safe(pos, q, &ucn->boards) {
        brd = list_entry(pos, struct can_board, list);
        list_del(pos);
        if (brd->drv->remove)
            brd->drv->remove(ucn, brd);
        __board_free(ucn, brd);
    }

    // remove drivers
    list_for_each_safe(pos, q, &ucn->drivers) {
        list_del(pos);
    }
}

static int __boards_find_init_all(struct unican *ucn)
{
    //int ret;
    struct can_board_driver *drv;
 
    list_for_each_entry(drv, &ucn->drivers, list) {
        if (drv->find_init_all)
            drv->find_init_all(ucn);
    }
    return 0;
}


extern int  __register_all_drivers(struct unican *ucn);

int unican_init(struct unican *ucn)
{

    INIT_LIST_HEAD(&ucn->devs);
    INIT_LIST_HEAD(&ucn->boards);
    INIT_LIST_HEAD(&ucn->drivers);

    if ( __register_all_drivers(ucn) < 0 ) {
        return -1;
    }

    if ( __boards_find_init_all(ucn) < 0 )
        return -1;

    return 0;
}

void unican_cleanup(struct unican *ucn)
{
    __boards_cleanup(ucn);
}

