/*
*  boards-conf.c
* 
*  boards configuration
*
*
*/

#include <chai.h>
#include <sysdep.h>
#include <unican.h>


extern struct can_board_driver cbpci_drv;
extern struct can_board_driver cbpci_exp_drv;


int __register_all_drivers(struct unican *ucn)
{
    int ret;

    ret = unican_driver_register (ucn, &cbpci_drv); 
    if (ret < 0)
        return ret;

    ret = unican_driver_register (ucn, &cbpci_exp_drv); 
    if (ret < 0)
        return ret;

    return 0;
}

