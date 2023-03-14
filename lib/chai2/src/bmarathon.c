/*
*  bmarathon.c
*  Marathon Ltd. boards drivers for Unican
*
*  Author: Fedor Nedeoglo, 1998-2016
*
*  Marathon Ltd. Moscow, 2016
*
*/

#include "chai.h"
#include "sysdep.h"
#include "unican.h"
#include "sja1000.h"

#define CBPCI_NAME         "CAN-bus-PCI"
#define CBPCI_EXP_NAME     "CAN-bus-PCIe"
#define MANUFACTURER_STR   "Marathon Ltd. Moscow"

// vendor id and device id for CAN-bus-PCI 
#define CAN_bus_PCI_VendorID 0x10b5
#define CAN_bus_PCI_DeviceID 0x2715     //0x9050 0x2715 

#define CAN_bus_PCI_EXP_VendorID 0x10b5
#define CAN_bus_PCI_EXP_DeviceID 0x3432


typedef struct {
    void * io;  // mapped io mem area
    void * ra;  // mapped reset mem area
} memiores_t;


static _u32 iomem_hread(struct can_dev *chip, _u32 shift) 
{
    return (_u32) ioread8( (void *) ( (_u8 *) (CANDEV_IORES_GET(chip,memiores_t)->io) + shift) );
}

static void iomem_hwrite(struct can_dev *chip, _u32 shift, _u32 val) 
{
    iowrite8( (_u8) val, (void *) ( (_u8 *) ((CANDEV_IORES_GET(chip,memiores_t)->io) + shift)) );
}

static void iomem_hreset(struct can_dev *chip) 
{
    iowrite8( (_u8) 1, (void *) (CANDEV_IORES_GET(chip,memiores_t)->ra) );
    sdep_udelay(100);
}


#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,22)
#define  sdep_pci_find_device(vendor,device,from) pci_find_device(vendor, device, from)
#else
#define  sdep_pci_find_device(vendor,device,from) pci_get_device(vendor, device, from)
#endif

#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,19)
#define CAN_HW_ISR_DECLARE(func_name,dev_id) static irqreturn_t func_name (int irq, void *dev_id)
#else
#define CAN_HW_ISR_DECLARE(func_name,dev_id) static irqreturn_t func_name (int irq, void *dev_id, struct pt_regs *regs)
#endif

#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,18)
#define CAN_HW_ISR_FLAGS IRQF_SHARED
#else
#define CAN_HW_ISR_FLAGS SA_SHIRQ
#endif



// ================================================================
// CAN-bus-PCI driver
// ================================================================

int  cbpci_find_init_all(struct unican *ucn);
void cbpci_remove(struct unican *ucn, struct can_board *brd);

struct can_board_driver cbpci_drv = {
          .name          = CBPCI_NAME,
          .manufact      = MANUFACTURER_STR,
          .find_init_all = cbpci_find_init_all,
          .remove        = cbpci_remove,
          };
// ret = unican_register_candriver (&ucn, &cbpci_drv);
// if (ret < 0) { //error... };

CAN_HW_ISR_DECLARE(cbpci_interrupt, board_ptr)
{
    _u16 irqn0 = 0, irqn1 = 0;
    struct can_board *brd = (struct can_board *) board_ptr;
    canmsg_t frame;

    if (sdep_atomic_get(&brd->chip[0]->opened)) {
        irqn0 = brd->chip[0]->cops->fast_isr(brd->chip[0], &frame);
        if (irqn0)
            can_isr(brd->chip[0], irqn0, &frame);
    }
    if (sdep_atomic_get(&brd->chip[1]->opened)) {
        irqn1 = brd->chip[1]->cops->fast_isr(brd->chip[1], &frame);
        if (irqn1)
            can_isr(brd->chip[1], irqn1, &frame);
    }
    if (irqn0 || irqn1)
        return IRQ_HANDLED;

    return IRQ_NONE;
}


int _cbpci_map_pcimem(struct pci_dev *pdev, 
                      _u32 *baddr1, _u32 *baddr2,
                      _u8 **vio1, _u8 **vrst1, 
                      _u8 **vio2, _u8 **vrst2 )
{
    _u32 res1, res2;

    if (pci_enable_device(pdev)) {
        PWARN("can't enable pci device\n");
        return -1;
    }
    if (pdev->irq == 0) {
        PWARN("no irq allocation by PCI\n");
        return -1;
    }
    // first channel
    *baddr1 = pci_resource_start(pdev, 2);
    if ((*vio1 =
        (_u8 *) ioremap(*baddr1, pci_resource_len(pdev, 2))) == NULL) {
            PERROR("can't map io addres 0x%lx\n", (unsigned long) *baddr1);
            return -1;
    }
    res1 = pci_resource_start(pdev, 3);
    if ((*vrst1 =
        (_u8 *) ioremap(res1, pci_resource_len(pdev, 3))) == NULL) {
            iounmap(*vio1);
            PERROR("can't map reset addres 0x%lx\n", (unsigned long) res1);
            return -1;
    }
    // second channel 
    *baddr2 = pci_resource_start(pdev, 4);
    if ((*vio2 =
        (_u8 *) ioremap(*baddr2, pci_resource_len(pdev, 4))) == NULL) {
            iounmap(*vio1);
            iounmap(*vrst1);
            PERROR("can't map io addres 0x%lx\n", (unsigned long) *baddr2);
            return -1;
    }
    res2 = pci_resource_start(pdev, 5);
    if ((*vrst2 =
        (_u8 *) ioremap(res2, pci_resource_len(pdev, 5))) == NULL) {
            iounmap(*vio1);
            iounmap(*vrst1);
            iounmap(*vio2);
            PERROR("can't map reset addres 0x%lx\n", (unsigned long) res2);
            return -1;
    }

    return 0;
}

static int _cbpci_register(struct unican *ucn, struct pci_dev *pdev)
{
    _u32 addr1, addr2;
    _u8 *vio1, *vrst1, *vio2, *vrst2, *lcr;
    struct can_dev *chip1, *chip2;
    struct can_board *brd;
    memiores_t *iores;


    if (_cbpci_map_pcimem(pdev, &addr1, &addr2, 
                  &vio1, &vrst1, &vio2, &vrst2 )<0)
        return -1;

    if ( (iores = CANDEV_IORES_ALLOC(memiores_t)) == NULL )
        goto fail;
    iores->io = vio1;
    iores->ra = vrst1;
    chip1 = unican_dev_alloc(SJA1000, pdev->irq, addr1, 
            (void *) iores, iomem_hread, iomem_hwrite, iomem_hreset);
    if (chip1 == NULL) {
        sdep_free(iores);
        PWARN("can't register chip 1\n");
        goto fail;
    }

    if ( (iores = CANDEV_IORES_ALLOC(memiores_t)) == NULL ) {
        goto fail_chip2;
    }
    iores->io = vio2;
    iores->ra = vrst2;
    chip2 = unican_dev_alloc(SJA1000, pdev->irq, addr2, 
            (void *) iores, iomem_hread, iomem_hwrite, iomem_hreset);
    if (chip2 == NULL) {
        sdep_free(iores);
        PWARN("can't register chip 2\n");
        goto fail_chip2;
    }

    brd = unican_board_alloc_register (ucn, &cbpci_drv, 
                   chip1, chip2, NULL, NULL, UNICAN_VER(2, 0, 0), NULL);
    if (brd == NULL) {
        PERROR("can't register board\n");
        goto fail_board;
    }

    chip1->cops->stall(chip1);
    chip2->cops->stall(chip2);

    if (request_irq (chip1->irqn, cbpci_interrupt,
        CAN_HW_ISR_FLAGS, "can", (void *) brd)) {
            PERROR("can't assign hardware irq handler\n");
    }

    // Write Interrupt Control/Status Register in Local Configuration space
    addr1 = pci_resource_start(pdev, 0);
    if ((lcr = (_u8 *) ioremap(addr1, pci_resource_len(pdev, 0))) == NULL) {
        PERROR ("can't map Local Configuration Registers address 0x%lx\n",
                (unsigned long) addr1);
    } else {
        // enable PCI, Local_1, Local_2 interrupts
        iowrite32(0x49, (void *) (lcr + 0x4c));     
        iounmap(lcr);
    }

return 0;

fail_board:
        sdep_free(chip2->iores);
        sdep_free(chip2);
fail_chip2:
    sdep_free(chip1->iores);
    sdep_free(chip1);
fail:
    iounmap(vio1);
    iounmap(vrst1);
    iounmap(vio2);
    iounmap(vrst2);
    return -1;
}

int cbpci_find_init_all(struct unican *ucn)
{
    struct pci_dev *pdev = NULL;

    PDEBUG ("cbpci_find_init_all() is called\n");
    while ((pdev = sdep_pci_find_device(CAN_bus_PCI_VendorID,
                           CAN_bus_PCI_DeviceID, pdev)) != NULL) {
       if ( _cbpci_register(ucn, pdev) < 0 ) {
           PERROR("CAN_bus_PCI can't alloc and register board\n");
       }
    }
    return 0;
}

//
// free hardware resources here
//

void cbpci_remove(struct unican *ucn, struct can_board *brd)
{
    int i;
    memiores_t *iores;

    free_irq(brd->chip[0]->irqn, brd);
    for (i = 0; i < 2; i++) {
        if (brd->chip[i]) {
            brd->chip[i]->cops->stall(brd->chip[i]);
            iores = CANDEV_IORES_GET(brd->chip[i],memiores_t);
            iounmap(iores->io);
            iounmap(iores->ra);
        }
    }
}


// ================================================================
// CAN-bus-PCIe driver
// ================================================================

int  cbpci_exp_find_init_all(struct unican *ucn);
void cbpci_exp_remove(struct unican *ucn, struct can_board *brd);

struct can_board_driver cbpci_exp_drv = {
          .name          = CBPCI_EXP_NAME,
          .manufact      = MANUFACTURER_STR,
          .find_init_all = cbpci_exp_find_init_all,
          .remove        = cbpci_exp_remove,
          };
// ret = unucan_register_candriver (&ucn, &cbpci_exp_drv); 
// if (ret < 0) { //error... };


int _cbpci_exp_map_pcimem(struct pci_dev *pdev, 
                      _u32 *baddr1, _u32 *baddr2,
                      _u8 **vio1, _u8 **vrst1, 
                      _u8 **vio2, _u8 **vrst2 )
{

    if (pci_enable_device(pdev)) {
        PWARN("can't enable pci device\n");
        return -1;
    }
    if (pdev->irq == 0) {
        PWARN("no irq allocation by PCI\n");
        return -1;
    }
    // first channel 
    *baddr1 = pci_resource_start(pdev, 2);
    if ((*vio1 =
        (_u8 *) ioremap(*baddr1, pci_resource_len(pdev, 2))) == NULL) {
            PERROR("can't map io addres 0x%lx\n", (unsigned long) *baddr1);
            return -1;
    }
    *vrst1 = *vio1 + 32;

    // second channel 
    *baddr2 = pci_resource_start(pdev, 3);
    if ((*vio2 =
        (_u8 *) ioremap(*baddr2 +128, pci_resource_len(pdev, 3) - 128)) == NULL) {
            iounmap(*vio1);
            PERROR("can't map io addres 0x%lx\n", (unsigned long) *baddr2);
            return -1;
    }
    *vrst2 = *vio2 + 32;

    return 0;
}

static int _cbpci_exp_register(struct unican *ucn, struct pci_dev *pdev)
{
    _u32 addr1, addr2;
    _u8 *vio1, *vrst1, *vio2, *vrst2, *lcr;
    struct can_dev *chip1, *chip2;
    struct can_board *brd;
    memiores_t *iores;

    if (_cbpci_exp_map_pcimem(pdev, &addr1, &addr2, 
                  &vio1, &vrst1, &vio2, &vrst2 )<0)
        return -1;

    if ( (iores = CANDEV_IORES_ALLOC(memiores_t)) == NULL )
        goto fail;
    iores->io = vio1;
    iores->ra = vrst1;
    chip1 = unican_dev_alloc(SJA1000, pdev->irq, addr1, 
            (void *) iores, iomem_hread, iomem_hwrite, iomem_hreset);

    if (chip1 == NULL) {
        sdep_free(iores);
        PWARN("can't register chip 1\n");
        goto fail;
    }
    
    if ( (iores = CANDEV_IORES_ALLOC(memiores_t)) == NULL ) {
        goto fail_chip2;
    }
    iores->io = vio2;
    iores->ra = vrst2;
    chip2 = unican_dev_alloc(SJA1000, pdev->irq, addr2, 
            (void *) iores, iomem_hread, iomem_hwrite, iomem_hreset);
    if (chip2 == NULL) {
        sdep_free(iores);
        PWARN("can't register chip 2\n");
        goto fail_chip2;
    }
    brd = unican_board_alloc_register (ucn, &cbpci_exp_drv, 
                   chip1, chip2, NULL, NULL, UNICAN_VER(2, 0, 0), NULL);
    if (brd == NULL) {
        PERROR("can't register board\n");
        goto fail_board;
    }

    chip1->cops->stall(chip1);
    chip2->cops->stall(chip2);

    if (request_irq (chip1->irqn, cbpci_interrupt,
        CAN_HW_ISR_FLAGS, "can", (void *) brd)) {
            PERROR("can't assign hardware irq handler\n");
    }
    
    // enable PLX interrupt
    // Write Interrupt Control/Status Register (LCS_INTCSR) in Local Configuration space
    addr1 = pci_resource_start(pdev, 0);
    if ((lcr =
        (_u8 *) ioremap(addr1, pci_resource_len(pdev, 0))) == NULL) {
            PERROR
                ("can't map Local Configuration Registers address 0x%lx\n",
                (unsigned long) addr1);
    } else {
        iowrite32((_u32) (0x01UL << 8) | (0x01UL << 11),
            (void *) (lcr + 0x68));
        iounmap(lcr);
    }

return 0;

fail_board:
    sdep_free(chip2->iores);
    sdep_free(chip2);
fail_chip2:
    sdep_free(chip1->iores);
    sdep_free(chip1);
fail:
    iounmap(vio1);
    iounmap(vio2);
    return -1;
}

int cbpci_exp_find_init_all(struct unican *ucn)
{
    struct pci_dev *pdev = NULL;

    PDEBUG ("cbpci_exp_find_init_all() is called\n");
    while ((pdev = sdep_pci_find_device(CAN_bus_PCI_EXP_VendorID,
                           CAN_bus_PCI_EXP_DeviceID, pdev)) != NULL) {
       if ( _cbpci_exp_register(ucn, pdev) < 0 ) {
           PERROR("CAN_bus_PCI_E can't alloc and register board\n");
       }
    }
    return 0;
}

//
// free hardware resources here
//

void cbpci_exp_remove(struct unican *ucn, struct can_board *brd)
{
    int i;
    memiores_t *iores;
    
    free_irq(brd->chip[0]->irqn, brd);
    for (i = 0; i < 2; i++) {
        if (brd->chip[i]) {
            brd->chip[i]->cops->stall(brd->chip[i]);
            iores = CANDEV_IORES_GET(brd->chip[i],memiores_t);
            iounmap(iores->io);
        }
    }
}

