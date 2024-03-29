#ifndef BITS_H
#define BITS_H

// COMMAND BYTES
#define LASER_ON_OFF        (1<<1)
#define LASER_SYNC          (1<<2)
#define LASER_DRYING_OFF    (1<<3)

// 0 Byte
#define BIT_MN1_ERROR       (1<<7)
#define BIT_MN1_READY       (1<<6)
#define BIT_MN1_ON          (1<<5)
#define BIT_MN2_ERROR       (1<<4)
#define BIT_MN2_READY       (1<<3)
#define BIT_MN2_ON          (1<<2)
#define BIT_AE_ERROR        (1<<1)
#define BIT_AE_READY        (1<<0)

// 1 Byte
#define BIT_GVG_ERROR       (1<<7)
#define BIT_GVG_READY       (1<<6)
#define BIT_MT_ERROR        (1<<5)
#define BIT_MT_READY        (1<<4)
#define BIT_SYNC            (1<<3)
#define BIT_EMITTING        (1<<2)
#define BIT_1               (1<<1)
#define BIT_2               (1<<0)


#endif // BITS_H
