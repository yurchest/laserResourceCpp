#ifndef __CHAI_CBUNP_H
#define __CHAI_CBUNP_H
#ifdef __cplusplus
extern "C" {
#endif

extern _s16 find_cbunp_devices(void);
extern _s16 cbunp_init(void);
extern struct chai_funcs cbunp_funcs;

extern _s16 cbunp_close(_u8 chan);
extern _s16 cbunp_stop(_u8 chan);
extern _s16 cbunp_chipstat(_u8 chan, chipstat_t * stat);
extern _s16 cbunp_getfirmwarever(_u8 chan, _u32 *ver);


#ifdef __cplusplus
}
#endif
#endif	/* __CHAI_CBUNP_H */
