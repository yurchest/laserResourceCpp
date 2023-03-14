/*
*  chai-test.c
*  Test for CHAI
*
*/

#include <stdio.h>
#include <stdarg.h>
#include <time.h>

#ifdef LINUX
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#endif                          // LINUX

#ifdef WIN32
#include <windows.h>
#endif                          // WIN32

#include <chai.h>

const char ver[] = "chaitest ver. 2.6, 23 May 2016";

#ifdef LINUX
#include <sys/time.h>
#define msleep(msec) usleep(msec*1000)
unsigned long long get_systime_in_msecs(void) 
{
    struct timeval tv;
    unsigned long long msecs = 0;

    gettimeofday(&tv, NULL);
    msecs = (unsigned long long) tv.tv_sec * 1000ULL;
    msecs += (unsigned long long) tv.tv_usec / 1000ULL;
    return msecs;
}

#endif

#ifdef WIN32
#define msleep(msec) Sleep(msec)

unsigned long long get_systime_in_msecs(void) 
{
    FILETIME fnow;
    unsigned long long msecs = 0;

    GetSystemTimeAsFileTime(&fnow);
    msecs = fnow.dwHighDateTime;
    msecs = msecs << 32;
    msecs |= (unsigned long long) fnow.dwLowDateTime;
    msecs = msecs / 10000ULL;
    return msecs;
}
#endif

_u8 ch1, ch2;

void time_measure(void (*func) (void), char *msg) 
{
    unsigned long long begint, finisht, difft;

    printf("%s: ", msg);
    fflush(stdout);
    begint = get_systime_in_msecs();

    func();

    finisht = get_systime_in_msecs();
    difft = finisht - begint;
    printf("(complete in %lu millisecs)\n", (unsigned long) difft);
    fflush(stdout);
} 

int tcall(int cierrno, void *fmt, ...) 
{
    va_list ap;
    char buf[4096];

    if (cierrno < 0) {
        CiStrError(cierrno, buf, 4096);
        printf("\n[FAULT] ");
        va_start(ap, fmt);
        vfprintf(stdout, (char *) fmt, ap);
        va_end(ap);
        printf(" : (errcode=%d) %s\n", cierrno, buf);
        printf("TEST FAIL !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n");
        printf("press [Enter] key to exit\n");
        fflush(stdin);
        fgetc(stdin);
        exit(1);
    }
    return cierrno;
} 

void print_chipstat(_u8 ch) 
{
    sja1000stat_t st;

    tcall(CiChipStat(ch, (chipstat_t *) & st),
        "can't get status for channel %d\n", ch);
    printf 
        ("channel %d: rx_errcnt=%#x, tx_errcnt=%#x, errcode=%#x, hw_ovrs=%ld, sw_ovrs=%ld\n",
        ch, st.rxec, st.txec, st.ecc, (long) st.hovr_cnt, (long) st.sovr_cnt);
} 

/************************************************************
*  BINFO TEST
*/ 

_u32 get_firmware_version(_u8 chan) 
{
    _u32 ver = 0;

    if (CiOpen(chan, 0) >= 0) {
        CiGetFirmwareVer(chan, &ver);
        CiClose(chan);
    }
    return ver;
}

void chai_testbinfo(void) 
{
    int i, j, ret;
    canboard_t binfo;
    _u32 fwver = 0;

    for (i = 0; i < CI_BRD_NUMS; i++) {
        binfo.brdnum = i;
        ret = CiBoardInfo(&binfo);
        if (ret < 0)
            continue;

        printf("%s [%s]: \n", binfo.name, binfo.manufact);
        for (j = 0; j < 4; j++) {
            if (binfo.chip[j] >= 0) {
                printf("   channel %d", binfo.chip[j]);
                if ((fwver =
                    get_firmware_version((_u8) binfo.chip[j])) > 0) {
                        printf(" (firmware %d.%d)", VERMIN(fwver),
                            VERSUB(fwver));
                }
                printf("\n");
            }
        }
    }
}


/************************************************************
*  STATUS TEST
*/ 

void status_test(_u8 ch) 
{
    chipstat_t stat;
    sja1000stat_t * sjst;
    int i;

    printf("testing chip status for channel %d ...\n", ch);
    tcall(CiOpen(ch, 0), "\nError opening CAN channel %d", ch);
    tcall(CiSetBaud(ch, BCI_500K), "can't set baud, channel %d", ch);
    tcall(CiStart(ch), "CiStart");

    tcall(CiChipStat(ch, &stat), "can't get status for channel %d\n",
        ch);
    if (stat.type == SJA1000) {
        for (i = 0; i <= 1000; i++) {
            tcall(CiChipStat(ch, &stat),
                "can't get status for channel %d\n", ch);
            sjst = (sja1000stat_t *) & stat;
            if (sjst->ewl == 0x60
                && sjst->btr0 == 0x0 
                &&sjst->btr1 == 0x1c
                && sjst->outctl == 0x5e) {
                    printf 
                        ("%d: ch %d: ewl=%#x, btr0=%#x, btr1=%#x, outctl=%#x OKEY\r",
                        i, ch, sjst->ewl, sjst->btr0, sjst->btr1,
                        sjst->outctl);
            } else {
                printf 
                    ("%d: ch %d: ewl=%#x, btr0=%#x, btr1=%#x, outctl=%#x FAILED\r",
                    i, ch, sjst->ewl, sjst->btr0, sjst->btr1,
                    sjst->outctl);
                break;
            }
        }
    }
    printf("\n\n");
    tcall(CiStop(ch), "CiStop");
    tcall(CiClose(ch), "CiClose");
    return;
}

/************************************************************
*  ECHO TEST
*/ 
int echotest_complete = 0;

void exit_failCB() 
{
    printf("TEST FAIL !!!!!!!!!!!!!!!!!!!\n");
    fflush(stdout);
    echotest_complete = 2;
} 

void exit_fail() 
{
    printf("TEST FAIL !!!!!!!!!!!!!!!!!!!\n\n");
    fflush(stdout);
    print_chipstat(ch1);
    print_chipstat(ch2);

    CiStop(ch1);
    CiStop(ch2);
    CiClose(ch1);
    CiClose(ch2);
    printf("press [Enter] key to exit\n");
    fflush(stdin);
    fgetc(stdin);
    exit(0);
} 

_u32 oldid = 0;
unsigned char olddata = 0;

void ch1_testbasic(_u8 chan1, short sig) 
{
    _s16 ret;
    canmsg_t frame;

    if (sig != CIEV_RC) {
        fprintf(stderr, "Bad event code on channel %d\n", chan1);
        exit_failCB();
    }

    if ((ret = CiRead(chan1, &frame, 1)) <= 0) {
        fprintf(stderr, "Error receiving back frame via channel %d\n",
            chan1);
        CiPerror(ret, "CiRead");
        exit_failCB();
    }
    printf("checking frame: id = %#lx data[0] = %#x\r", (unsigned long) frame.id,
        frame.data[0]);
    fflush(stdout);

    if (frame.id != oldid + 1 || frame.len != 1 || frame.flags != 0) {
        fprintf(stderr,
            "\ncan echo test fail, frame (sent id = %#lx, rcvd id=%#lx) was distorted\n",
            (unsigned long) oldid, (unsigned long) frame.id);
        exit_failCB();
    }
    if (frame.id == 2047) {
        echotest_complete = 1;
        return;
    }

    oldid = frame.id;
    olddata = frame.data[0];

    if ((ret = CiWrite(chan1, &frame, 1)) <= 0) {
        fprintf(stderr, "Error sending frame via channel %d\n", chan1);
        CiPerror(ret, "CiWrite");
        exit_failCB();
    }
}

void ch1_testbasic_cb(short sig) 
{
    ch1_testbasic(ch1, sig);
} 

void ch2_testbasic(_u8 chan2, short sig) 
{
    canmsg_t frame;
    int ret;

    if (sig != CIEV_RC) {
        fprintf(stderr, "Bad event code on channel %d\n", chan2);
        exit_failCB();
    }

    if ((ret = CiRead(chan2, &frame, 1)) <= 0) {
        fprintf(stderr, "Error receiving frame via channel %d\n", chan2);
        CiPerror(ret, "CiRead");
        exit_failCB();
    }
    //printf("ch2_testbasic:  id = 0x%lx, len=%d\n", (unsigned long) frame.id, frame.len);
    frame.data[0] = frame.data[0] + 1;
    frame.id = frame.id + 1;
    if ((ret = CiWrite(chan2, &frame, 1)) <= 0) {
        fprintf(stderr, "Error sending frame back via channel %d\n",
            chan2);
        CiPerror(ret, "CiWrite");
        exit_failCB();
    }
}


void ch2_testbasic_cb(short sig) 
{
    ch2_testbasic(ch2, sig);
} 

void err_testbasic(_u8 chan, short sig) 
{
    if (sig == CIEV_WTOUT) {
        printf("\nwrite timeout occured\n");
    } else if (sig == CIEV_EWL) {
        printf("\n\nerror warning level on channel %d\n", chan);
    } else if (sig == CIEV_BOFF) {
        printf("\n\nbus off on channel %d\n", chan);
    } else if (sig == CIEV_HOVR) {
        printf("\n\nhardware overrun on channel %d\n", chan);
    } else if (sig == CIEV_SOVR) {
        printf("\n\nsoftware overrun on channel %d\n", chan);
    } else {
        printf("\n\nunknown error event %d on channel %d\n", sig, chan);
    }
    fflush(stdout);
    exit_failCB();
}

void ch1err_testbasic_cb(short sig) 
{
    err_testbasic(ch1, sig);
} 

void ch2err_testbasic_cb(short sig) 
{
    err_testbasic(ch2, sig);
} 

void testbasic_cbext(_u8 chan, _s16 sig, void *arg) 
{
    unsigned long val = (unsigned long) arg;

    if (sig == CIEV_RC) {
        if (chan == ch1) {
            if (val != 0x1) {
                fprintf(stderr,
                    "Bad RC callback argument on channel1 %d\n",
                    chan);
                exit_failCB();
            }
            ch1_testbasic(chan, sig);
        } else {               //ch2
            if (val != 0x2) {
                fprintf(stderr,
                    "Bad RC callback argument on channel2 %d\n",
                    chan);
                exit_failCB();
            }
            ch2_testbasic(chan, sig);
        }
    } else {                   // expect CIEV_ERR
        if (val != 0x31 && val != 0x32) {
            fprintf(stderr, "Bad ERR callback argument on channel %d\n",
                chan);
            exit_failCB();
        }
        err_testbasic(chan, sig);
    }
}


void echo_open(_u8 bt0, _u8 bt1, _u8 flags, _u16 tout) 
{
    _u16 rtout;

    tcall(CiOpen(ch1, flags), "\nError opening CAN channel %d", ch1);
    tcall(CiOpen(ch2, flags), "\nError opening CAN channel %d", ch2);

    tcall(CiSetBaud(ch1, bt0, bt1),
        "can't set baud to bt0 = %#x, bt1 = %#x, channel %d", bt0,
        bt1, ch1);
    tcall(CiSetBaud(ch2, bt0, bt1),
        "can't set baud to bt0 = %#x, bt1 = %#x, channel %d", bt0, bt1,
        ch2);
    tcall(CiSetWriteTout(ch1, tout),
        "can't set write timeout %d msecs for channel %d", tout, ch1);

    tcall(CiGetWriteTout(ch1, &rtout),
        "can't get write timeout for channel %d", tout, ch1);

    if (rtout != tout) {
        fprintf(stderr, "Read tout %d don't equal to set value\n", rtout);
        exit_fail();
    }

    tcall(CiSetWriteTout(ch2, tout),
        "can't set write timeout %d msecs for channel %d", tout, ch2);

    tcall(CiGetWriteTout(ch2, &rtout),
        "can't get write timeout for channel %d", tout, ch2);
    if (rtout != tout) {
        fprintf(stderr, "Read tout %d don't equal to set value\n", rtout);
        exit_fail();
    }
}

enum { 
    TEST_CB = 0, 
    TEST_CBEXT 
};


int echo_test(_u8 bt0, _u8 bt1, _u16 tout, int ext_flag) 
{
    canmsg_t frame;

//    printf("echo test begin: ch1 = %d, ch2 = %d\n", ch1, ch2); fflush(stdout);
    echo_open(bt0, bt1, 0, tout);
    if (ext_flag) {
        tcall(CiSetCBex(ch1, CIEV_RC, testbasic_cbext, (void *) 0x1),
            "can't set extended CIEV_RC CB");
        tcall(CiSetCBex(ch2, CIEV_RC, testbasic_cbext, (void *) 0x2),
            "can't set extended CIEV_RC CB");

        tcall(CiSetCBex(ch1, CIEV_CANERR, testbasic_cbext, (void *) 0x31),
            "can't set extended CIEV_CANERR CB");

        tcall(CiSetCBex(ch2, CIEV_CANERR, testbasic_cbext, (void *) 0x32),
            "can't set extended CIEV_CANERR CB");
    } else {
        tcall(CiSetCB(ch1, CIEV_RC, ch1_testbasic_cb),
            "can't set CIEV_RC CB");

        tcall(CiSetCB(ch2, CIEV_RC, ch2_testbasic_cb),
            "can't set CIEV_RC CB");

        tcall(CiSetCB(ch1, CIEV_CANERR, ch1err_testbasic_cb),
            "can't set CIEV_CANERR CB");

        tcall(CiSetCB(ch2, CIEV_CANERR, ch2err_testbasic_cb),
            "can't set CIEV_CANERR CB");
    }
    tcall(CiStart(ch1), "CiStart failed");
    tcall(CiStart(ch2), "CiStart failed");

    frame.id = 0;
    frame.flags = 0;
    frame.len = 1;
    frame.data[0] = 0;

    oldid = 0;
    olddata = 0;

    tcall(CiWrite(ch1, &frame, 1),
        "Error sending frame via channel %d\n", ch1);

    while (echotest_complete <= 0) {
        msleep(1);
    }

    printf("\n");
    print_chipstat(ch1);
    print_chipstat(ch2);
    tcall(CiClose(ch1), "CiClose");
    tcall(CiClose(ch2), "CiClose");
//    printf("echo test fin: ch1 = %d, ch2 = %d\n", ch1, ch2); fflush(stdout);
    return 0;
}


void echo_test_10K_50_ext(void)
{
    echo_test(BCI_10K, 50, TEST_CBEXT);
}

void echo_test_250K_20_ext(void)
{
    echo_test(BCI_250K, 20, TEST_CBEXT);
} 

void echo_test_500K_20(void)
{
    echo_test(BCI_500K, 20, TEST_CB);
}

void echo_test_1000K_20(void)
{
    echo_test(BCI_1M, 20, TEST_CB);
}

void chai_testecho() 
{
    printf("ECHO TEST\n");

    echotest_complete = 0;
    time_measure(echo_test_250K_20_ext,
        "echo test 250K extcb (timeout 20 msec): \n");
    if (echotest_complete != 1)
        exit(0);
    else
        printf("OK\n");

    echotest_complete = 0;
    time_measure(echo_test_500K_20,
        "echo test 500K (timeout 20 msec): \n");
    if (echotest_complete != 1)
        exit(0);
    else
        printf("OK\n");


    echotest_complete = 0;
    time_measure(echo_test_1000K_20,
        "echo test 1M (timeout 20 msec): \n");
    if (echotest_complete != 1)
        exit(0);
    else
        printf("OK\n");

    echotest_complete = 0;
    time_measure(echo_test_10K_50_ext,
        "echo test 10K extcb (timeout 50 msec): \n");
    if (echotest_complete != 1)
        exit(0);
    else
        printf("OK\n");

    printf("TEST SUCCESS\n\n");
}


/************************************************************
*  PONG TEST (ECHO WITHOUT CALLBACKS)
*/ 
void check_canerrs(_u8 chan) 
{

    canerrs_t errs;

    printf("\nCAN-error occured on channel %d: ", chan);
    fflush(stdout);

    tcall(CiErrsGetClear(chan, &errs), "CiErrsGetClear() failed");
    if (errs.ewl) {
        printf("EWL %d times ", errs.ewl);
    } else if (errs.boff) {
        printf("BOFF %d times ", errs.boff);
    } else if (errs.hwovr) {
        printf("HOVR %d times ", errs.hwovr);
    } else if (errs.swovr) {
        printf("SOVR %d times ", errs.swovr);
    } else if (errs.wtout) {
        printf("WTOUT %d times ", errs.wtout);
    } else {
        printf("UNKNOWN CAN ERROR!!! \n");
    }
    printf("\n");
    exit_fail();
}


int pong_test(_u8 bt0, _u8 bt1, _u16 tout) 
{
    canmsg_t ping, rping, pong;
    canwait_t cw[2];
    int i, ret;

    echo_open(bt0, bt1, CIO_CAN11 | CIO_CAN29, tout);
    tcall(CiStart(ch1), "CiStart failed");
    tcall(CiStart(ch2), "CiStart failed");

    cw[0].chan = ch1;
    cw[1].chan = ch2;
    cw[0].wflags = cw[1].wflags = CI_WAIT_RC | CI_WAIT_ER;

    ping.id = 0;
    ping.flags = 0;
    ping.len = 1;
    ping.data[0] = 0;
    msg_seteff(&ping);

    printf("checking frame: id = %#lx data[0] = %#x\r", (unsigned long) ping.id,
        ping.data[0]);
    fflush(stdout);
    tcall(CiWrite(ch1, &ping, 1), "Error sending frame via channel %d\n",
        ch1);

    for (i = 1; i < 2047; i++) {
        ret = CiWaitEvent(cw, 2, 1000);
        if (ret < 0) {
            fprintf(stderr, "CiWaitEvent() failed, errcode = %d\n", ret);
            exit_fail();
        }
        if (ret == 0) {
            fprintf(stderr, "CiWaitEvent() timeout\n");
            exit_fail();
        }
        if (ret & 0x1) {       //ch1
            if (cw[0].rflags & CI_WAIT_ER) {
                check_canerrs(ch1);
            }
            if (cw[0].rflags & CI_WAIT_RC) {
                tcall(CiRead(ch1, &rping, 1),
                    "\nCiRead on channel %d failed", ch1);
                if (rping.id != ping.id + 1
                    || rping.len != ping.len 
                    ||rping.flags != ping.flags
                    || rping.data[0] != ping.data[0]) {
                        printf
                            ("\ncan echo test fail (frame was distorted): \n");
                        printf("     sent id=0x%lx, len=%d, data[0]=0x%x\n",
                            (unsigned long) ping.id, ping.len, ping.data[0]);
                        printf("      rcv id=0x%lx, len=%d, data[0]=0x%x\n",
                            (unsigned long) rping.id, rping.len, rping.data[0]);
                        exit_fail();
                }
                ping.id = i;
                ping.data[0]++;
                printf("checking frame: id = %#lx data[0] = %#x\r",
                    (unsigned long) ping.id, ping.data[0]);
                fflush(stdout);
                tcall(CiWrite(ch1, &ping, 1),
                    "\nError sending frame via channel %d\n", ch1);
            }
        }
        if (ret & 0x2) {       //ch2
            if (cw[1].rflags & CI_WAIT_ER) {
                check_canerrs(ch2);
            }
            if (cw[1].rflags & CI_WAIT_RC) {
                tcall(CiRead(ch2, &pong, 1),
                    "CiRead on channel %d failed", ch2);
                pong.id++;
                tcall(CiWrite(ch2, &pong, 1),
                    "\nError sending frame via channel %d\n", ch2);
            }
        }
    }
    print_chipstat(ch1);
    print_chipstat(ch2);
    tcall(CiClose(ch1), "CiClose");
    tcall(CiClose(ch2), "CiClose");
    return 0;
}



void pong_test_250K_20(void) 
{
    pong_test(BCI_250K, 20);
}

void pong_test_50K_50(void) 
{
    pong_test(BCI_50K, 50);
}


void chai_testpong(void) 
{
    printf("PONG TEST\n");
    time_measure(pong_test_250K_20,
        "pong test 250K (timeout 20 msec): \n");
    printf("OK\n");
    //msleep(500);
    time_measure(pong_test_50K_50,
        "pong test 50K (timeout 50 msec): \n");
    printf("OK\n");
    printf("TEST SUCCESS\n\n");
}

/************************************************************
*  RC QUEUE TEST
*/ 
#define FNUM 2048
int queue_test(_u8 bt0, _u8 bt1, _u16 tout) 
{
    canmsg_t send;
    int i, ret, cnt;

    send.id = 0x1;
    send.flags = 0;
    send.len = 1;
    send.data[0] = 1;

    echo_open(bt0, bt1, CIO_CAN11 | CIO_CAN29, tout);
    tcall(CiStart(ch1), "CiStart failed");
    tcall(CiStart(ch2), "CiStart failed");
    for (i = 0; i < CIQUE_DEFSIZE_RC; i++) {
        printf("send frame number %d\r", i + 1);
        fflush(stdout);
        ret = CiWrite(ch1, &send, 1);
        tcall(ret, "Error sending frame via channel %d\n", ch1);
        if (ret == 0) {
            tcall(CiWrite(ch1, &send, 1),
                "Error sending frame via channel %d\n", ch1);
        }
        msleep(1);
    }
    printf("\n");
    msleep(50);
    cnt = CiRcGetCnt(ch2);
    tcall(cnt, "CiRcGetCnt failed on channel %d", ch2);
    if (cnt != CIQUE_DEFSIZE_RC) {
        printf("rcv num not equal to sent, channel %d: sent=%d, rcv=%d\n",
            ch2, CIQUE_DEFSIZE_RC, cnt);
        exit_fail();
    }
    tcall(CiRcQueEmpty(ch2), "CiRcQueEmpty failed on channel %d", ch2);
    cnt = CiRcGetCnt(ch2);
    tcall(cnt, "CiRcGetCnt failed on channel %d", ch2);
    if (cnt != 0) {
        printf("rcv num not equal to zero, channel %d\n", ch2);
        exit_fail();
    }
    tcall(CiStop(ch2), "CiStop failed");
    ret = CiQueResize(ch2, FNUM);
    if (ret == -ECINOSYS) {
        printf
            ("there is no CiQueResize() in this type of CAN-board, skipping...\n");
    } else {
        tcall(ret, "CiQueResize failed on channel %d", ch2);
        tcall(CiStart(ch2), "CiStart failed");
        for (i = 0; i < FNUM; i++) {
            printf("send frame number %d\r", i + 1);
            fflush(stdout);
            ret = CiWrite(ch1, &send, 1);
            tcall(ret, "Error sending frame via channel %d\n", ch1);
            if (ret == 0) {
                tcall(CiWrite(ch1, &send, 1),
                    "Error sending frame via channel %d\n", ch1);
            }
            msleep(1);
        }
        msleep(50);
        cnt = CiRcGetCnt(ch2);
        tcall(cnt, "CiRcGetCnt failed, channel %d", ch2);
        if (cnt != FNUM) {
            printf
                ("rcv num not equal to sent, channel %d: sent=%d, rcv=%d\n",
                ch2, FNUM, cnt);
            exit_fail();
        }
    }
    printf("\n");
    print_chipstat(ch1);
    print_chipstat(ch2);
    tcall(CiClose(ch1), "CiClose");
    tcall(CiClose(ch2), "CiClose");
    return 0;
}


void queue_test_500K_50(void) 
{
    queue_test(BCI_500K, 50);
} 

void chai_testque(void) 
{
    printf("RC QUEUE TEST\n");
    time_measure(queue_test_500K_50,
        "queue test 500K (timeout 50 msec): \n");
    printf("OK\n");
    printf("TEST SUCCESS\n\n");
}

/************************************************************
*  OPEN FLAGS TEST
*/ 
void chai_test_openflags(void) 
{
    canmsg_t send, rcv;

    printf("OPEN FLAGS TEST\n");
    tcall(CiOpen(ch2, CIO_CAN11 | CIO_CAN29),
        "\nError opening CAN channel %d", ch2);
    tcall(CiSetBaud(ch2, BCI_500K), "can't set baud 500K for channel %d",
        ch2);
    tcall(CiStart(ch2), "CiStart failed");

    printf("testing CIO_CAN11 flag\n");
    tcall(CiOpen(ch1, CIO_CAN11), "\nError opening CAN channel %d", ch1);
    tcall(CiSetBaud(ch1, BCI_500K), "can't set baud 500K for channel %d",
        ch1);
    tcall(CiStart(ch1), "CiStart failed");

    msg_zero(&send);
    send.id = 0x1;
    send.len = 0;
    tcall(CiWrite(ch2, &send, 1), "Error sending frame via channel %d\n",
        ch2);
    msleep(500);
    msg_zero(&rcv);
    if (CiRead(ch1, &rcv, 1) != 1) {
        printf 
            ("Can't recieve SFF frame on channel opened with flag CIO_CAN11, channel %d\n",
            ch1);
        exit_fail();
    }
    msg_seteff(&send);
    tcall(CiWrite(ch2, &send, 1), "Error sending frame via channel %d\n",
        ch2);
    msleep(500);
    msg_zero(&rcv);
    if (CiRead(ch1, &rcv, 1) == 1) {
        printf
            ("Recieve EFF frame on channel opened with flag CIO_CAN11, channel %d\n",
            ch1);
        exit_fail();
    }
    tcall(CiClose(ch1), "CiClose");

    printf("testing CIO_CAN29 flag\n");
    tcall(CiOpen(ch1, CIO_CAN29), "\nError opening CAN channel %d", ch1);
    tcall(CiSetBaud(ch1, BCI_500K), "can't set baud 500K for channel %d",
        ch1);
    tcall(CiStart(ch1), "CiStart failed");
    msg_zero(&send);
    send.id = 0x1;
    send.len = 0;
    tcall(CiWrite(ch2, &send, 1), "Error sending frame via channel %d\n",
        ch2);
    msleep(500);
    msg_zero(&rcv);
    if (CiRead(ch1, &rcv, 1) == 1) {
        printf
            ("Recieve SFF frame on channel opened with flag CIO_CAN29, channel %d\n",
            ch1);
        exit_fail();
    }
    msg_seteff(&send);
    tcall(CiWrite(ch2, &send, 1), "Error sending frame via channel %d\n",
        ch2);
    msleep(500);
    msg_zero(&rcv);
    if (CiRead(ch1, &rcv, 1) != 1) {
        printf 
            ("Can't recieve EFF frame on channel opened with flag CIO_CAN29, channel %d\n",
            ch1);
        exit_fail();
    }
    tcall(CiClose(ch1), "CiClose");

    printf("testing CIO_CAN11 | CIO_CAN29 flag\n");
    tcall(CiOpen(ch1, CIO_CAN11 | CIO_CAN29),
        "\nError opening CAN channel %d", ch1);
    tcall(CiSetBaud(ch1, BCI_500K), "can't set baud 500K for channel %d",
        ch1);
    tcall(CiStart(ch1), "CiStart failed");
    msg_zero(&send);
    send.id = 0x1;
    send.len = 0;
    tcall(CiWrite(ch2, &send, 1), "Error sending frame via channel %d\n",
        ch2);
    msleep(500);
    msg_zero(&rcv);
    if (CiRead(ch1, &rcv, 1) != 1) {
        printf 
            ("Can't recieve SFF frame on channel opened with flag CIO_CAN11 | CIO_CAN29, channel %d\n",
            ch1);
        exit_fail();
    }
    msg_seteff(&send);
    tcall(CiWrite(ch2, &send, 1), "Error sending frame via channel %d\n",
        ch2);
    msleep(500);
    msg_zero(&rcv);
    if (CiRead(ch1, &rcv, 1) != 1) {
        printf 
            ("Can't recieve EFF frame on channel opened with flag CIO_CAN11 | CIO_CAN29, channel %d\n",
            ch1);
        exit_fail();
    }
    tcall(CiClose(ch1), "CiClose");
    tcall(CiClose(ch2), "CiClose");
    printf("OK\n");
    printf("TEST SUCCESS\n\n");
}


/************************************************************
*  ACCEPTANCE FILTER TEST
*/ 
void chai_test_acfilter(void) 
{
    canmsg_t send, rcv;

    printf("ACCEPTANCE FILTER TEST\n");

    tcall(CiOpen(ch2, CIO_CAN11 | CIO_CAN29),
        "\nError opening CAN channel %d", ch2);
    tcall(CiSetBaud(ch2, BCI_500K), "can't set baud 500K for channel %d",
        ch2);
    tcall(CiStart(ch2), "CiStart failed");

    printf("testing 11-bit identifier\n");
    tcall(CiOpen(ch1, CIO_CAN11), "\nError opening CAN channel %d", ch1);
    tcall(CiSetBaud(ch1, BCI_500K), "can't set baud 500K for channel %d",
        ch1);
    tcall(CiSetFilter(ch1, 0x32, 0xffff),
        "can't set acceptance filter for channel %d", 
        ch1);
    tcall(CiStart(ch1), "CiStart failed");
    msg_zero(&send);
    send.id = 0x32;
    send.len = 0;
    tcall(CiWrite(ch2, &send, 1), "Error sending frame via channel %d\n",
        ch2);
    msleep(500);
    msg_zero(&rcv);
    if (CiRead(ch1, &rcv, 1) != 1) {
        printf("Can't recieve frame id=0x32 on channel %d\n", ch1);
        exit_fail();
    }
    send.id = 0x33;
    tcall(CiWrite(ch2, &send, 1), "Error sending frame via channel %d\n",
        ch2);
    msleep(500);
    msg_zero(&rcv);
    if (CiRead(ch1, &rcv, 1) == 1) {
        printf("Recieve frame id=0x33 on channel %d\n", ch1);
        exit_fail();
    }
    send.id = 0x31;
    tcall(CiWrite(ch2, &send, 1), "Error sending frame via channel %d\n",
        ch2);
    msleep(500);
    msg_zero(&rcv);
    if (CiRead(ch1, &rcv, 1) == 1) {
        printf("Recieve frame id=0x31 on channel %d\n", ch1);
        exit_fail();
    }
    tcall(CiClose(ch1), "CiClose");

    printf("testing 29-bit identifier\n");
    tcall(CiOpen(ch1, CIO_CAN29), "\nError opening CAN channel %d", ch1);
    tcall(CiSetBaud(ch1, BCI_500K), "can't set baud 500K for channel %d",
        ch1);
    tcall(CiSetFilter(ch1, 0x932, 0xffffffff),
        "can't set acceptance filter for channel %d", ch1);
    tcall(CiStart(ch1), "CiStart failed");
    msg_zero(&send);
    msg_seteff(&send);
    send.id = 0x932;
    send.len = 0;
    tcall(CiWrite(ch2, &send, 1), "Error sending frame via channel %d\n",
        ch2);
    msleep(500);
    msg_zero(&rcv);
    if (CiRead(ch1, &rcv, 1) != 1) {
        printf("Can't recieve frame id=0x932 on channel %d\n", ch1);
        exit_fail();
    }
    send.id = 0x933;
    tcall(CiWrite(ch2, &send, 1), "Error sending frame via channel %d\n",
        ch2);
    msleep(500);
    msg_zero(&rcv);
    if (CiRead(ch1, &rcv, 1) == 1) {
        printf("Recieve frame id=0x933 on channel %d\n", ch1);
        exit_fail();
    }
    send.id = 0x931;
    tcall(CiWrite(ch2, &send, 1), "Error sending frame via channel %d\n",
        ch2);
    msleep(500);
    msg_zero(&rcv);
    if (CiRead(ch1, &rcv, 1) == 1) {
        printf("Recieve frame id=0x931 on channel %d\n", ch1);
        exit_fail();
    }

    tcall(CiClose(ch1), "CiClose");
    tcall(CiClose(ch2), "CiClose");
    printf("OK\n");
    printf("TEST SUCCESS\n\n");
}

/************************************************************
*
*  TRANSMIT TEST
*
*/ 

int check_is_cbunp(_u8 chan, _u32 * fwver) 
{
    int ret = 0;
    sja1000stat_t st;
    canboard_t bi;

    if (CiChipStat(chan, (chipstat_t *) & st) >= 0) {
        bi.brdnum = (_u8) st.brdnum;
        if (CiBoardInfo(&bi) >= 0) {
            if (strcmp(bi.name, "CAN-bus-USBnp") == 0) {
                CiGetFirmwareVer(chan, fwver);
                ret = 1;
            }
        }
    }
    return ret;
}


void chai_test_transmit(void) 
{
    canmsg_t send[16];         //, rcv[16];
    canmsg_t rcv;
    _u16 trqcnt = 0;
    int i, ret;
    canwait_t cw;
    _u32 fwver = 0;

    printf("TRANSMIT TEST\n");

    tcall(CiOpen(ch2, CIO_CAN11 | CIO_CAN29),
        "\nError opening CAN channel %d", ch2);
    tcall(CiSetBaud(ch2, BCI_500K), "can't set baud 500K for channel %d",
        ch2);
    tcall(CiStart(ch2), "CiStart failed");

    tcall(CiOpen(ch1, CIO_CAN11 | CIO_CAN29),
        "\nError opening CAN channel %d", ch1);
    tcall(CiSetBaud(ch1, BCI_500K), "can't set baud 500K for channel %d",
        ch1);
    tcall(CiStart(ch1), "CiStart failed");

    for (i = 0; i < 16; i++) {
        msg_zero(&send[i]);
        send[i].id = i;
        send[i].len = 1;
        send[i].data[0] = i;
    }

    printf("check CiTrStat with CiWrite: ");
    fflush(stdout);
    tcall(CiStop(ch1), "CiStop failed");
    tcall(CiStop(ch2), "CiStop failed");
    tcall(CiSetWriteTout(ch2, 0), "Set write timeout to zero failed\n",
        ch2);
    tcall(CiStart(ch2), "CiStart failed");
    tcall(CiWrite(ch2, &send[0], 1),
        "Error sending frame via channel %d\n", ch2);
    ret = CiTrStat(ch2, &trqcnt);
    if (ret != CI_TR_INCOMPLETE) {
        printf("Bad transmit status at channel %d\n", ch2);
        exit_fail();
    }
    if (trqcnt != 0) {
        printf
            ("Bad count of transmit queue channel %d, trqcnt = %u (should be zero)\n",
            ch2, trqcnt);
        exit_fail();
    }
    tcall(CiStart(ch1), "CiStart failed");
    msleep(400);
    tcall(CiRead(ch1, &rcv, 1), "CiRead failed");
    tcall(CiStop(ch1), "CiStop failed");
    printf("ok\n");

    // if one of the channel is CAN-bus-USBnp fw ver < 1.4 - complete
    if (check_is_cbunp(ch1, &fwver)) {
        if (fwver < CHAI_VER(0, 1, 4))
            goto out;
    }
    if (check_is_cbunp(ch2, &fwver)) {
        if (fwver < CHAI_VER(0, 1, 4))
            goto out;
    }

    printf("check CiTrStat, CiTrCancel with CiTransmit: ");
    fflush(stdout);

    for (i = 0; i < 16; i++) {
        tcall(CiTransmit(ch2, &send[i]),
            "Error sending frame via channel %d, iteration %d\n", ch2,
            i);
    }
    msleep(300);
    ret = CiTrStat(ch2, &trqcnt);
    if (ret != CI_TR_INCOMPLETE) {
        printf("Bad transmit status at channel %d\n", ch2);
        exit_fail();
    }
    if (trqcnt != 15) {
        printf
            ("Bad count of transmit queue channel %d, trqcnt = %u (should be 15)\n",
            ch2, trqcnt);
        exit_fail();
    }
    tcall(CiTrCancel(ch2, &trqcnt),
        "Can't cancel transmit at channel %d\n", ch2);
    ret = CiTrStat(ch2, &trqcnt);
    if (ret != CI_TR_COMPLETE_ABORT) {
        printf("Bad transmit status after cancel at channel %d\n", ch2);
        exit_fail();
    }
    if (trqcnt != 0) {
        printf
            ("Bad count of transmit queue after cancel at channel %d, trqcnt = %u (should be zero)\n",
            ch2, trqcnt);
        exit_fail();
    }
    printf("ok\n");

    printf
        ("check CiTrStat and CiWaitEvent for CI_WAIT_TR with CiTransmit : ");
    fflush(stdout);
    tcall(CiStop(ch1), "CiStop failed");
    tcall(CiStart(ch2), "CiStart failed");
    cw.chan = ch2;
    cw.wflags = CI_WAIT_TR;
    for (i = 0; i < 16; i++)
        tcall(CiTransmit(ch2, &send[i]),
        "Error sending frame via channel %d\n", ch2);
    ret = CiWaitEvent(&cw, 1, 1000);
    if (ret != 0) {
        printf
            ("Bad return from CiWaitEvent(): %d, should be zero (timeout)\n",
            ret);
        exit_fail();
    }
    tcall(CiStart(ch1), "CiStart failed");
    ret = CiWaitEvent(&cw, 1, 1000);
    if (ret <= 0) {
        printf
            ("Bad return from CiWaitEvent(): %d, should be greater then zero\n",
            ret);
        exit_fail();
    }
    if (cw.rflags != CI_WAIT_TR) {
        printf("Bad rflags field from CiWaitEvent() %d, should be %d \n",
            cw.rflags, CI_WAIT_TR);
        exit_fail();
    }
    ret = CiTrStat(ch2, &trqcnt);
    if (ret != CI_TR_COMPLETE_OK) {
        printf("Bad transmit status at channel %d\n", ch2);
        exit_fail();
    }
    if (trqcnt != 0) {
        printf
            ("Bad count of transmit queue after start channel %d, trqcnt = %u (should be zero)\n",
            ch2, trqcnt);
        exit_fail();
    }
    printf("ok\n");
    fflush(stdout);

    printf ("check CiWrite return code for wrong transmit treshold: ");
    fflush(stdout);
	trqcnt = 1;
    tcall(CiStop(ch2), "CiStop failed");
    tcall(CiTrQueThreshold(ch2, CI_CMD_SET, &trqcnt), "CiTrQueThreshold failed");
    tcall(CiStart(ch2), "CiStart failed");
    ret = CiWrite(ch2, &send[0], 1);
	if (ret != -ECIINVAL) {
        printf("Bad CiWrite ret code at channel %d, ret = %d, should be -ECIINVAL (%d)\n", ch2, ret, -ECIINVAL);
        exit_fail();
	}
    printf("ok\n");
    fflush(stdout);

out:
    tcall(CiClose(ch1), "CiClose");
    tcall(CiClose(ch2), "CiClose");
    printf("OK\n");
    printf("TEST SUCCESS\n\n");
}


/************************************************************
*  Transmit speed test
*/ 

void chai_measure_transmit(canmsg_t * send, int send_size) 
{
    int i, ret;
    canwait_t cw;
    unsigned long long begint, finisht, difft;

    cw.chan = ch1;
    cw.wflags = CI_WAIT_TR | CI_WAIT_ER;

    printf("CiTransmit: transmitting %d EFF frames ...\n", send_size);
    fflush(stdout);
    begint = get_systime_in_msecs();
    for (i = 0; i < send_size; i++) {
try_again:
        ret = CiTransmit(ch1, &send[i]);
        if (ret == -ECINORES) {
            //fulltimes++;
            //printf("tr queue is full %d times\r", fulltimes);fflush(stdout);
            if ((ret = CiWaitEvent(&cw, 1, 1000)) > 0) {
                if (cw.rflags & CI_WAIT_ER) {
                    check_canerrs(ch1);
                }
            } else if (ret < 0) {
                tcall(ret, "CiWaitEvent");
            }
            goto try_again;
        } else if (ret < 0) {
            tcall(ret, "CiTransmit");
        }
    }
    // wait transmit finish
    if ((ret = CiWaitEvent(&cw, 1, 1000)) > 0) {
        if (cw.rflags & CI_WAIT_ER) {
            check_canerrs(ch1);
        }
    } else if (ret < 0) {
        tcall(ret, "CiWaitEvent");
    }
    finisht = get_systime_in_msecs();
    difft = finisht - begint;
    printf("okey                               \n");
    printf("(complete in %lu msecs)\n", (unsigned long) difft);
} 

void chai_measure_write(canmsg_t * send, int send_size) 
{
    int i;
    unsigned long long begint, finisht, difft;

    printf("CiWrite: transmitting %d EFF frames ...\n", send_size);
    fflush(stdout);
    begint = get_systime_in_msecs();
    for (i = 0; i < send_size; i++) {
        tcall(CiWrite(ch1, &send[i], 1), "CiWrite");
    }
    finisht = get_systime_in_msecs();
    difft = finisht - begint;
    printf("okey                               \n");
    printf("(complete in %lu msecs)\n", (unsigned long) difft);
} 

canmsg_t sbuf[CIQUE_DEFSIZE_RC];

void chai_test_measure_transmit(_u8 bt0, _u8 bt1) 
{
    _u32 fwver = 0;
    int i;

    printf("TRANSMIT MEASURE TEST\n");
    for (i = 0; i < CIQUE_DEFSIZE_RC; i++) {
        msg_zero(&sbuf[i]);
        msg_seteff(&sbuf[i]);
        sbuf[i].id = i;
        sbuf[i].len = 1;
        sbuf[i].data[0] = i;
    }

    tcall(CiOpen(ch1, CIO_CAN11 | CIO_CAN29),
        "\nError opening CAN channel %d", ch1);
    tcall(CiSetBaud(ch1, bt0, bt1), "can't set baud  for channel %d",
        ch1);
    tcall(CiStart(ch1), "CiStart failed");

    tcall(CiOpen(ch2, CIO_CAN11 | CIO_CAN29),
        "\nError opening CAN channel %d", ch2);
    tcall(CiSetBaud(ch2, bt0, bt1), "can't set baud for channel %d", ch2);
    tcall(CiStart(ch2), "CiStart failed");

    // if one of the channel is CAN-bus-USBnp fw ver < 1.4 - complete
    if (check_is_cbunp(ch1, &fwver)) {
        if (fwver < CHAI_VER(0, 1, 4)) {
            printf
                ("channel %d is CAN-bus-USBnp firmware version less then 1.4\n",
                ch1);
            printf
                ("This test is not applicable for CAN-bus-USBnp firmware less then 1.4 ... skipped\n");
            goto out;
        }
    }
    if (check_is_cbunp(ch2, &fwver)) {
        if (fwver < CHAI_VER(0, 1, 4)) {
            printf
                ("channel %d is CAN-bus-USBnp firmware version less then 1.4\n",
                ch1);
            printf
                ("This test is not applicable for CAN-bus-USBnp firmware less then 1.4 ... skipped\n");
            goto out;
        }
    }

    printf("Measure Transmit with default transmit threshold: %u\n",
        (_u16) CIQUE_TR_THRESHOLD_DEF);
    chai_measure_transmit(sbuf, CIQUE_DEFSIZE_RC);
    tcall(CiClose(ch1), "CiClose");
    tcall(CiClose(ch2), "CiClose");
    tcall(CiOpen(ch1, CIO_CAN11 | CIO_CAN29),
        "\nError opening CAN channel %d", ch1);
    tcall(CiSetBaud(ch1, bt0, bt1), "can't set baud for channel %d", ch1);
    tcall(CiStart(ch1), "CiStart failed");
    tcall(CiOpen(ch2, CIO_CAN11 | CIO_CAN29),
        "\nError opening CAN channel %d", ch2);
    tcall(CiSetBaud(ch2, bt0, bt1), "can't set baud for channel %d", ch2);
    tcall(CiStart(ch2), "CiStart failed");
    printf
        ("Measure Transmit with default transmit threshold divided by 2: %u\n",
        (_u16) CIQUE_TR_THRESHOLD_DEF / 2);
    chai_measure_transmit(sbuf, CIQUE_DEFSIZE_RC);
    tcall(CiClose(ch1), "CiClose");
    tcall(CiClose(ch2), "CiClose");
    tcall(CiOpen(ch1, CIO_CAN11 | CIO_CAN29),
        "\nError opening CAN channel %d", ch1);
    tcall(CiSetBaud(ch1, bt0, bt1), "can't set baud for channel %d", ch1);
    tcall(CiStart(ch1), "CiStart failed");
    tcall(CiOpen(ch2, CIO_CAN11 | CIO_CAN29),
        "\nError opening CAN channel %d", ch2);
    tcall(CiSetBaud(ch2, bt0, bt1), "can't set baud for channel %d", ch2);
    tcall(CiStart(ch2), "CiStart failed");

    chai_measure_write(sbuf, CIQUE_DEFSIZE_RC);

    printf("TEST SUCCESS\n\n");
out:
    tcall(CiClose(ch1), "CiClose");
    tcall(CiClose(ch2), "CiClose");
    return;
}

void chai_test_delay_transmit(_u8 bt0, _u8 bt1, _u32 delay_mks) 
{

#define TDELAY_SIZE 10
    _u32 fwver = 0;
    canmsg_t send[TDELAY_SIZE];
    canmsg_t rcv[TDELAY_SIZE];
    int i, ret;
    _u16 rcq_thres;
    canwait_t cw;
    _u16 rcqcnt;

    for (i = 0; i < TDELAY_SIZE; i++) {
        msg_zero(&send[i]);
        msg_seteff(&send[i]);
        msg_setdelaytr(&send[i], delay_mks);
        send[i].id = i;
        send[i].len = 1;
        send[i].data[0] = i;
    }

    tcall(CiOpen(ch1, CIO_CAN11 | CIO_CAN29),
        "\nError opening CAN channel %d", ch1);
    tcall(CiSetBaud(ch1, bt0, bt1), "can't set baud  for channel %d",
        ch1);
    tcall(CiOpen(ch2, CIO_CAN11 | CIO_CAN29),
        "\nError opening CAN channel %d", ch2);
    tcall(CiSetBaud(ch2, bt0, bt1), "can't set baud for channel %d", ch2);

    // if one of the channel is CAN-bus-USBnp fw ver < 1.4 - complete
    if (check_is_cbunp(ch1, &fwver)) {
        if (fwver < CHAI_VER(0, 1, 4)) {
            printf ("channel %d is CAN-bus-USBnp firmware version less then 1.4\n",
                ch1);
            printf 
                ("This test is not applicable for CAN-bus-USBnp firmware less then 1.4 ... skipped\n");
            goto out;
        }
    }

    if (check_is_cbunp(ch2, &fwver)) {
        if (fwver < CHAI_VER(0, 1, 4)) {
            printf
                ("channel %d is CAN-bus-USBnp firmware version less then 1.4\n",
                ch1);
            printf
                ("This test is not applicable for CAN-bus-USBnp firmware less then 1.4 ... skipped\n");
            goto out;
        }
    }
    rcq_thres = TDELAY_SIZE;
    tcall(CiRcQueThreshold(ch2, CI_CMD_SET, &rcq_thres),
        "CiRcQueThreshold");
    tcall(CiStart(ch1), "CiStart failed");
    tcall(CiStart(ch2), "CiStart failed");

    printf("CiTransmit: transmitting %d EFF frames ...\n", TDELAY_SIZE);
    fflush(stdout);
    for (i = 0; i < TDELAY_SIZE; i++) {
        tcall(CiTransmit(ch1, &send[i]), "CiTransmit");
    }
    // wait transmit finish
    cw.chan = ch2;
    cw.wflags = CI_WAIT_RC | CI_WAIT_ER;
    cw.rflags = 0;
    if ((ret = CiWaitEvent(&cw, 1, 20000)) > 0) {
        if (cw.rflags & CI_WAIT_ER) {
            check_canerrs(ch1);
        }
    } else if (ret < 0) {
        tcall(ret, "CiWaitEvent");
    } else {                   //0 timeout
        tcall(CiRcQueGetCnt(ch2, &rcqcnt), "CiRcQueGetCnt failed");
        printf
            ("CiWaitEvent() timeout, there is %d frames in rcv queue (should be %d)\n",
            (int) rcqcnt, (int) TDELAY_SIZE);
        exit_fail();
    } 
    ret = CiRead(ch2, rcv, TDELAY_SIZE);
    if (ret < 0)
        tcall(ret, "CiRead");
    if (ret != TDELAY_SIZE) {
        printf("Bad return from CiRead(): %d, should be %d\n", ret,
            TDELAY_SIZE);
        exit_fail();
    }
    printf("transmit delays in mks: ");
    fflush(stdout);
    for (i = 1; i < TDELAY_SIZE; i++) {
        printf("%7lu ", (unsigned long) send[i].ts);
    }
    printf("\n");
    fflush(stdout);
    printf("recieved delays in mks: ");
    fflush(stdout);
    for (i = 1; i < TDELAY_SIZE; i++) {
        printf("%7lu ", (unsigned long) (rcv[i].ts - rcv[i - 1].ts));
    }
    printf("\n");
    fflush(stdout);
    printf("TEST SUCCESS\n\n");
out:
    tcall(CiClose(ch1), "CiClose");
    tcall(CiClose(ch2), "CiClose");
    return;
}


/************************************************************
*  MAIN
*/ 

void test_exit(void) 
{
    CiStop(ch1);
    CiStop(ch2);
    CiClose(ch1);
    CiClose(ch2);
    printf("good bye\n");
} 

void quit(int status, void *pid) 
{
    test_exit();
} 
#ifdef WIN32
BOOL CtrlHandler(DWORD fdwCtrlType) 
{
    switch (fdwCtrlType) {
        // Handle the CTRL+C signal. 
    case CTRL_C_EVENT:
    case CTRL_CLOSE_EVENT:
        test_exit();
        // Pass other signals to the next handler. 
    case CTRL_BREAK_EVENT:
    case CTRL_LOGOFF_EVENT:
    case CTRL_SHUTDOWN_EVENT:
    default:
        return FALSE;
    }
}
#endif

int main(int argc, char **argv) 
{
    unsigned long chver;
    int ret;
    int c1, c2;

    printf("\nCHAI TestTool\n");
    printf("%s\n", ver);
    if ((ret = CiInit()) < 0) {
        printf("CHAI library initialization failed\n");
        exit(1);
    }

    chver = CiGetLibVer();
    printf("using CHAI %d.%d.%d\n\n", VERMAJ(chver), VERMIN(chver),
        VERSUB(chver));

    printf("\nDetected CAN adapters:\n");
    chai_testbinfo();
    printf("\n");

    //printf("sizeof(canmsg_t) = %d\n", sizeof(canmsg_t));
    //printf("sizeof(sja1000stat_t) = %d\n", sizeof(sja1000stat_t));

    printf
        ("Test application require two CAN channels connected with CAN cable\n");
    printf("enter first channel to use: ");
    scanf("%d", (int *) &c1);
    printf("enter second channel to use: ");
    scanf("%d", (int *) &c2);
    ch1 = (_u8) c1;
    ch2 = (_u8) c2;

#ifdef WIN32
    SetConsoleCtrlHandler((PHANDLER_ROUTINE) CtrlHandler, TRUE);
    fflush(stdin);
#endif
    printf("\nUsing CAN channels %d and %d\n", ch1, ch2);
    printf("Connect these channels by CAN cable, then press [Enter]\n\n");
    fgetc(stdin);

    status_test(ch1);
    status_test(ch2);



    chai_testpong();

    chai_testecho();
    chai_test_transmit();

    chai_test_measure_transmit(BCI_500K);




    //chai_test_measure_transmit(BCI_50K);
    /*
    printf("DELAY TRANSMIT TEST, BCI_500K, delay 1000 mks\n");
    chai_test_delay_transmit(BCI_500K, 1000);
    printf("DELAY TRANSMIT TEST, BCI_500K, delay 10000 mks\n");
    chai_test_delay_transmit(BCI_500K, 10000);
    printf("DELAY TRANSMIT TEST, BCI_500K, delay 20000 mks\n");
    chai_test_delay_transmit(BCI_500K, 20000);
    printf("DELAY TRANSMIT TEST, BCI_500K, delay 100000 mks\n");
    chai_test_delay_transmit(BCI_500K, 100000);
    //printf("DELAY TRANSMIT TEST, BCI_500K, delay 1000000 mks\n");
    //chai_test_delay_transmit(BCI_500K, 1000000);
    */


    chai_test_acfilter();

    chai_test_openflags();


    printf("press [Enter] key to exit\n");
    fflush(stdin);
    fgetc(stdin);
    exit(0);
}
