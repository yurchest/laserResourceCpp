/*
*  canmon.c
*  CAN Monitor for CHAI
*
*  Author: Fedor Nedeoglo
*
*/

#include <stdio.h>
#include <stdarg.h>
#include <time.h>
#include <chai.h>

#ifdef LINUX
#include <unistd.h>
#include <dirent.h>             // for PATH_MAX macro
#include <stdlib.h>
#include <string.h>
#ifdef READLINE_SPRT
#include <readline/readline.h>
#include <readline/history.h>
#endif
#endif

#ifdef WIN32
#include <windows.h>
#include <process.h>
#endif

const char cm_version[] = "CAN monitor for CHAI, ver. 3.1, 13 Jun 2013";

#define PROMPT  "canmon> "
#define STR_LEN_MAX 1024
#define CMD_LEN_MAX 128
#define CMD_NUM_MAX 20

struct cm_cmd {
    char name[CMD_LEN_MAX];
    int (*exec) (char *args);
    char *arguments;
    char *help;
};

enum {
    CM_FRAME_TX = 0,
    CM_FRAME_RX = 1
};

struct _cm_cnt {
    unsigned long rx;
    unsigned long tx;
    unsigned long ewl;
    unsigned long boff;
    unsigned long hwovr;
    unsigned long swovr;
    unsigned long wtout;
};

/////////////////////////////////////////////// 
//  globals
//
struct cm_cmd *commands[CMD_NUM_MAX] = { NULL };

_u8 chan;
int baud;
_u8 bt0, bt1;
FILE * flog = NULL;
struct _cm_cnt cmcnt = { 0 };
int transmit_tout = 30 * CIQUE_DEFSIZE_TR;     // in millisecs
int cm_show_flag = 1;

/* 
*  forward declarations
*/ 
void cm_input_monitor(void);
void register_all_commands(void);

void cm_onexit(void);

///////////////////////////////////////////////////
//  Linux depend functions
//
#ifdef LINUX
#define CM_PATH_MAX PATH_MAX

#include <pthread.h>
typedef struct {
    int exit;
    pthread_mutex_t csec;
    pthread_t id;
} cm_thread_t;

#define CMTHR_RET_T void *
#define CMTHR_FUNC_DECL

int cm_thr_create(cm_thread_t * thr, CMTHR_RET_T (*start_routine) (void *))
{
    int ret = 0;
    thr->exit = 0;
    pthread_mutex_init(&thr->csec, NULL);
    ret = pthread_create(&thr->id, NULL, start_routine, (void *) thr);
    return ret;
}

int cm_thr_terminate(cm_thread_t * thr)
{
    int ret = 0;
    thr->exit = 1;
    ret = pthread_join(thr->id, NULL);
    pthread_mutex_destroy(&thr->csec);
    return ret;
}

#define cm_crit_enter(pthr) pthread_mutex_lock( &((pthr)->csec) )
#define cm_crit_leave(pthr) pthread_mutex_unlock( &((pthr)->csec) )

void __quit(int status, void *pid) 
{
    cm_onexit();
}

void cm_set_exit_func(void)
{
    on_exit(__quit, NULL);
} 
#endif                          // LINUX
///////////////////////////////////////////////////
//  Windows depend functions
//

#ifdef WIN32
#define CM_PATH_MAX MAX_PATH
#define snprintf _snprintf
#define inline _inline

typedef struct {
    int exit;
    CRITICAL_SECTION csec;
    HANDLE id;
} cm_thread_t;

#define CMTHR_RET_T unsigned
#define CMTHR_FUNC_DECL __stdcall

int cm_thr_create(cm_thread_t * thr, unsigned (__stdcall *start_routine) (void *))
{
    thr->exit = 0;
    InitializeCriticalSection(&thr->csec);
    thr->id = (HANDLE) _beginthreadex(NULL, 0, start_routine, (void *) thr, 0, NULL);
    if (!thr->id) {
        return -1;
    }
    return 0;
}

int cm_thr_terminate(cm_thread_t * thr)
{
    thr->exit = 1;
    WaitForSingleObject(thr->id, INFINITE);
    CloseHandle(thr->id);
    DeleteCriticalSection(&thr->csec);
    return 0;
}

#define cm_crit_enter(pthr) EnterCriticalSection( &((pthr)->csec) )
#define cm_crit_leave(pthr) LeaveCriticalSection( &((pthr)->csec) )


BOOL CtrlHandler(DWORD fdwCtrlType) 
{
    switch (fdwCtrlType) {
        // Handle the CTRL+C signal. 
    case CTRL_C_EVENT:
    case CTRL_CLOSE_EVENT:
        cm_onexit();
        // Pass other signals to the next handler. 
    case CTRL_BREAK_EVENT:
    case CTRL_LOGOFF_EVENT:
    case CTRL_SHUTDOWN_EVENT:
    default:
        return FALSE;
    }
}

void cm_set_exit_func(void)
{
    HANDLE hproc = NULL;
    SetConsoleCtrlHandler((PHANDLER_ROUTINE) CtrlHandler, TRUE);
    atexit(cm_onexit);
    hproc =
        OpenProcess(STANDARD_RIGHTS_REQUIRED | PROCESS_SET_INFORMATION 
        |PROCESS_QUERY_INFORMATION, TRUE, _getpid());
    if (hproc) {
        SetPriorityClass(hproc, HIGH_PRIORITY_CLASS);
        CloseHandle(hproc);
    }
}

#endif                          // WIN32

/////////////////////////////////////////////////////
//  Main
//

char logfile[CM_PATH_MAX];

cm_thread_t th;

void cm_onexit(void) 
{
    cm_thr_terminate(&th);
    CiStop(chan);
    CiClose(chan);
    fprintf(stdout, "good bye\n");
    fflush(stdout);
    if (flog) {
        fflush(flog);
        fclose(flog);
    }
}


#define cmprint(stream,...) \
    do { cm_crit_enter(&th); fprintf(stream,__VA_ARGS__); fflush(stream); cm_crit_leave(&th); } while(0)

#define cmlog(...) \
    do { cm_crit_enter(&th); \
    if (flog) { fprintf(flog,__VA_ARGS__); fflush(flog); };\
    cm_crit_leave(&th); } while(0)

#define cmprint_and_log(stream,...) \
    do { cm_crit_enter(&th); \
    fprintf(stream,__VA_ARGS__); fflush(stream); \
    if (flog) { fprintf(flog,__VA_ARGS__); fflush(flog); };\
    cm_crit_leave(&th); } while(0)

const char cm_header[] =
    "\nEV NUM/ERR F/F ID     LEN R/D D0 D1 D2 D3 D4 D5 D6 D7 TIMESTAMP\n";

const char swovr_fmt[] =
    "ER SOVR    SOFTWARE OVERRUN    OCCURED %3d TIMES                 \n";
const char ewl_fmt[] =
    "ER EWL     ERROR WARNING LIMIT OCCURED %3d TIMES                 \n";
const char boff_fmt[] =
    "ER BOFF    BUS OFF             OCCURED %3d TIMES                 \n";
const char wtout_fmt[] =
    "ER WTOUT   WRITE TIMEOUT       OCCURED %3d TIMES                 \n";
const char hwovr_fmt[] =
    "ER HOVR    HARDWARE OVERRUN    OCCURED %3d TIMES                 \n";

void cm_process_errs(canerrs_t * errs) 
{
    char bufout[512];
    if (errs->ewl) {
        snprintf(bufout, 512, ewl_fmt, errs->ewl);
        cm_crit_enter(&th);
        cmcnt.ewl += errs->ewl;
        cm_crit_leave(&th);
    }
    if (errs->boff) {
        snprintf(bufout, 512, boff_fmt, errs->boff);
        cm_crit_enter(&th);
        cmcnt.boff += errs->boff;
        cm_crit_leave(&th);
    }
    if (errs->hwovr) {
        snprintf(bufout, 512, hwovr_fmt, errs->hwovr);
        cm_crit_enter(&th);
        cmcnt.hwovr += errs->hwovr;
        cm_crit_leave(&th);
    }
    if (errs->swovr) {
        snprintf(bufout, 512, swovr_fmt, errs->swovr);
        cm_crit_enter(&th);
        cmcnt.swovr += errs->swovr;
        cm_crit_leave(&th);
    }
    if (errs->wtout) {
        snprintf(bufout, 512, wtout_fmt, errs->wtout);
        cm_crit_enter(&th);
        cmcnt.wtout += errs->wtout;
        cm_crit_leave(&th);
    }
    cmprint_and_log(stdout, "%s", bufout);
}

static void frame_str_for_print(int ftype, canmsg_t * f, char *bufout, int buflen) 
{
    int n;
    char buf[512];
    int k;
    char *cptr;
    unsigned long *pfcnt;


    cptr = buf;

    if (ftype == CM_FRAME_RX) {
        pfcnt = &cmcnt.rx;
    } else {
        pfcnt = &cmcnt.tx;
    }

    cm_crit_enter(&th);
    *pfcnt += 1;
    if (*pfcnt == 10000000)
        *pfcnt = 0;
    if ( ((cmcnt.rx + cmcnt.tx) % 10) == 0  && cm_show_flag) {
        printf("%s", cm_header);
        fflush(stdout);
    }
    cm_crit_leave(&th);

    if (buflen > 0 && cm_show_flag == 0) {
        bufout[0] = '\0';
        return;
    }

    if (ftype == CM_FRAME_RX)
        n = sprintf(cptr, "RX ");
    else
        n = sprintf(cptr, "TX ");
    cptr += n;
    n = sprintf(cptr, "%07lu ", *pfcnt);
    cptr += n;
    if (msg_iseff(f))
        n = sprintf(cptr, "EFF ");
    else
        n = sprintf(cptr, "SFF ");
    cptr += n;
    n = sprintf(cptr, "%08lX %-1d ", (unsigned long) f->id, f->len);
    cptr += n;
    if (msg_isrtr(f)) {
        n = sprintf(cptr, "RTR                         ");
        cptr += n;
    } else {
        n = sprintf(cptr, "DAT ");
        cptr += n;
        for (k = 0; k < f->len; k++) {
            n = sprintf(cptr, "%02X ", f->data[k]);
            cptr += n;
        }
        for (k = 7; k >= f->len; k--) {
            n = sprintf(cptr, "   ");
            cptr += n;
        }
    }
    if (ftype == CM_FRAME_RX) {
        n = sprintf(cptr, "%010lu", (unsigned long) f->ts);
        cptr += n;
        n = sprintf(cptr, "\n");
        cptr += n;
    } else {
        n = sprintf(cptr, "          \n");
        cptr += n;
    }
    strncpy(bufout, buf, buflen);
}

#define CM_RCVBUF_SIZE 64


CMTHR_RET_T CMTHR_FUNC_DECL rcv_err_func(void *arg) 
{
    cm_thread_t *t = (cm_thread_t *) arg;
    canmsg_t rx[CM_RCVBUF_SIZE];
    canerrs_t errs;
    canwait_t cw;
    int ret = 0, ret1 = 0;
    int i;
    char cbuf[512];

    cw.chan = chan;
    cw.wflags = CI_WAIT_RC | CI_WAIT_ER;
    while (t->exit == 0) {
        ret = CiWaitEvent(&cw, 1, 200);
        if (ret < 0) {
            cmprint_and_log(stdout, "CiWaitEvent() failed, ret = %d\n",
                ret);
        } else if (ret > 0) {
            if (cw.rflags & CI_WAIT_RC) {
                ret1 = (int) CiRead(chan, rx, CM_RCVBUF_SIZE);
                if (ret1 > 0) {
                    for (i = 0; i < ret1; i++) {
                        frame_str_for_print(CM_FRAME_RX, &rx[i], cbuf,
                            512);
                        cmprint_and_log(stdout, "%s", cbuf);
                    }
                } else if (ret1 < 0) {
                    cmprint_and_log(stdout, "CiRead failed, ret = %d\n",
                        ret1);
                }
            }
            if (cw.rflags & CI_WAIT_ER) {
                ret1 = CiErrsGetClear(chan, &errs);
                if (ret1 >= 0) {
                    cm_process_errs(&errs);
                } else {
                    cmprint_and_log(stdout,
                        "CiErrsGetClear failed, ret = %d\n",
                        ret1);
                }
            }
        } /*else { // ret == 0 - timeout
          printf("CiWaitEvent timeout\n");
          } */
    }
    return (CMTHR_RET_T) 0;
}

_u32 get_firmware_version(canboard_t * binfo) 
{
    _s16 i;
    _u32 ver = 0;

    for (i = 0; i < 4; i++) {
        if (binfo->chip[i] >= 0) {
            if (CiOpen((_u8) binfo->chip[i], 0) >= 0) {
                CiGetFirmwareVer((_u8) binfo->chip[i], &ver);
                CiClose((_u8) binfo->chip[i]);
                break;
            }
        }
    }
    return ver;
}

int binfo(void) 
{
    _s16 i, j, ret;
    canboard_t binfo;
    int cnt = 0;
    _u32 fwver = 0;

    for (i = 0; i < CI_BRD_NUMS; i++) {
        binfo.brdnum = (_u8) i;
        ret = CiBoardInfo(&binfo);
        if (ret < 0)
            continue;
        printf("%-13s [%s", binfo.name, binfo.manufact);
        if ((fwver = get_firmware_version(&binfo)) > 0) {
            printf(" firmware %d.%d", VERMIN(fwver), VERSUB(fwver));
        }
        printf("]\n");

        for (j = 0; j < 4; j++) {
            if (binfo.chip[j] >= 0) {
                printf("   channel %d\n", binfo.chip[j]);
                cnt++;
            }
        }
    }
    return cnt;
}

void get_chan(void) 
{
    int ch;

    printf("CAN adapters:\n");
    if (binfo() > 0) {
        printf("\nCAN channel to use => ");
        scanf(" %i", &ch);
        chan = (_u8) ch;
    } else {
        printf 
            ("\nThere are no CAN channels.\nCheck CHAI installation.\n\n");
        printf("press [Enter] key to quit...");
        fgetc(stdin);
        exit(1);
    }
}

void get_bitrate(void) 
{
    int ask_flag;
    int tmp;

    do {
        ask_flag = 0;
        printf("Bit rate in Kbit/s => ");
        scanf(" %i", &baud);

        switch (baud) {
        case 1000:
            bt0 = BCI_1M_bt0;
            bt1 = BCI_1M_bt1;
            break;
        case 800:
            bt0 = BCI_800K_bt0;
            bt1 = BCI_800K_bt1;
            break;
        case 500:
            bt0 = BCI_500K_bt0;
            bt1 = BCI_500K_bt1;
            break;
        case 250:
            bt0 = BCI_250K_bt0;
            bt1 = BCI_250K_bt1;
            break;
        case 125:
            bt0 = BCI_125K_bt0;
            bt1 = BCI_125K_bt1;
            break;
        case 100:
            bt0 = BCI_100K_bt0;
            bt1 = BCI_100K_bt1;
            break;
        case 50:
            bt0 = BCI_50K_bt0;
            bt1 = BCI_50K_bt1;
            break;
        case 20:
            bt0 = BCI_20K_bt0;
            bt1 = BCI_20K_bt1;
            break;
        case 10:
            bt0 = BCI_10K_bt0;
            bt1 = BCI_10K_bt1;
            break;
        case 0:
            printf("Manual bit rate\n");
            printf
                ("value for bit timing register BTR0 (for example 0x00)=> ");
            scanf(" %i", &tmp);
            bt0 = (_u8) tmp;
            printf("\n");
            printf
                ("value for bit timing register BTR1 (for example 0x1c)=> ");
            scanf(" %i", &tmp);
            bt1 = (_u8) tmp;
            break;
        default:
            printf 
                ("Unsupported bit rate, valid only:\n 1000, 800, 500, 250, 125, 100, 50, 20, 10, 0 (for manual bit timing)\n");
            ask_flag = 1;
            break;
        }
    } while (ask_flag);
}


void init_can_channel(void) 
{
    _s16 ret;

    if ((ret = CiOpen(chan, CIO_CAN11 | CIO_CAN29)) < 0) {
        fprintf(stdout, "Error opening CAN channel %d, ret = %d\n", chan,
            ret);
        exit(1);
    }

    if ((ret = CiSetBaud(chan, bt0, bt1)) < 0) {
        fprintf(stdout, "can't set baud, ret = %d\n", ret);
        exit(1);
    }

    if ((ret = CiSetFilter(chan, 0xffff, 0x0)) < 0) {
        fprintf(stdout, "can't set hardware filter, ret = %d\n", ret);
        exit(1);
    }
}


int main(int argc, char **argv) 
{
    _s16 ret;
    unsigned long chver;

    printf("\n%s\n", cm_version);
    if ((ret = CiInit()) < 0) {
        printf("CHAI library initialization failed\n");
        exit(1);
    }

    chver = CiGetLibVer();
    printf 
        ("using CHAI %d.%d.%d\n\n", VERMAJ(chver), VERMIN(chver),
        VERSUB(chver));

    get_chan();
    get_bitrate();
    if (baud == 0)
        printf 
        ("\nusing CAN channel %d at manual bit rate: bt0=0x%x, bt1=0x%x\n",
        chan, bt0, bt1);
    else
        printf("\nusing CAN channel %d at %d Kbit/s\n", chan, baud);


    register_all_commands();
    fprintf(stdout,
        "type 'help' for commands info and 'quit' for exit.\n\n");
    printf("%s", cm_header);
    init_can_channel();
    if (cm_thr_create(&th, rcv_err_func) < 0) {
        printf("FATAL: can't start rcv_err thread, terminated\n");
        exit(1);
    }
    cm_set_exit_func();
    CiStart(chan);

    cm_input_monitor();
    return 0;
}


//////////////////////////////////////////////////////
//
// input monitor functions
//

const char *cm_delim = " \f\n\r\t\v";

/*
* rl_gets() Read a string, and return a pointer to it.  
* Returns NULL on EOF.
*/ 
#ifdef READLINE_SPRT
static char *line_read = (char *) NULL; /* for readline */

char *rl_gets(void) 
{
    /* If the buffer has already been allocated, return the 
    memory to the free pool. */ 
    if (line_read) {
        free(line_read);
        line_read = (char *) NULL;
    }
    /* Get a line from the user. */ 
    line_read = readline(PROMPT);
    /* If the line has any text in it, save it on the history. */ 
    if (line_read && *line_read)
        add_history(line_read);
    return (line_read);
}

#else
static char line_read[STR_LEN_MAX];
char *rl_gets(void) 
{
    char *ret;

    fprintf(stderr, "%s", PROMPT);
    ret = fgets(line_read, STR_LEN_MAX, stdin);
    if (!ret)
        return NULL;

    return line_read;
}
#endif

int execute_command(const char *command_name, char *args) 
{
    int slot;

    for (slot = 0; slot < CMD_NUM_MAX; slot++) {
        if (commands[slot]) {
            if (strcmp(commands[slot]->name, command_name) != 0)
                continue;
            commands[slot]->exec(args);
            return 0;
        }
    }
    return -1;
}

#define _is_delim(c,delim) ( (long) strchr(delim, c) )

 int get_token_num(const char *str, const char *delim) 
{

    int tn = 0;
    int curch_flag;
    int prevch_flag = 1;             // is delimiter
    const char *s = str;

    if (!str)
        return 0;

    while (*s) {
        curch_flag = _is_delim(*s, delim);
        if (!curch_flag && prevch_flag)
            tn++;
        prevch_flag = curch_flag;
        s++;
    }

    //fprintf(stdout, "get_token_num: tn = %d\n", tn);
    return tn;

}


#define CM_SYMBOL 0
#define CM_DELIM 1
// returns: NULL - end of str, pointer to first not to miss character
 char *_miss_delims_or_symbols(char *str, const char *delim,
    int type)
{
    char *s = str;
    int cond;

    while (*s) {
        cond = _is_delim(*s, delim);
        if ((type == CM_DELIM) && (!cond))
            return s;           // found symbol while missing delims
        if ((type == CM_SYMBOL) && cond)
            return s;           // found delim while missing symbols
        s++;
    }
    return NULL;
}

 void _str_trim(char *str, const char *delim)
{
    char *s = str;
    int i, len = 0;

    while (*s++)
        len++;
    for (i = len; i > 0; i--) {
        if (_is_delim(*s, delim))
            *s = '\0';
        else
            break;
        s--;
    }
}

// separate and trim cmd and args from str
void get_cmd_and_args(char *str, char **cmd, char **args) 
{
    char *start;
    char *finish;
    char *cmd_delim = NULL;

    *cmd = *args = NULL;
    start = _miss_delims_or_symbols(str, cm_delim, CM_DELIM);
    if (start) {
        *cmd = start;
        finish = _miss_delims_or_symbols(start, cm_delim, CM_SYMBOL);
        if (finish) {           // have delim after cmd
            cmd_delim = finish;
            start = _miss_delims_or_symbols(finish, cm_delim, CM_DELIM);
            if (start) {
                _str_trim(start, cm_delim);
                *args = start;
            }
        }
    }
    if (cmd_delim)
        *cmd_delim = '\0';
}

int interpret_str(char *str) 
{
    char *cmd = NULL;
    char *args = NULL;

    get_cmd_and_args(str, &cmd, &args);
    if (!cmd)
        return -1;
    //fprintf(stdout, "%s: arg is:%s, len = %d\n", cmd, args, strlen(cmd));
    if (execute_command(cmd, args) < 0)
        cmprint(stdout, "unknown command\n");
    return 0;
}


void cm_input_monitor(void)
{
    char *ln;

    while ((ln = rl_gets()))
        interpret_str(ln);
    return;
}


int register_command(const char *name, 
    int (*execute_func) (char *args),
    char *arguments, char *help)
{
    int slot;

    for (slot = 0; slot < CMD_NUM_MAX; slot++) {
        if (!commands[slot])
            break;
    }
    if (slot == CMD_NUM_MAX)
        return -1;             // no free command slots
    commands[slot] = (struct cm_cmd *) malloc(sizeof(struct cm_cmd));
    if (!commands[slot])
        return -1;             // memory fault
    strncpy(commands[slot]->name, name, CMD_LEN_MAX);
    commands[slot]->exec = execute_func;
    commands[slot]->help = help;
    commands[slot]->arguments = arguments;
    return 0;
}

int command_usage(const char *command_name) 
{
    int slot;

    for (slot = 0; slot < CMD_NUM_MAX; slot++) {
        if (commands[slot]) {
            if (strcmp(commands[slot]->name, command_name) == 0) {
                cmprint(stdout, "usage: %s", command_name);
                if (commands[slot]->arguments != NULL)
                    cmprint(stdout, " %s", commands[slot]->arguments);
                cmprint(stdout, "\n");
                return 0;
            }
        }
    }
    return -1;
}


/////////////////////////////////////////////
//  CANmon commands
//
_s16 cm_call(_s16 chai_ret, char *err_msg)
{
    if (chai_ret < 0) {
        cmprint_and_log(stdout, "%s, ret = %d\n", err_msg, chai_ret);
    }
    return chai_ret;
}


// quit
int cm_quit(char *args) 
{
    if (args) {
        command_usage("quit");
        return -1;
    }
    exit(0);
}
// help
int help_command(const char *command_name) 
{
    int slot;

    for (slot = 0; slot < CMD_NUM_MAX; slot++) {
        if (commands[slot]) {
            if (strcmp(commands[slot]->name, command_name) == 0) {
                cmprint(stdout, "%s - ", commands[slot]->name);
                if (commands[slot]->help != NULL)
                    cmprint(stdout, "%s", commands[slot]->help);
                cmprint(stdout, "\n\n");
                cmprint(stdout, "usage: %s", commands[slot]->name);
                if (commands[slot]->arguments != NULL)
                    cmprint(stdout, " %s", commands[slot]->arguments);
                cmprint(stdout, "\n\n");
                return 0;
            }
        }
    }
    return -1;
}


int cm_help(char *args) 
{
    int i;
    int args_num;

    args_num = get_token_num(args, cm_delim);

    if (args_num == 0) {
        cmprint(stdout, "\nAvailable commands:\n");
        for (i = 0; i < CMD_NUM_MAX; i++) {
            if (!commands[i])
                continue;
            cmprint(stdout, "   %-15s - ", commands[i]->name);
            if (commands[i]->help != NULL)
                cmprint(stdout, "%s", commands[i]->help);
            cmprint(stdout, "\n");
        }
        cmprint(stdout, "\n");
    } else if (args_num == 1) {
        cmprint(stdout, "\n");
        if (help_command(args) < 0) {
            cmprint(stdout, "help error: unknown command %s\n", args);
            return -1;
        }
        return 0;
    } else {
        command_usage("help");
        return -1;
    }
    return 0;
}

// start
int cm_start(char *args) 
{
    _s16 ret;

    if (args) {
        command_usage("start");
        return -1;
    }

    ret = cm_call(CiStart(chan), "can't start CAN-controller, CiStart");
    if (ret < 0)
        return -1;
    return 0;
}

// stop
int cm_stop(char *args) 
{
    _s16 ret;

    if (args) {
        command_usage("stop");
        return -1;
    }
    ret = cm_call(CiStop(chan), "can't stop CAN-controller, CiStop");
    if (ret < 0)
        return -1;
    return 0;
}

//reset
int cm_reset(char *args) 
{
    int ret, args_num;
    _u16 trqcnt = 0;
    char *tok;

    args_num = get_token_num(args, cm_delim);

    if (args_num != 1) {
        goto err;
    }

    tok = strtok(args, cm_delim);
    if (strcmp(tok, "cnt") == 0) {
        cm_crit_enter(&th);
        memset(&cmcnt, 0, sizeof(struct _cm_cnt));
        cm_crit_leave(&th);
        cmprint(stdout, "recieve, transmit and error counters was reset to 0\n");
    } else if (strcmp(tok, "hw") == 0) {
        ret =
            cm_call(CiHwReset(chan),
            "can't reset CAN-controller, CiHwReset");
        if (ret < 0)
            return -1;
        cmprint(stdout, "CAN-controller was reset\n");
    } else if (strcmp(tok, "tr") == 0) {
        ret = cm_call(CiTrCancel(chan, &trqcnt), "CiTrCancel failed");
        if (ret < 0)
            return -1;
        switch (ret) {
        case CI_TRCANCEL_TRANSMITTED:
            cmprint(stdout,
                "last transmition was completed (frame was sent)\n");
            break;
        case CI_TRCANCEL_ABORTED:
            cmprint(stdout, "last transmition was aborted\n");
            break;
        case CI_TRCANCEL_NOTRANSMISSION:
            cmprint(stdout, "no transmition in progress\n");
            break;
        case CI_TRCANCEL_DELAYABORTED:
            cmprint(stdout, "delay transmition was aborted\n");
            break;
        default:
            break;
        }
        cmprint(stdout, "%u frames was erased in trq\n", trqcnt);
    } else {
        goto err;
    }
    return 0;

err:
    command_usage("reset");
    return -1;
}

//stat
int cm_stat(char *args) 
{
    int i;
    chstat_desc_t desc;
    sja1000stat_t st;
    _u32 tmp32 = 0;
    _u16 tmp16;
    _s16 ret = 0;

    if (args) {
        command_usage("stat");
        return -1;
    }

    cmprint(stdout, "channel     : %d\n", chan);
    ret =
        cm_call(CiGetFirmwareVer(chan, &tmp32),
        "firmware    : CiGetFirmwareVer");
    if (ret >= 0) {
        cmprint(stdout, "firmware    : %d.%d\n", VERMIN(tmp32),
            VERSUB(tmp32));
    }

    cmprint(stdout, "tr    tout  : %d msec\n", transmit_tout);
    ret =
        cm_call(CiWriteTout(chan, CI_CMD_GET, &tmp16),
        "write tout  : CiWriteTout");
    if (ret >= 0) {
        cmprint(stdout, "write tout  : %u msec\n", tmp16);
    }

    cmprint(stdout, "rc count    : %lu\n", cmcnt.rx);
    cmprint(stdout, "tr count    : %lu\n", cmcnt.tx);
    cmprint(stdout, "ewl count   : %lu\n", cmcnt.ewl);
    cmprint(stdout, "boff count  : %lu\n", cmcnt.boff);
    cmprint(stdout, "hwovr count : %lu\n", cmcnt.hwovr);
    cmprint(stdout, "swovr count : %lu\n", cmcnt.swovr);
    cmprint(stdout, "wtout count : %lu\n", cmcnt.wtout);

    ret =
        cm_call(CiRcQueGetCnt(chan, &tmp16),
        "rcq count   : CiRcQueGetCnt");
    if (ret >= 0) {
        cmprint(stdout, "rcq count   : %u\n", tmp16);
    }
    ret = cm_call(CiTrStat(chan, &tmp16), "trq count   : CiTrStat");
    if (ret >= 0) {
        cmprint(stdout, "trq count   : %u\n", tmp16);
        cmprint(stdout, "tr state    : ");
        if (ret == CI_TR_COMPLETE_OK) {
            cmprint(stdout, "CI_TR_COMPLETE_OK\n");
        } else
            if (ret == CI_TR_COMPLETE_ABORT) {
                cmprint(stdout, "CI_TR_COMPLETE_ABORT\n");
            } else
                if (ret == CI_TR_INCOMPLETE) {
                    cmprint(stdout, "CI_TR_INCOMPLETE\n");
                } else
                    if (ret == CI_TR_DELAY) {
                        cmprint(stdout, "CI_TR_DELAY\n");
                    }
    }
    ret =
        cm_call(CiRcQueThreshold(chan, CI_CMD_GET, &tmp16),
        "rcq treshold: CiRcQueThreshold");
    if (ret >= 0) {
        cmprint(stdout, "rcq treshold: %u\n", tmp16);
    }
    ret =
        cm_call(CiTrQueThreshold(chan, CI_CMD_GET, &tmp16),
        "trq treshold: CiTrQueThreshold");
    if (ret >= 0) {
        cmprint(stdout, "trq treshold: %u\n", tmp16);
    }

    ret = cm_call(CiChipStat(chan, (chipstat_t *) &st), "error       : CiChipStat");

    if (ret >= 0) {
       CiChipStatToStr( (chipstat_t *) &st, &desc);
        i = 0;
        while (desc.name[i][0] != '\0') {
            if(strcmp(desc.name[i], "hardware ovr") != 0 &&
                strcmp(desc.name[i], "software ovr") != 0 )  {
                    cmprint(stdout, "%-12s: %s\n", desc.name[i], desc.val[i]);
            }
            i++;
        }
    }
    return 0;
}


// wr - write
int get_txframe_from_str(char *args, canmsg_t * tx, int *repeat) 
{
    int args_num, ret;
    char *tok, *rtrstr;
    int i;
    int tmp[8];

    args_num = get_token_num(args, cm_delim);

    if ((args_num != 2) && (args_num != 4)) {
        return -1;
    }

    *repeat = 1;
    msg_zero(tx);
    tok = strtok(args, cm_delim);
    if ((rtrstr = strstr(tok, "rtr")) != NULL) {        // RTR frame
        if (strlen(rtrstr) < 4)
            return -1;
        ret = sscanf(tok, "%i:rtr%i ", (int *) &tx->id, &tmp[0]);
        if (ret != 2)
            return -1;
        tx->len = (_u8) tmp[0];
        if (tx->len > 8)
            return -1;
        msg_setrtr(tx);
    } else {                    // DATA frame
        ret =
            sscanf(tok, "%i:%i,%i,%i,%i,%i,%i,%i,%i", 
            (int *) &tx->id,
            &tmp[0], &tmp[1], &tmp[2], &tmp[3], &tmp[4], 
            &tmp[5],
            &tmp[6], &tmp[7]);
        if (ret > 9 || ret < 1)
            return -1;
        tx->len = (_u8) (ret - 1);
        for (i = 0; i < tx->len; i++)
            tx->data[i] = (_u8) tmp[i];
    }
    tok = strtok(NULL, cm_delim);
    if (strcmp(tok, "eff") == 0) {     //EFF
        msg_seteff(tx);
    } else if (strcmp(tok, "sff") != 0) {
        return -1;
    }

    if (args_num == 4) {
        tok = strtok(NULL, cm_delim);
        if (strcmp(tok, "repeat") != 0) {
            return -1;
        }
        tok = strtok(NULL, cm_delim);
        ret = sscanf(tok, "%d", repeat);
        if (ret != 1)
            return -1;
    }
    return 0;
}


int cm_wr(char *args) 
{
    int ret;
    canmsg_t tx;
    int repeat, i;
    char cbuf[512];

    ret = get_txframe_from_str(args, &tx, &repeat);
    if (ret < 0)
        goto err;

    for (i = 1; i <= repeat; i++) {
        ret = CiWrite(chan, &tx, 1);
        if (ret > 0) {
            frame_str_for_print(CM_FRAME_TX, &tx, cbuf, 512);
            cmprint_and_log(stdout, "%s", cbuf);
        } else {
            cmprint_and_log(stdout, "frame %d can't write, ret = %d\n", i,
                ret);
        }
    }
    return 0;
err:
    command_usage("wr");
    return -1;
}

// tr - transmit
//returns: < 0 - error; 0 - timeout; > 0 success (tr complete)
int cm_transmit_wait_complete(_u8 chan, _u16 trqcnt_begin) 
{
    int ret, ret1;
    _u16 trqcnt = 0;
    int diff;
    canwait_t cw;

    cw.chan = chan;
    cw.wflags = CI_WAIT_TR;

    ret =
        cm_call(CiWaitEvent(&cw, 1, transmit_tout),
        "CiWaitEvent failed while transmitting");
    if (ret < 0)
        cmprint(stdout, "transmit aborted\n");
    else if (ret == 0)
        cmprint(stdout, "transmit aborted, timeout\n");
    if (ret <= 0) {
        ret1 =
            cm_call(CiTrStat(chan, &trqcnt),
            "CiTrStat failed while transmitting");
        if (ret1 >= 0) {
            diff = (int) trqcnt - (int) trqcnt_begin;
            if (diff < 0)
                diff = trqcnt;
            if (diff > 0) {
                cmprint(stdout, "last %d or %d CAN frames was not sent\n",
                    diff, diff + 1);
                cmprint(stdout, "look at tr state in status cmd output\n");
            }
        }
    }
    return ret;
}


int cm_tr(char *args) 
{
    int ret, ret1;
    canmsg_t tx;
    int repeat, i;
    char cbuf[512];
    _u16 trqcnt_begin;

    ret = get_txframe_from_str(args, &tx, &repeat);
    if (ret < 0)
        goto err;

    ret = cm_call(CiTrStat(chan, &trqcnt_begin), "CiTrStat failed");
    if (ret < 0)
        return -1;
    for (i = 1; i <= repeat; i++) {
try_again:
        ret = CiTransmit(chan, &tx);
        if (ret == 0) {
            frame_str_for_print(CM_FRAME_TX, &tx, cbuf, 512);
            cmprint_and_log(stdout, "%s", cbuf);
        } else if (ret == -ECINORES) {
            ret1 = cm_transmit_wait_complete(chan, trqcnt_begin);
            if (ret1 <= 0)      // timeout or error
                return -1;
            goto try_again;
        } else {
            cmprint_and_log(stdout, "frame %d can't transmit, ret = %d\n",
                i, ret);
        }
    }
    // wait transmit complete
    ret1 = cm_transmit_wait_complete(chan, trqcnt_begin);
    if (ret1 <= 0)             // timeout or error
        return -1;
    return 0;
err:
    command_usage("tr");
    return -1;
}


// filter
int cm_filter(char *args) 
{
    int args_num, ret;
    char *tok;
    unsigned long acode, amask;

    args_num = get_token_num(args, cm_delim);
    if (args_num != 1 && args_num != 3) {
        goto err;
    }

    tok = strtok(args, cm_delim);

    if (args_num == 1) { // possible off
        if (strcmp(tok, "off") != 0)
            goto err;
        acode = 0x0UL;
        amask = 0x0UL; // don't matter
    } else {
        if (strcmp(tok, "on") != 0)
            goto err;
        tok = strtok(NULL, cm_delim);

        ret = sscanf(tok, "%lx", &acode);
        if (ret < 1)
            goto err;

        tok = strtok(NULL, cm_delim);
        ret = sscanf(tok, "%lx", &amask);
        if (ret < 1)
            goto err;
    }
    cmprint(stdout, "setting filter acode=0x%lx, amask=0x%lx\n", acode,
        amask);
    ret =
        cm_call(CiSetFilter(chan, acode, amask),
        "can't set hardware filter, CiSetFilter");
    if (ret < 0)
        return -1;
    return 0;
err:
    command_usage("filter");
    return -1;
}

// lom
int cm_lom(char *args) 
{
    int args_num;
    char *tok;
    _s16 ret;
    _u8 cmd;

    args_num = get_token_num(args, cm_delim);
    if (args_num != 1) {
        goto err;
    }

    tok = strtok(args, cm_delim);
    if (strcmp(tok, "off") == 0) {
        cmd = CI_LOM_OFF;
    } else if (strcmp(tok, "on") == 0) {
        cmd = CI_LOM_ON;
    } else {
        goto err;
    }
    ret =
        cm_call(CiSetLom(chan, cmd),
        "can't switch controller Listen Only Mode, CiSetLom");
    if (ret < 0)
        return -1;
    cmprint(stdout, "Listen Only Mode is switched ");
    if (cmd == CI_LOM_OFF)
        cmprint(stdout, "on\n");
    else
        cmprint(stdout, "off\n");
    return 0;
err:
    command_usage("lom");
    return -1;
}

int cm_show(char *args) 
{
    int args_num;
    char *tok;

    args_num = get_token_num(args, cm_delim);
    if (args_num != 1) {
        goto err;
    }

    tok = strtok(args, cm_delim);
    if (strcmp(tok, "off") == 0) {
        cm_show_flag = 0;
        cmprint(stdout, "frames output is off\n");
    } else if (strcmp(tok, "on") == 0) {
        cm_show_flag = 1;
        cmprint(stdout, "frames output is on\n");
    } else {
        goto err;
    }
    return 0;
err:
    command_usage("show");
    return -1;
}

// log
int cm_log(char *args) 
{
    int args_num;
    char *tok;

    args_num = get_token_num(args, cm_delim);
    if (args_num != 1 && args_num != 2) {
        goto err;
    }
    tok = strtok(args, cm_delim);

    if (args_num == 1) {       //possible off
        if (strcmp(tok, "off") != 0)
            goto err;
        if (flog == NULL) {
            cmprint(stdout, "logging is switched off already\n");
            return -1;
        }
        cm_crit_enter(&th);
        fclose(flog);
        flog = NULL;
        cm_crit_leave(&th);
        cmprint(stdout, "logging to %s file is switched off\n", logfile);
    } else {                    // on
        if (strcmp(tok, "on") != 0)
            goto err;
        if (flog) {
            cmprint(stdout,
                "logging is switched off already, logfile is \"%s\"\n",
                logfile);
            return -1;
        }
        tok = strtok(NULL, cm_delim);
        strcpy(logfile, tok);
        cm_crit_enter(&th);
        flog = fopen(logfile, "a");
        cm_crit_leave(&th);

        if (flog == NULL) {
            cmprint(stdout, "can't open logfile %s\n", logfile);
            return -1;
        }
        cmprint(stdout, "logging to \"%s\" is switched on\n", logfile);
    }

    return 0;
err:
    command_usage("log");
    return -1;
}

// tout
int cm_tout(char *args) 
{
    int args_num, ret, wrflag;
    char *tok;
    unsigned long tmp;

    args_num = get_token_num(args, cm_delim);
    if (args_num != 2) {
        goto err;
    }

    tok = strtok(args, cm_delim);
    if (strcmp(tok, "wr") == 0) {
        wrflag = 0;
    } else if (strcmp(tok, "tr") == 0) {
        wrflag = 1;
    } else {
        goto err;
    }
    tok = strtok(NULL, cm_delim);
    ret = sscanf(tok, "%lu", &tmp);
    if (ret < 1)
        goto err;
    if (wrflag) {
        ret =
            cm_call(CiSetWriteTout(chan, (_u16) tmp),
            "can't set write timeout, CiSetWriteTout");
        if (ret < 0)
            return -1;
        cmprint(stdout, "write timeout was set to %lu milliseconds\n",
            tmp);
    } else {
        transmit_tout = tmp;
        cmprint(stdout, "transmit timeout was set to %d milliseconds\n",
            transmit_tout);
    }
    return 0;
err:
    command_usage("tout");
    return -1;
}


// tresh
int cm_tresh(char *args) 
{
    int args_num, ret;
    char *tok;
    int tmp, is_rcq;
    char *s_func, *s;

    args_num = get_token_num(args, cm_delim);
    if (args_num != 2)
        goto err;

    tok = strtok(args, cm_delim);
    if (strcmp(tok, "rcq") == 0) {
        is_rcq = 1;
    } else if (strcmp(tok, "trq") == 0) {
        is_rcq = 0;
    } else {
        goto err;
    }

    tok = strtok(NULL, cm_delim);
    ret = sscanf(tok, "%i", &tmp);

    if (ret < 1) {
        goto err;
    }
    if (is_rcq) {
        ret = CiRcQueThreshold(chan, CI_CMD_SET, (_u16 *) & tmp);
        s_func = "CiRcQueThreshold";
        s = "rcq";
    } else {
        ret = CiTrQueThreshold(chan, CI_CMD_SET, (_u16 *) & tmp);
        s_func = "CiTrQueThreshold";
        s = "trq";
    }
    if (ret < 0) {
        cmprint(stdout, "can't set %s treshold\n", s);
        cmlog("%s failed, ret = %d\n", s_func, ret);
        return -1;
    }
    cmprint(stdout, "%s treshold was set to %u\n", s, (_u16) tmp);
    return 0;
err:
    command_usage("tresh");
    return -1;
}


//rreg
int cm_rreg(char *args) 
{
    int args_num, ret;
    char *tok;
    int tmp;
    _u32 val;

    args_num = get_token_num(args, cm_delim);
    if (args_num != 1) {
        goto err;
    }
    tok = strtok(args, cm_delim);
    ret = sscanf(tok, "%i", &tmp);
    if (ret < 1)
        goto err;
    ret =
        cm_call(CiRegRead(chan, (_u32) tmp, (_u32 *) & val),
        "CiRegRead failed");
    if (ret < 0)
        return -1;
    cmprint(stdout, "reg val: 0x%lx\n", (unsigned long) val);
    return 0;
err:
    command_usage("rreg");
    return -1;
}

//wreg
int cm_wreg(char *args) 
{
    int args_num, ret;
    char *tok;
    int tmp, tmpval;
    _u32 val;

    args_num = get_token_num(args, cm_delim);
    if (args_num != 2) {
        goto err;
    }
    tok = strtok(args, cm_delim);
    ret = sscanf(tok, "%i", &tmp);
    if (ret < 1)
        goto err;
    tok = strtok(NULL, cm_delim);
    ret = sscanf(tok, "%i", &tmpval);
    if (ret < 1)
        goto err;
    val = (_u32) tmpval;
    return cm_call(CiRegWrite(chan, (_u32) tmp, (_u32) val),
        "CiRegWrite failed");
err:
    command_usage("wreg");
    return -1;
}


void register_all_commands(void) 
{

    register_command("help", cm_help, "[command name]",
        "display help message on command");

    register_command("start", cm_start, NULL,
        "start CAN-chip (leave reset mode)");

    register_command("stop", cm_stop, NULL,
        "stop CAN-chip (enter reset mode)");

    register_command("filter", cm_filter, "on [acode] [amask] | off",
        "set hardware acceptance filter");

    register_command("lom", cm_lom, "on|off", "switch Listen Only Mode");
    
	register_command("show", cm_show, "on|off", "switch frames output");

    register_command("tout", cm_tout, "wr|tr [millisecs]",
        "set write/transmit timeout for channel (in milliseconds)");

    register_command("tresh", cm_tresh, "rcq|trq [val]",
        "set RC or TR queue treshold for channel");

    register_command("wr", cm_wr, "id:[rtrN]|[d1,d2 ... d8] sff|eff [repeat num]\n\
         example1: wr 0x2:0x3,0x4,0x5 sff\n\
         example2: wr 0x2:0x3,0x4,0x5 eff repeat 5\n\
         example3: wr 0x2:rtr2 eff repeat 5\n",
                                  "write frame to CAN-bus");

    register_command("tr", cm_tr, "id:[rtrN]|[d1,d2 ... d8] sff|eff [repeat num]\n\
         example1: tr 0x2:0x3,0x4,0x5 sff\n\
         example2: tr 0x2:0x3,0x4,0x5 eff repeat 5\n\
         example3: tr 0x2:rtr2 eff repeat 5\n",
                                  "transmit frame to CAN-bus");

    register_command("stat", cm_stat, NULL,
        "print CAN-channel and CAN-chip status");

    register_command("reset", cm_reset, "hw|tr|cnt", "reset feature:\n\
         hw  - hard reset CAN-chip, reset hovr and sovr counters also,\n\
         tr  - reset(cancel) current transmition and clear trasnmit queue (trq)\n\
         cnt - reset recieve, transmit and errors counters\n");

    register_command("log", cm_log, "on [filename] | off",
        "open/close logfile and switch logging on/off");

    register_command("rreg", cm_rreg, "[offset]",
        "read CAN-controller register at given offset");

    register_command("wreg", cm_wreg, "[offset] [val]",
        "write CAN-controller register at given offset");

    register_command("quit", cm_quit, NULL, "exit CANmonitor");

}
