#include "ReadThread.h"
#include <iostream>

using namespace std;

ReadThread::ReadThread() {
    CHANEL_NUMBER  = 1;
    std::cout << CHANEL_NUMBER << std::endl;
}

void ReadThread::run() {

    while (1) {
        canmsg_t rx_buffer[1];
        canwait_t cw[1];
        cw[0].chan = CHANEL_NUMBER;
        cw[0].wflags = CI_WAIT_RC | CI_WAIT_ER;

        _s16 ret;

        /* Блокирует работу потока выполнения до наступления заданного события или до
        наступления тайм-аута в одном из указанных каналов ввода-вывода. */
        ret = CiWaitEvent(cw, 1, 1000);


        if (ret > 0) {
            if (cw[0].rflags & CI_WAIT_RC) { // получен кадр в канале cw[0].chan
                CiPerror(CiRead(CHANEL_NUMBER, rx_buffer, 1), "CiRead");
                emit(data_signal(rx_buffer));
            }
            if (cw[0].rflags & CI_WAIT_ER) { // в канале cw[0].chan произошла ошибка
                // читаем ошибки с помощью CiErrsGetClear()
                //CiErrsGetClear(chNum, &errs1);
            }
        } else if (ret < 0) {
            // ошибка CiWaitEvent()
//            reset_leds();
        } else { // ret == 0
            // timeout
//            reset_leds();
        }

    }

//    exec();
}
