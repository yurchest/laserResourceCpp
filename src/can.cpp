#include "control_laser.h"
#include "ui_control_laser.h"
#include <iostream>

void control_laser::receive_msg() {

    // TEST
    canmsg_t test_frame;
    test_frame.id = ID_WRITE_SETTINGS_FREQ_T;
    test_frame.len = 8;
    uint8_t array2[8] = {1, 245, 0, 201, 0, 202, 0, 54};
    std::memcpy(test_frame.data, array2, 8);

    if (can_state) {
        canmsg_t rx_buffer[64];
        canwait_t cw[1];
        cw[0].chan = 1;
        cw[0].wflags = CI_WAIT_RC | CI_WAIT_ER;
        _s16 ret, ret1, i;

//        CiWrite(0, &test_frame, 1);
        /* Блокирует работу потока выполнения до наступления заданного события или до
        наступления тайм-аута в одном из указанных каналов ввода-вывода. */
        ret = CiWaitEvent(cw, 1, -1);


        if (ret > 0) {
            if (cw[0].rflags & CI_WAIT_RC) { // получен кадр в канале cw[0].chan
                ret1 = CiRead(1, rx_buffer, 64);
                if (ret1 > 0){
                    for (i = 0; i< ret1; i++) {
                        gui_update(rx_buffer[i]);
                        for (_u8 j = 0; j < 7; j+=2){
                            std::cout << (rx_buffer[i].data[j] << 8 | rx_buffer[i].data[j+1]) << " ";
                        }
                        std::cout << std::endl;
                    }
                }

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
}

_s16 control_laser::board_info() {
    _s16 ret;               // код, возвращаемый ф-ией CiBoardInfo(&binfo)
    canboard_t binfo;       // объект структуры информации об адаптере
    uint8_t chan_cnt = 0;   // кол-во доступных каналов

    binfo.brdnum = 0;           // задаём порядковый номер проверяемого адаптера
    ret = CiBoardInfo(&binfo);  // проверяем подключенный адаптер

    if (ret < 0) {
        ui->statusbar->showMessage("Не удалось подключится к адаптеру");
    } else {
        for (short j: binfo.chip) { if (j >= 0) { chan_cnt++; }}

        brd_name = binfo.name;      // считываем наименование адаптера
        brd_manuf = binfo.manufact; // считываем производителя адаптера

        ui->statusbar->showMessage("Подключен адаптер " + brd_name + ", " + brd_manuf + ", кол-во доступных каналов: " +
                                   QString::number(chan_cnt));
    }
    return ret;
}

// Функция подключения/отключения CAN - адаптера
void control_laser::connect_disconnect_adapter() {
    _u16 trqcnt = 0;    // Вспомогательная переменная для хранения кол-ва стёртых посылок
    _u16 *ptr_trqcnt = &trqcnt;

    if (board_info() >= 0) {
        if (can_state == OFF) {
            CiPerror(CiOpen(CHANEL_NUMBER, CIO_CAN11), "CiOpen");             // открываем канал 0
            CiPerror(CiSetBaud(CHANEL_NUMBER, BCI_500K),"CiSetBaud");             // конфигурируем канал (устанавливаем скорость)
            CiPerror(CiRcQueResize(CHANEL_NUMBER, 4),"CiRcQueResize");       // конфигурируем канал (размер очереди приёма)
//            CiPerror(CiTrCancel(CHANEL_NUMBER, ptr_trqcnt), "CiTrCancel");  // стираем содержимое очереди приёма
            CiPerror(CiStart(CHANEL_NUMBER), "CiStart");                          // запускаем канал

            CiPerror(CiOpen(1, CIO_CAN11), "CiOpen");             // открываем канал 0
            CiPerror(CiSetBaud(1, BCI_500K),"CiSetBaud");             // конфигурируем канал (устанавливаем скорость)
            CiPerror(CiRcQueResize(1, 4),"CiRcQueResize");       // конфигурируем канал (размер очереди приёма)
//            CiPerror(CiTrCancel(1, ptr_trqcnt), "CiTrCancel");  // стираем содержимое очереди приёма
            CiPerror(CiStart(1), "CiStart");

            ui->pushButton_6->setText("Отключить адаптер");
            can_state = ON;
        } else {
            CiPerror(CiStop(CHANEL_NUMBER), "CiStop");                         // останавливаем канал
            CiPerror(CiClose(CHANEL_NUMBER), "CiClose");                       // закрываем канал 0
            ui->pushButton_6->setText("Подключить адаптер");
            ui->statusbar->showMessage("Адаптер отключен");
            can_state = OFF;
        }
    }
}

void control_laser::send_settings_data(const _u8 *tx_data_freq_t, const _u8 *tx_data_energy) {
    // TEST
    /*
    for (_u8 i = 0; i < 7; i+=2){
        qDebug() << (tx_data_energy[i] << 8 | tx_data_energy[i+1]);
    }
    */

    if (can_state)    // если адаптер подключен
    {
        canmsg_t tx_frame_t[2];
        int chaierr;
        int *p_chaierr = &chaierr;
        _u16 trqcnt;
        canwait_t cw;
        cw.chan = 0;
        cw.wflags = CI_WAIT_TR;

        tx_frame_t[0].id = ID_WRITE_SETTINGS_FREQ_T;
        tx_frame_t[1].id = ID_WRITE_SETTINGS_ENERGY;

        tx_frame_t[0].len = 8;
        tx_frame_t[1].len = 8;

        memcpy(tx_frame_t[0].data, tx_data_freq_t, 8);
        memcpy(tx_frame_t[1].data, tx_data_energy, 8);


        CiPerror(CiWrite(CHANEL_NUMBER, &tx_frame_t[0], 1), "CiWrite Settings Freg, T");
        CiPerror(CiWrite(CHANEL_NUMBER, &tx_frame_t[1], 1), "CiWrite Settings Energy");


//      CiPerror(CiTransmitSeries(CHANEL_NUMBER, tx_frame_t, 2, p_chaierr), "CiTransmit Settings");

    }
}


void control_laser::send_command(_u8 ID_TYPE_COMMAND) {

/*
    // TODO TEST
    ui->pushButton->setProperty("border", "red");
    ui->pushButton->setStyle(QApplication::style());
*/

    if (can_state)  // если адаптер подключен
    {
        canmsg_t tx;
        tx.len = 1;
        tx.id = ID_COMMAND;

        switch (ID_TYPE_COMMAND) {
            case LASER_ON_OFF:
                std::memcpy(tx.data, tx_laser_on_off, 8);
                break;

            case LASER_SYNC:
                std::memcpy(tx.data, tx_laser_sync, 8);
                break;

            case LASER_DRYING_OFF:
                std::memcpy(tx.data, tx_laser_drying_off, 8);
                break;

            default:
                break;
        }

        CiPerror(CiTransmit(CHANEL_NUMBER, &tx), "CiTransmit COMMAND");

    }
}
