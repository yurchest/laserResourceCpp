#include "control_laser.h"
#include "ui_control_laser.h"


void control_laser::receive_msg() {

    // TEST
    canmsg_t test_frame;

    uint8_t array2[8] = {1, 245, 0, 201, 0, 202, 7, 8};

    std::memcpy(test_frame.data, array2, 8);
    update_energy_diag(test_frame.data);
    update_energy(test_frame.data);
    update_leds(test_frame.data);
    check_settings(test_frame.data, ID_SETTINGS_FREQ_T);
    check_settings(test_frame.data, ID_SETTINGS_ENERGY);
    if (is_update_freq_t) {
        update_freq_t(test_frame.data);
        is_update_freq_t = false;
    }
    //

    if (can_state) {
        CiPerror(CiRead(1, rx_buffer, 4), "CiRead");

        for (canmsg_t rx_frame: rx_buffer) {
            switch (rx_frame.id) {
                case ID_STATUS:
                    update_leds(rx_frame.data);
                    break;

                case ID_SETTINGS_FREQ_T:
                    if (is_update_freq_t) {
                        update_freq_t(rx_frame.data);
                        check_settings(rx_frame.data, ID_SETTINGS_FREQ_T);
                        is_update_freq_t = false;
                    }
                    break;

                case ID_SETTINGS_ENERGY:
                    if (is_update_energy) {
                        update_energy(rx_frame.data);
                        check_settings(rx_frame.data, ID_SETTINGS_ENERGY);
                        is_update_energy = false;
                    }
                    break;

                case ID_ENERGY_DIAD:
                    update_energy_diag(rx_frame.data);
                    break;

                default:
                    break;
            }
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
void control_laser::connect_adapter() {
    _u16 trqcnt = 0;    // Вспомогательная переменная для хранения кол-ва стёртых посылок
    _u16 *ptr_trqcnt = &trqcnt;

    if (board_info() > 0) {
        if (can_state == OFF) {
            CiPerror(CiOpen(0, CIO_CAN11), "CiOpen");             // открываем канал 0
            CiPerror(CiSetBaud(0, BCI_250K), "CiSetBaud");        // конфигурируем канал (устанавливаем скорость)
            CiPerror(CiRcQueResize(0, 4), "CiRcQueResize");       // конфигурируем канал (размер очереди приёма)
            CiPerror(CiTrCancel(0, ptr_trqcnt), "CiTrCancel");    // стираем содержимое очереди приёма
            CiPerror(CiStart(0), "CiStart");                      // запускаем канал
            ui->pushButton_6->setText("Отключить адаптер");
            can_state = ON;
        } else {
            CiPerror(CiStop(0), "CiStop");                         // останавливаем канал
            CiPerror(CiClose(0), "CiClose");                       // закрываем канал 0
            ui->pushButton_6->setText("Подключить адаптер");
            ui->statusbar->showMessage("Адаптер отключен");
            can_state = OFF;
        }
    }
}

void control_laser::send_settings_data() {
    if (can_state)    // если адаптер подключен
    {
        canmsg_t tx_frame_t[2];
        int chaierr;
        int *p_chaierr = &chaierr;

        tx_frame_t[0].id = ID_SETTINGS_FREQ_T;
        tx_frame_t[1].id = ID_SETTINGS_ENERGY;

        tx_frame_t[0].len = 8;
        tx_frame_t[1].len = 8;

        memcpy(tx_frame_t[0].data, tx_data_freg_t, 8);
        memcpy(tx_frame_t[1].data, tx_data_energy, 8);

//        CiPerror(CiTransmit(0, &tx_frame_t[0]), "CiTransmit Settings Freg, T");
//        CiPerror(CiTransmit(0, &tx_frame_t[1]), "CiTransmit Settings Energy");

        CiPerror(CiTransmitSeries(0, tx_frame_t, 2, p_chaierr), "CiTransmit Settings");
    }
}


void control_laser::send_command(_u8 ID_TYPE_COMMAND) {
    if (can_state)  // если адаптер подключен
    {
        canmsg_t tx;
        tx.len = 1;
        tx.id = ID_COMMAND;

        switch (ID_TYPE_COMMAND) {
            case LASER_ON_OFF:
                std::memcpy(tx.data, tx_laser_on_off, 8);
                CiPerror(CiTransmit(0, &tx), "CiTransmit LASER ON");
                break;

            case LASER_SYNC:
                std::memcpy(tx.data, tx_laser_sync, 8);
                CiPerror(CiTransmit(0, &tx), "CiTransmit LASER SYNC");
                break;

            case LASER_DRYING_OFF:
                std::memcpy(tx.data, tx_laser_drying_off, 8);
                CiPerror(CiTransmit(0, &tx), "CiTransmit LASER DRYING OFF");
                break;

            default:
                break;

        }
    }
}
