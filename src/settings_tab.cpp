#include "control_laser.h"
#include "ui_control_laser.h"

void control_laser::apply_settings() {
    read_frequency();   // Считываем частоту
    read_t1();          // Считываем время накачки 1
    read_t2();          // Считываем время накачки 2
    read_energy();      // Считываем мин-макс энергию 1064/532

    send_settings_data(); // Отправляем посылку (команду записи)
}

void control_laser::read_frequency() {
    QString str = ui->spinBox->cleanText();
    uint16_t freq = str.toInt(&str_error, 10);

    tx_data_freg_t[AL] = freq >> 8;
    tx_data_freg_t[AH] = freq;
}

void control_laser::read_t1() {
    QString str = ui->spinBox_2->cleanText();
    uint16_t T1 = str.toInt(&str_error, 10);

    tx_data_freg_t[BL] = T1 >> 8;
    tx_data_freg_t[BH] = T1;
}

void control_laser::read_t2() {
    QString str = ui->spinBox_3->cleanText();
    uint16_t T2 = str.toInt(&str_error, 10);

    tx_data_freg_t[CL] = T2 >> 8;
    tx_data_freg_t[CH] = T2;
}

void control_laser::read_energy() {
    QString str = ui->doubleSpinBox->cleanText();
    auto energy_1064_min = static_cast<uint16_t>(100 * str.toFloat());
    str = ui->doubleSpinBox_3->cleanText();
    auto energy_1064_max = static_cast<uint16_t>(100 * str.toFloat());
    str = ui->doubleSpinBox_4->cleanText();
    auto energy_532_min = static_cast<uint16_t>(100 * str.toFloat());
    str = ui->doubleSpinBox_2->cleanText();
    auto energy_532_max = static_cast<uint16_t>(100 * str.toFloat());

    tx_data_energy[AL] = energy_1064_min >> 8;
    tx_data_energy[AH] = energy_1064_min;

    tx_data_energy[BL] = energy_1064_max >> 8;
    tx_data_energy[BH] = energy_1064_max;

    tx_data_energy[CL] = energy_532_min >> 8;
    tx_data_energy[CH] = energy_532_min;

    tx_data_energy[DL] = energy_532_max >> 8;
    tx_data_energy[DH] = energy_532_max;

    // TODO
};