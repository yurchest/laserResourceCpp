#include "control_laser.h"
#include "ui_control_laser.h"

void control_laser::apply_settings() {
    _u8 tx_data_freq_t[8]; // частота, время накачки 1,2
    _u8 tx_data_energy[8]; // мин, макс энергия 1064, 532

    read_frequency(tx_data_freq_t);   // Считываем частоту
    read_t1(tx_data_freq_t);          // Считываем время накачки 1
    read_t2(tx_data_freq_t);          // Считываем время накачки 2
    read_energy(tx_data_energy);      // Считываем мин-макс энергию 1064/532

    send_settings_data(tx_data_freq_t, tx_data_energy); // Отправляем посылку (команду записи)
}

void control_laser::read_frequency(_u8 *tx_data_freq_t) {
    int value = ui->spinBox->value();
    auto freq = static_cast<uint16_t>(value);

    tx_data_freq_t[AL] = freq >> 8;
    tx_data_freq_t[AH] = freq;
}

void control_laser::read_t1(_u8 *tx_data_freq_t) {
    int value = ui->spinBox_2->value();
    auto T1 = static_cast<uint16_t>(value);

    tx_data_freq_t[BL] = T1 >> 8;
    tx_data_freq_t[BH] = T1;
}

void control_laser::read_t2(_u8 *tx_data_freq_t) {
    int value = ui->spinBox_3->value();
    auto T2 = static_cast<uint16_t>(value);

    tx_data_freq_t[CL] = T2 >> 8;
    tx_data_freq_t[CH] = T2;
}

void control_laser::read_energy(_u8 *tx_data_energy) {
    double value = ui->doubleSpinBox->value();
    auto energy_1064_min = static_cast<uint16_t>(100 * value);
    value = ui->doubleSpinBox_3->value();
    auto energy_1064_max = static_cast<uint16_t>(100 * value);
    value = ui->doubleSpinBox_4->value();
    auto energy_532_min = static_cast<uint16_t>(100 * value);
    value = ui->doubleSpinBox_2->value();
    auto energy_532_max = static_cast<uint16_t>(100 * value);

    tx_data_energy[AL] = energy_1064_min >> 8;
    tx_data_energy[AH] = energy_1064_min;

    tx_data_energy[BL] = energy_1064_max >> 8;
    tx_data_energy[BH] = energy_1064_max;

    tx_data_energy[CL] = energy_532_min >> 8;
    tx_data_energy[CH] = energy_532_min;

    tx_data_energy[DL] = energy_532_max >> 8;
    tx_data_energy[DH] = energy_532_max;

};