#include "control_laser.h"
#include "ui_control_laser.h"
#include <iostream>

void control_laser::update_leds(const _u8 *rx_data_status) {

    if (rx_data_status[AL] & BIT_MN1_ERROR) { ui->label_mn1_error->setPixmap(GREEN_LED); }
    else { ui->label_mn1_error->setPixmap(RED_LED); }

    if (rx_data_status[AL] & BIT_MN1_READY) { ui->label_mn1_ready->setPixmap(GREEN_LED); }
    else { ui->label_mn1_ready->setPixmap(RED_LED); }

    if (rx_data_status[AL] & BIT_MN1_ON) { ui->label_mn1_on->setPixmap(GREEN_LED); }
    else { ui->label_mn1_on->setPixmap(RED_LED); }


    // TODO Все лампочки
}

void control_laser::check_settings(const _u8 *rx_data_settings , _u32 ID){
    switch (ID){
        case ID_SETTINGS_FREQ_T:
            {
                _u16 rx_freq = rx_data_settings[AL] << 8 | rx_data_settings[AH];
                _u16 rx_t1 = rx_data_settings[BL] << 8 | rx_data_settings[BH];
                _u16 rx_t2 = rx_data_settings[CL] << 8 | rx_data_settings[CH];

                if (ui->spinBox->value() == rx_freq) { ui->label_11->setPixmap(GREEN_TICK); }
                else {ui->label_11->setPixmap(RED_CROSS);}

                if (ui->spinBox_2->value() == rx_t1) { ui->label_12->setPixmap(GREEN_TICK); }
                else {ui->label_12->setPixmap(RED_CROSS);}

                if (ui->spinBox_3->value() == rx_t2) { ui->label_13->setPixmap(GREEN_TICK); }
                else {ui->label_13->setPixmap(RED_CROSS);}

                break;
            }

        case ID_SETTINGS_ENERGY:
            {
                u_long rx_energy_1064_min = (u_long)((rx_data_settings[AL] << 8) | rx_data_settings[AH]) / 100;
                u_long rx_energy_1064_max = (u_long)((rx_data_settings[BL] << 8) | rx_data_settings[BH]) / 100;
                u_long rx_energy_532_min  = (u_long)((rx_data_settings[CL] << 8) | rx_data_settings[CH]) / 100;
                u_long rx_energy_532_max  = (u_long)((rx_data_settings[DL] << 8) | rx_data_settings[DH]) / 100;
                
                if (ui->doubleSpinBox->value() == rx_energy_1064_min & ui->doubleSpinBox_3->value() == rx_energy_1064_max) { 
                        ui->label_15->setPixmap(GREEN_TICK); 
                    }
                else {ui->label_15->setPixmap(RED_CROSS);}

                if (ui->doubleSpinBox_4->value() == rx_energy_532_min & ui->doubleSpinBox_2->value() == rx_energy_532_max) { 
                        ui->label_16->setPixmap(GREEN_TICK); 
                    }
                else {ui->label_16->setPixmap(RED_CROSS);}

                break;
            }
        
        default:
            break;
    }

}

void control_laser::update_freq_t(const _u8 *rx_data_settings_freq_t){

    ui->spinBox -> setValue(rx_data_settings_freq_t[AL] << 8 | rx_data_settings_freq_t[AH]); // FREQ
    ui->spinBox_2->setValue(rx_data_settings_freq_t[BL] << 8 | rx_data_settings_freq_t[BH]); // T1
    ui->spinBox_3->setValue(rx_data_settings_freq_t[CL] << 8 | rx_data_settings_freq_t[CH]); // T2

}

void control_laser::update_energy(const _u8 *rx_data_energy){

    u_long energy_1064_min = (u_long)((rx_data_energy[AL] << 8) | rx_data_energy[AH]) / 100;
    u_long energy_1064_max = (u_long)((rx_data_energy[BL] << 8) | rx_data_energy[BH]) / 100;
    u_long energy_532_min  = (u_long)((rx_data_energy[CL] << 8) | rx_data_energy[CH]) / 100;
    u_long energy_532_max  = (u_long)((rx_data_energy[DL] << 8) | rx_data_energy[DH]) / 100;

    ui->doubleSpinBox   ->setValue(energy_1064_min);
    ui->doubleSpinBox_3 ->setValue(energy_1064_max);
    ui->doubleSpinBox_4 ->setValue(energy_532_min);
    ui->doubleSpinBox_2 ->setValue(energy_532_max);
}

void control_laser::update_energy_diag(const _u8 *rx_data_energy_diag) {
    float energy_mn1  = (float)((rx_data_energy_diag[AL] << 8) | rx_data_energy_diag[AH]) / 100;
    float energy_mn2  = (float)((rx_data_energy_diag[BL] << 8) | rx_data_energy_diag[BH]) / 100;
    float energy_1064 = (float)((rx_data_energy_diag[CL] << 8) | rx_data_energy_diag[CH]) / 100;
    float energy_532  = (float)((rx_data_energy_diag[DL] << 8) | rx_data_energy_diag[DH]) / 100;

    ui->lineEdit_2->setText(QString::number(energy_mn1));
    ui->lineEdit_3->setText(QString::number(energy_mn2));
    ui->lineEdit_4->setText(QString::number(energy_1064));
    ui->lineEdit -> setText(QString::number(energy_532));
    
}