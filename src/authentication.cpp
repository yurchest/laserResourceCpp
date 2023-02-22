#include "control_laser.h"
#include "ui_control_laser.h"
#include <string>

#include "../lib/bcrypt/include/bcrypt.h"


void control_laser::button_auth_click() {
//    std::string pass = ui->lineEdit_5->text().toStdString();
//    std::string hash = bcrypt::generateHash(pass);
//    qDebug() << QString::fromStdString(hash);

// PASS: "vlp"

    if (bcrypt::validatePassword(ui->lineEdit_5->text().toStdString(),"$2b$10$susNUO7uYDRCauyCzkegSObPHQ.wOMTKsWHh/duYbWSszRGdJ30Bq")) {
        ui->stackedWidget->setCurrentWidget(ui->page);
        isAuthorized = true;
    } else {
        ui->lineEdit_5->setText("");
        ui->label_54->setText("Неверный пароль");
        ui->lineEdit_5->setFocus();
    }
}