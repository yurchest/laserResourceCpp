#include "control_laser.h"
#include "ui_control_laser.h"


control_laser::control_laser(QMainWindow *parent)
    : QMainWindow(parent), ui(new Ui::control_laser)
{

    ui->setupUi(this);

    isAuthorized = false;

    unsigned long chver = CiGetLibVer();
    printf("using CHAI %d.%d.%d\n\n", VERMAJ(chver), VERMIN(chver),
           VERSUB(chver));

    if (CiInit() < 0){ // инициализируем библиотеку CHAI для can адаптера
        printf("Can`t INIT CHAI");
        exit(1);
    }

    can_arrays_init(); // инициализируем массивы нулями
    timers_init();     // запускаем таймер

    connect(timer_rx_data, SIGNAL(timeout()), this, SLOT(receive_msg()));
    connect(ui->pushButton_6, SIGNAL(clicked(bool)), this, SLOT(connect_disconnect_adapter()));
    connect(ui->pushButton_4, SIGNAL(clicked(bool)), this, SLOT(apply_settings()));
    connect(ui->pushButton, &QPushButton::clicked, this, [this]{ send_command(LASER_ON_OFF);});
    connect(ui->pushButton_3, &QPushButton::clicked, this, [this]{ send_command(LASER_SYNC);});
    connect(ui->pushButton_2, &QPushButton::clicked, this, [this]{ send_command(LASER_DRYING_OFF);});
    connect(ui->pushButton_5, &QPushButton::clicked, this, [this]
            {
        is_update_freq_t = true;
        is_update_energy = true; });

    connect(ui->tabWidget, SIGNAL(currentChanged(int)), this, SLOT(onTabChanged(int)));
    connect(ui->pushButton_7, SIGNAL(clicked(bool)), this, SLOT(button_auth_click()));
    connect(ui->lineEdit_5, &QLineEdit::returnPressed, this, &control_laser::button_auth_click);

    ui->statusbar->showMessage("Подключите адаптер 'Марафон' нажатием на кнопку 'Подключить адаптер'");
}

control_laser::~control_laser()
{
    delete ui;
}

void control_laser::onTabChanged(int tabIndex) {
    if (tabIndex == 1) {
        if (not isAuthorized){
            ui->label_54->setText("");
            ui->lineEdit_5->setFocus();
            ui->stackedWidget->setCurrentWidget(ui->page_2);
        } else {
            ui->stackedWidget->setCurrentWidget(ui->page);
        }
    }
}

void control_laser::can_arrays_init()
{
    can_state = OFF; // статус адаптера - откл.
    is_update_freq_t = true;
    is_update_energy = true;

    tx_laser_on_off[0] = LASER_ON_OFF;
    tx_laser_sync[0] = LASER_SYNC;
    tx_laser_drying_off[0] = LASER_DRYING_OFF;

}

void control_laser::timers_init()
{
    timer_rx_data = new QTimer();
    timer_rx_data->start(200);
    // TODO ?Отключение таймера когда адаптер не подключен?
}
