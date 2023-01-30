#include "control_laser.h"
#include "ui_control_laser.h"

control_laser::control_laser(QMainWindow *parent)
    : QMainWindow(parent), ui(new Ui::control_laser)
{

    ui->setupUi(this);

    CiInit();          // инициализируем библиотеку CHAI для can адаптера
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

    ui->statusbar->showMessage("Подключите адаптер 'Марафон' нажатием на кнопку 'Подключить адаптер'");
}

control_laser::~control_laser()
{
    delete ui;
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
