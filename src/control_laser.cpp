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
    connect(ui->pushButton_6, SIGNAL(clicked(bool)), this, SLOT(connect_adapter()));
    connect(ui->pushButton_4, SIGNAL(clicked(bool)), this, SLOT(apply_settings()));
    connect(ui->pushButton, SIGNAL(clicked(bool)), this, SLOT(send_laser_on()));
    connect(ui->pushButton_2, SIGNAL(clicked(bool)), this, SLOT(send_laser_off()));
    connect(ui->pushButton_3, SIGNAL(clicked(bool)), this, SLOT(send_change_sync()));
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

void control_laser::can_arrays_init(void)
{
    can_state = OFF; // статус адаптера - откл.
    is_update_freq_t = false;
    is_update_energy = false;

    uint8_t init_array[8] = {0, 0, 0, 0, 0, 0, 0, 0};
    memcpy(rx_data_status, init_array, 8);
    memcpy(rx_data_settings_freq_t, init_array, 8);

    tx_laser_on.id = ID_LASER_ON;
    tx_laser_off.id = ID_LASER_OFF;
    tx_laser_sync.id = ID_LASER_SYNC;

    tx_laser_on.len = 8;
    tx_laser_off.len = 8;
    tx_laser_sync.len = 8;

    memcpy(tx_laser_on.data, init_array, 8);
    memcpy(tx_laser_off.data, init_array, 8);
    memcpy(tx_laser_sync.data, init_array, 8);
}

void control_laser::timers_init()
{
    timer_rx_data = new QTimer();
    timer_rx_data->start(200);
    // TODO ?Отключение таймера когда адаптер не подключен?
}
