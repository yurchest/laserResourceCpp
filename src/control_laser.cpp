#include "control_laser.h"
#include "ui_control_laser.h"


control_laser::control_laser(QMainWindow *parent)
    : QMainWindow(parent), ui(new Ui::control_laser)
{

    ui->setupUi(this);

    isAuthorized = false;

    chai_init();
    can_arrays_init(); // инициализируем массивы нулями
    timers_init();     // запускаем таймер

    connect(timer_rx_data, SIGNAL(timeout()), this, SLOT(receive_msg()), Qt::DirectConnection);
    connect(ui->pushButton_6, SIGNAL(clicked(bool)), this, SLOT(run_thread_con_disc_adapter()));
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
    timer_thread->exit();
    delete ui;
}

void control_laser::chai_init(){
    unsigned long chver = CiGetLibVer();
    printf("using CHAI %d.%d.%d\n", VERMAJ(chver), VERMIN(chver),
           VERSUB(chver));

    if (CiInit() < 0){ // инициализируем библиотеку CHAI для can адаптера
        printf("Can`t INIT CHAI");
        QMessageBox messageBox;
        QMessageBox::critical(0,"Error","Can`t INIT CHAI !");
        messageBox.setFixedSize(500,200);
        exit(1);
    }
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

void control_laser::run_thread_con_disc_adapter(){
    std::thread([this](){connect_disconnect_adapter();}).detach();

}


void control_laser::timers_init()
{

//    timer_rx_data->start(1000);
    timer_thread = new QThread(this);
    timer_rx_data = new QTimer(nullptr);
    timer_rx_data->setInterval(10);
    timer_rx_data->moveToThread(timer_thread);
    connect(timer_thread, SIGNAL(started()), timer_rx_data,SLOT(start()));
    timer_thread->start();
    // TODO ?Отключение таймера когда адаптер не подключен?
}
