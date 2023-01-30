#ifndef CONTROL_LASER_H
#define CONTROL_LASER_H

#include <QtWidgets/QMainWindow>
#include <QtCore/QTimer>
#include "../lib/chai.h"
#include "bits.h"

#define RED_LED QPixmap(":/leds/led-red-on.png")
#define GREEN_LED QPixmap(":/leds/green-led-on.png")
#define BLUE_LED QPixmap(":/leds/blue-led-on.png")
#define GREEN_TICK QPixmap(":/tick_cross/green_tick.png")
#define RED_CROSS QPixmap(":/tick_cross/red_cross.png")

#define CHANEL_NUMBER               0x0

#define ID_COMMAND                  0x01    // Пакет команды
#define ID_STATUS                   0x02    // Статусный пакет
#define ID_SETTINGS_FREQ_T          0x03    // Пакет данных (Частота, время накачек)
#define ID_SETTINGS_ENERGY          0x04    // Пакет данных (мин/макс энергия 1064/532)
#define ID_WRITE_SETTINGS_FREQ_T    0x05    // Пакет записи данных (Частота, время накачек)
#define ID_WRITE_SETTINGS_ENERGY    0x06    // Пакет записи данных (мин/макс энергия 1064/532)
#define ID_ENERGY_DIAG              0x07    // Пакет данных (измерения фотодиодов)

QT_BEGIN_NAMESPACE
namespace Ui
{
    class control_laser;
}
QT_END_NAMESPACE

class control_laser : public QMainWindow
{
    Q_OBJECT

public:
    explicit control_laser(QMainWindow *parent = nullptr);
    ~control_laser() override;

private:
    Ui::control_laser *ui;

    QTimer *timer_rx_data;

    bool str_error;
    uint8_t can_state;     // статус сети can (готова к работе или нет)
    bool is_update_freq_t; // Флаг для считывания настроек частоти и вр. накачек
    bool is_update_energy; // Флаг для считывания мин макс энергий 1064 532
    QString brd_name;      // наименование платы адаптера
    QString brd_manuf;     // производитель платы

    // uint8_t rx_data_status[8];
    // uint8_t rx_data_settings_freq_t[8];
    // uint8_t rx_data_energy[8];
    // uint8_t rx_data_energy_diag[8];

    uint8_t tx_data_freg_t[8]; // частота, время накачки 1,2
    uint8_t tx_data_energy[8]; // мин, макс энергия 1064, 532

    // Команды
    uint8_t tx_laser_on_off[8];
    uint8_t tx_laser_sync[8];
    uint8_t tx_laser_drying_off[8];

    typedef enum on_off
    {
        OFF,
        ON
    } TE_on_off;

    /*******  enum байтов посылок Can *************/
    typedef enum canbytes
    {
        AL,
        AH,
        BL,
        BH,
        CL,
        CH,
        DL,
        DH,
        DATA_NUM
    } TE_canbytes;

    // UI functions
    void timers_init();
    void can_arrays_init();
    

    // CAN functions
    _s16 board_info();
    void send_settings_data();

    // Settings TAB
    void read_frequency();
    void read_t1();
    void read_t2();
    void read_energy();

    void gui_update(canmsg_t rx_frame);
    void update_leds(const _u8 *rx_data_status);
    void update_freq_t(const _u8 *rx_data_settings_freq_t);
    void update_energy(const _u8 *rx_data_energy);
    void update_energy_diag(const _u8 *rx_data_energy_diag);

    void check_settings(const _u8 *rx_data_settings , _u32 ID);

private slots:
    void send_command(_u8 ID_TYPE_COMMAND);
    void connect_disconnect_adapter();
    void receive_msg();
    void apply_settings();
};

#endif // CONTROL_LASER_H
