#ifndef     ENTER_SIGNAL_H
#define     ENTER_SIGNAL_H

#include    <trigger-counter.h>
#include    <rail-signal.h>
#include    <combine-releay.h>
#include    <timer.h>

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
class TOPOLOGY_EXPORT EnterSignal : public Signal
{
public:

    EnterSignal(QObject *parent = Q_NULLPTR);

    ~EnterSignal();

    void step(double t, double dt) override;

    void setFwdBusy(bool is_fwd_busy)
    {
        this->is_fwd_busy = is_fwd_busy;
    }

    void setBwdBusy(bool is_bwd_busy)
    {
        this->is_bwd_busy = is_bwd_busy;
    }

public slots:

    void slotPressOpen();

    void slotPressClose();

protected:

    enum
    {
        NUM_MSR_CONTACTS = 5,
        NUM_SSR_CONTACTS = 7,
        NUM_DSR_CONTACTS = 3,
        NUM_RCR_CONTACTS = 3,
        NUM_SR_CONTACTS = 5,
        NUM_ALR_CONTACTS = 1,
        NUM_ESR_CONTACTS = 1,
        NUM_LR_CONTACTS = 2,

        MSR_RED = 0,
        MSR_YELLOW = 1,
        MSR_PLUS = 2,
        MSR_MINUS = 3,
        MSR_BLINK = 4,

        SSR_RED = 0,
        SSR_TOP_YELLOW = 1,
        SSR_BOTTOM_YELLOW = 2,
        SSR_SIDE = 3,
        SSR_PLUS = 4,
        SSR_MINUS = 5,
        SSR_BLINK = 6,

        DSR_TOP_YELLOW = 0,
        DSR_GREEN = 1,
        DSR_BLINK = 2,

        RCR_SR_CTRL = 0,
        RCR_MSR_SSR_CTRL = 1,
        RCR_DSR_CTRL = 2,

        SR_SELF_LOCK = 0,
        SR_MSR_SSR_CTRL = 1,
        SR_ALR_CTRL = 2,
        SR_PLUS = 3,
        SR_MINUS = 4,

        ALR_MSR_SSR_CTRL = 0,

        ESR_DSR_CTRL = 0,

        LR_PLUS = 0,
        LR_MINUS = 1
    };

    /// Главное сигнальное реле
    Relay *main_signal_relay = new Relay(NUM_MSR_CONTACTS);

    /// Боковое сигнальное реле
    Relay *side_signal_relay = new Relay(NUM_SSR_CONTACTS);

    /// Сигнальное реле сквозного пропуска
    Relay *direct_signal_relay = new Relay(NUM_DSR_CONTACTS);

    /// Контрольное маршрутное реле
    Relay *route_control_relay = new Relay(NUM_RCR_CONTACTS);

    /// Сигнальное реле
    Relay *signal_relay = new Relay(NUM_SR_CONTACTS);

    /// Реле замыкания маршрута прибытия
    Relay *arrival_lock_relay = new Relay(NUM_ALR_CONTACTS);

    /// Указательное реле выходного светофора
    Relay *exit_signal_relay = new Relay(NUM_ESR_CONTACTS);

    /// Линейное реле связи с предвходным
    Relay *line_relay = new Relay(NUM_LR_CONTACTS);

    enum
    {
        NUM_FWD_BUSY = 3,
        BWD_BUSY_RED = 0,

        NUM_BWD_BUSY = 3,
        FWD_BUSY_PLUS = 0,
        FWD_BUSY_MINUS = 1,
        FWD_BUSY_CLOSE = 2
    };

    /// Путевое реле на учатке приближения
    Relay *fwd_way_relay = new Relay(NUM_FWD_BUSY);

    /// Путевое реле на стрелочном участке
    Relay *bwd_way_relay = new Relay(NUM_BWD_BUSY);

    /// Признак занятия учатка приближения
    bool is_fwd_busy = false;

    /// Признак занятия стрелочного участка
    bool is_bwd_busy = false;

    /// Признак нажатия кнопки открытия
    bool is_open_button_pressed = false;

    /// Признак НЕнажатия кнопки закрытия (нормально замкнутая)
    bool is_close_button_nopressed = true;

    double U_bat = 12.0;    

    /// Таймер выдержкм времени удержания кнопки открыть
    Timer *open_timer = new Timer(1.0, false);

    /// Таймер выдержки времени удержания кнопки закрыть
    Timer *close_timer = new Timer(1.0, false);

    bool is_SR_ON = false;

    bool is_RCR_ON = false;

    bool is_MSR_ON = false;

    bool is_SSR_ON = false;

    bool is_ALR_ON = false;    

    enum
    {
        NUM_AR_CONTACTS = 1,
        AR_OPEN = 0
    };

    /// Указательное реле, для связи с предыдущим входным светофором
    Relay *allow_relay = new Relay(NUM_AR_CONTACTS);

    /// Таймер мигания верхнего желтого сигнала
    Timer *blink_timer = new Timer(0.75, false);

    bool blink_contact = true;

    /// Реле мигания верхнего желтого
    enum
    {
        NUM_BLINK_CONTACTS = 2,
        BLINK_GREEN = 0,
        BLINK_YELLOW = 1
    };

    Relay *blink_relay = new Relay(NUM_BLINK_CONTACTS);

    bool is_yellow_wire_ON = false;

    TriggerCounter reset_alsn;

    Trigger set_alsn;

    void preStep(state_vector_t &Y, double t) override;

    void ode_system(const state_vector_t &Y,
                    state_vector_t &dYdt,
                    double t) override;

    /// Управление состоянием линз
    void lens_control();

    /// Контроль занятости примыкающих участков
    void busy_control();

    /// Управление цепями питания реле
    void relay_control();

    /// Проверка занятости маршрута по текущим стрелкам
    bool is_route_free(Connector *conn, Signal **signal);

    /// Проверка состояния стрелок по маршруту
    bool is_switch_minus(Connector *conn);

private slots:

    void slotOpenTimer();

    void slotCloseTimer();

    void slotOnBlinkTimer();
private:
    Signal * route_control();
    void signal_control();
    void arrival_lock();
    void signal_relays_control();
    void exit_signal_control(Signal *next_signal);
    void allow_signal_control();
    void blink_control(Signal *next_signal);
    void alsn_control();
};

#endif
