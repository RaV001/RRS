#ifndef     KRM_395_H
#define     KRM_395_H

#include    "brake-crane.h"

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
enum
{
    MAX_FLOW_COEFFS = 14,

    NUM_POSITIONS = 7,

    POS_I = 0,
    POS_II = 1,
    POS_III = 2,
    POS_IV = 3,
    POS_Va = 4,
    POS_V = 5,
    POS_VI = 6
};

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
class BrakeCrane395 : public BrakeCrane
{
    Q_OBJECT

public:

    BrakeCrane395(QObject *parent = Q_NULLPTR);

    ~BrakeCrane395();

    void init(double pBP, double pFL);

    void setHandlePosition(int position);

    QString getPositionName() const;

    double getHandlePosition() const;

private:

    /// Коэффициент утечек из УР
    double k_leak;

    /// Коэффициент зарядки УР из ГР в I и II положениях
    double k_charge;

    /// Коэффициент расхода из УР через стабилизатор в II положении
    /// (ликвидация сверхзарядного давления)
    double k_stab;

    /// Коэффициент разрядки УР в Va положении
    double k_Va;

    /// Коэффициент разрядки УР в V положении
    double k_V;

    /// Коэффициент разрядки УР в VI положении
    double k_VI;

    /// Коэффициент к разнице давлений ТМ и УР (условный уравнительный поршень)
    double A;

    /// Коэффициент зарядки ТМ из ГР в I положении
    double K_charge;

    /// Коэффициент к давлению импульсной сверхзарядки при отпуске вторым положением
    double K_pulse_II;

    /// Коэффициент зарядки ТМ из ГР от уравнительного поршня
    double K_feed;

    /// Коэффициент разрядки ТМ в атмосферу от уравнительного поршня
    double K_atm;

    /// Коэффициент разрядки ТМ в VI положении
    double K_VI;

    int handle_pos;

    double pos_delay;

    int min_pos;

    int max_pos;

    // Коэффициенты к громкости озвучки
    double Kv_in;
    double Kv_out;
//    double Kv_1;
    double Kv_2;
    double Kv_5;

    Timer   *incTimer;
    Timer   *decTimer;

    std::array<double, NUM_POSITIONS> pos;

    QStringList positions_names;

    void preStep(state_vector_t &Y, double t);

    void ode_system(const state_vector_t &Y, state_vector_t &dYdt, double t);

    void load_config(CfgReader &cfg);

    void stepKeysControl(double t, double dt);

private slots:

    void inc();

    void dec();
};

#endif // KRM_395_H
