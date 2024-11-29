//------------------------------------------------------------------------------
//
//      Электро-пневматический клапан автостопа (ЭПК) усл. №150
//      (c) maisvendoo, 06/05/2019
//
//------------------------------------------------------------------------------

#include    <epk150.h>
#include    <Journal.h>

/*
 *  Y[0] - усилие от катушки ЭПК
 *  Y[1] - давление в камере выдержки времени
 *  Y[2] - давление над срывным клапаном
 */

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
AutoTrainStopEPK150::AutoTrainStopEPK150(QObject *parent)
    : AutoTrainStop(parent)
    , T1(0.1)
    , pd(0.2)
    , pk(0.4)
    , p_key(10.0)
    , ps1(0.15)
    , ps2(0.15)
    , V1(1e-4)
    , V2(1e-3)
    , is_emergency_brake(false)
    , is_whistle_on(0.0)
{
    std::fill(K.begin(), K.end(), 0.0);
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
AutoTrainStopEPK150::~AutoTrainStopEPK150()
{

}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
void AutoTrainStopEPK150::init(double pBP, double pFL)
{
    setY(0, 0.0);
    setY(2, pBP);
    setY(1, pFL);
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
bool AutoTrainStopEPK150::getEmergencyBrakeContact() const
{
    return is_emergency_brake;
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
sound_state_t AutoTrainStopEPK150::getSoundState(size_t idx) const
{
    (void) idx;
    return sound_state_t(is_whistle_on > Physics::ZERO);
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
float AutoTrainStopEPK150::getSoundSignal(size_t idx) const
{
    (void) idx;
    return sound_state_t::createSoundSignal(is_whistle_on > Physics::ZERO);
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
void AutoTrainStopEPK150::ode_system(const state_vector_t &Y,
                                     state_vector_t &dYdt,
                                     double t)
{
    Q_UNUSED(t)

    double u1 = is_powered;

    double dp1 = Y[1] - ps2;

    double u2 = cut(pf(k[1] * dp1), 0.0, 1.0);

    double u3 = cut(nf(k[2] * dp1), 0.0, 1.0);

    double sum_p1 = Y[2] + ps1 - pBP;

    double u4 = cut(nf(k[3] * sum_p1), 0.0, 1.0);

    bool old_emergency_brake = is_emergency_brake;

    // Имитируем опускание диафрагмы в камере выдержки времени
    is_emergency_brake = Y[1] <= 0.15;

    if (is_emergency_brake != old_emergency_brake)
    {
        if (is_emergency_brake)
            Journal::instance()->info("EPK emergency brake!!!");
        else
            Journal::instance()->info("EPK is released...");
    }

    double sum_p2 = Y[0] + pk * (1.0 - is_key_on) - pd;

    is_whistle_on = cut(nf(k[4] * sum_p2), 0.0, 1.0);

    // Поток из питательной магистрали в камеру выдержки времени
    double Q_fl_2 = K[4] * (pFL - Y[1]) * hs_p(sum_p2);

    // Разрядка камеры выдержки времени в атмосферу
    double Q_2_atm = K[5] * Y[1] * is_whistle_on;

    // Поток из тормозной магистрали в камеру над срывным клапаном
    double Q_bp_1 = K[1] * (pBP - Y[2]);

    // Разрядка камеры над срывным клапаном в атмосферу
    double Q_1_atm = K[2] * Y[2] * u3;

    // Экстренная разрядка тормозной магистрали в атмосферу
    double Q_bp_emergency = K[3] * pBP * u4;

    // Поток в питательную магистраль
    QFL = - Q_fl_2;

    // Поток в тормозную магистраль
    QBP = - Q_bp_1 - Q_bp_emergency;

    // Поток в камеру выдержки времени
    dYdt[1] = (Q_fl_2 - Q_2_atm) / V2;

    // Поток в камеру над срывным клапаном
    dYdt[2] = (Q_bp_1 - Q_1_atm) / V1;

    dYdt[0] = ( pk * u1 * u2 - Y[0] ) / T1;
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
void AutoTrainStopEPK150::load_config(CfgReader &cfg)
{
    QString secName = "Device";

    for (size_t i = 1; i < K.size(); ++i)
    {
        QString coeff = QString("K%1").arg(i);
        cfg.getDouble(secName, coeff, K[i]);
    }

    for (size_t i = 1; i < k.size(); ++i)
    {
        QString coeff = QString("k%1").arg(i);
        cfg.getDouble(secName, coeff, k[i]);
    }

    cfg.getDouble(secName, "T1", T1);
    cfg.getDouble(secName, "pd", pd);
    cfg.getDouble(secName, "pk", pk);
    cfg.getDouble(secName, "p_key", p_key);
    cfg.getDouble(secName, "ps1", ps1);
    cfg.getDouble(secName, "ps2", ps2);
    cfg.getDouble(secName, "V1", V1);
    cfg.getDouble(secName, "V2", V2);
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
void AutoTrainStopEPK150::stepKeysControl(double t, double dt)
{
    Q_UNUSED(t)
    Q_UNUSED(dt)

    if (getKeyState(KEY_N))
    {
        setKeyOn(isShift());
    }

    /*if (getKeyState(KEY_K))
    {
        if ( getKeyState(KEY_Shift_L) || getKeyState(KEY_Shift_R) )
            powerOn(true);
        else
            powerOn(false);
    }*/
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
GET_AUTO_TRAIN_STOP(AutoTrainStopEPK150)
