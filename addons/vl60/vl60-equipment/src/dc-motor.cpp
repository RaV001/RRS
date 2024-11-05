#include    "dc-motor.h"

#include    "filesystem.h"

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
DCMotor::DCMotor(QObject *parent) : Device(parent)
    , beta(0.95)
    , R_a(0.031)
    , R_gp(0.0238)
    , R_dp(0.032)
    , R_r(0.645)
    , L_af(0.001)
    , omega(0.0)
    , U(0.0)
    , torque(0.0)
    , direction(1)
    , torque_max(10000.0)
    , omega_nom(62.0)
{

}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
DCMotor::~DCMotor()
{

}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
void DCMotor::setOmega(double value)
{
    omega = value;
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
void DCMotor::setU(double value)
{
    U = value;
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
double DCMotor::getTorque() const
{
    return torque;
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
double DCMotor::getIa() const
{
    return getY(0);
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
double DCMotor::getIf() const
{
    return getIa() * beta;
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
double DCMotor::getUd() const
{
    return U - getIa() * R_r;
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
void DCMotor::setBeta(double value)
{
    beta = value;
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
void DCMotor::setBetaStep(int step)
{
    if (fieldStep.contains(step))
    {
        setBeta(fieldStep[step]);
    }
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
void DCMotor::setDirection(int revers_state)
{
    if (revers_state == 0)
        return;
    else
        direction = revers_state;
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
sound_state_t DCMotor::getSoundState(size_t idx) const
{
    (void) idx;
    return sound_state;
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
float DCMotor::getSoundSignal(size_t idx) const
{
    (void) idx;
    return sound_state.createSoundSignal();
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
void DCMotor::preStep(state_vector_t &Y, double t)
{
    Q_UNUSED(t)

    torque = Y[0] * calcCPhi(Y[0] * beta * direction);

    // Озвучка
    // Относительная громкость пропорциональна крутящему моменту
    double relative_volume = abs(torque) / torque_max;
    // Плавное включение на низкой скорости
    relative_volume = relative_volume * max(abs(omega) / 2.0, 1.0);
    sound_state.volume = static_cast<float>(relative_volume);

    // Относительная частота вращения от номинальной, для которой записан звук
    double relative_pitch = abs(omega) / omega_nom;
    // Костыль - нелинейное преобразование частоты (0.5 x^2 + 0.5),
    // чтобы охватить низкие скорости с частотой хотя бы 0.5
    if (relative_pitch < 1.0)
        relative_pitch = 0.5 * relative_pitch * relative_pitch + 0.5;
    sound_state.pitch = static_cast<float>(relative_pitch);
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
void DCMotor::ode_system(const state_vector_t &Y,
                         state_vector_t &dYdt,
                         double t)
{
    Q_UNUSED(t)

    double R = R_a + beta * R_gp + R_dp + R_r;
    double E = omega * calcCPhi(Y[0] * beta * direction);

    dYdt[0] = (U - R * Y[0] - E) / L_af;
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
void DCMotor::load_config(CfgReader &cfg)
{
    QString secName = "Device";

    cfg.getDouble(secName, "R_a", R_a);
    cfg.getDouble(secName, "R_gp", R_gp);
    cfg.getDouble(secName, "R_dp", R_dp);
    cfg.getDouble(secName, "R_r", R_r);
    cfg.getDouble(secName, "L_af", L_af);
    cfg.getDouble(secName, "TorqueMax", torque_max);
    cfg.getDouble(secName, "OmegaNom", omega_nom);

    QString cPhiFileName = "";

    cfg.getString(secName, "cPhi", cPhiFileName);


    FileSystem &fs = FileSystem::getInstance();
    QString cfg_dir(fs.getVehiclesDir().c_str());
    cfg_dir += fs.separator() + custom_cfg_dir;
    cPhi.load((cfg_dir + QDir::separator() + cPhiFileName).toStdString());

    QDomNode secNode;

    secNode = cfg.getFirstSection("FieldPos");

    while (!secNode.isNull())
    {
        double field_step = 0.95;
        int number = 0;

        cfg.getInt(secNode, "Number", number);
        cfg.getDouble(secNode, "beta", field_step);

        fieldStep.insert(number, field_step);

        secNode = cfg.getNextSection();
    }
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
double DCMotor::calcCPhi(double I)
{
    return cPhi.getValue(I);
}
