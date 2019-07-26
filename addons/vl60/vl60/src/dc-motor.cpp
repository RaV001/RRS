#include    "dc-motor.h"

#include    <QDir>

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
    , omega_max(100.0)
    , direction(1)
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
void DCMotor::preStep(state_vector_t &Y, double t)
{
    torque = Y[0] * cPhi.getValue(Y[0] * beta * direction);

    emit soundSetPitch("TED", static_cast<float>(omega / omega_max));
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
void DCMotor::ode_system(const state_vector_t &Y,
                         state_vector_t &dYdt,
                         double t)
{
    double R = R_a + beta * R_gp + R_dp + R_r;
    double E = omega * cPhi.getValue(Y[0] * beta * direction);

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
    cfg.getDouble(secName, "OmegaMax", omega_max);

    QString cPhiFileName = "";

    cfg.getString(secName, "cPhi", cPhiFileName);

    cPhi.load((custom_config_dir + QDir::separator() + cPhiFileName).toStdString());
}