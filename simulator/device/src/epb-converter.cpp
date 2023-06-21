#include    "epb-converter.h"

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
EPBConverter::EPBConverter(QObject *parent) : Device(parent)
  , U_bat(0.0)
  , r(1.87)
  , ks(1.145)
  , f(625.0)
  , U_out(0.0)
  , I_out(0.0)
{

}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
EPBConverter::~EPBConverter()
{

}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
void EPBConverter::setInputVoltage(double value)
{
    U_bat = value;
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
void EPBConverter::setOutputCurrent(double value)
{
    I_out = value;
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
double EPBConverter::getOutputCurrent()
{
    return I_out;
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
double EPBConverter::getOutputVoltage()
{
    return U_out;
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
double EPBConverter::getOutputFrequency()
{
    return f;
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
void EPBConverter::step(double t, double dt)
{
    Q_UNUSED(t)
    Q_UNUSED(dt)

    U_out = ks * U_bat - r * abs(I_out);
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
void EPBConverter::ode_system(const state_vector_t &Y,
                              state_vector_t &dYdt, double t)
{
    Q_UNUSED(Y)
    Q_UNUSED(dYdt)
    Q_UNUSED(t)
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
void EPBConverter::load_config(CfgReader &cfg)
{
    QString secName = "Device";

    cfg.getDouble(secName, "r", r);
    cfg.getDouble(secName, "ks", ks);
    cfg.getDouble(secName, "f", f);
}