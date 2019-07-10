//------------------------------------------------------------------------------
//
//      Магистральный пассажирский электровоз переменного тока ЧС4т.
//      Дополнение для Russian Railway Simulator (RRS)
//
//      (c) RRS development team:
//          Дмитрий Притыкин (maisvendoo),
//          Николай Авилкин (avilkin.nick)
//
//      Дата: 16/06/2019
//
//------------------------------------------------------------------------------


#include "stepswitch.h"

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
StepSwitch::StepSwitch(QObject* parent) : Device(parent)
  , KL(0)
  , UV(0)
  , s62(0)
  , s67(0)
  , s69(0)
{
    rs = new Trigger();

}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
void StepSwitch::setCtrlState(ControllerState controlState)
{
    this->controlState = controlState;
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
void StepSwitch::ode_system(const state_vector_t& Y, state_vector_t& dYdt, double t)
{

}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
void StepSwitch::load_config(CfgReader& cfg)
{

}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
void StepSwitch::preStep(state_vector_t& Y, double t)
{

}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
void StepSwitch::stepKeysControl(double t, double dt)
{
    if (controlState.a2b2 || controlState.e2f2)
        rs->set();

    if (1.0 - KL)
        rs->reset();

    double s01 = static_cast<double>(rs->getState());

    double s1 = s01 * KL;

    double s2 = s1 * controlState.c2d2 * UV;

    s62 = 1.0 - s2;
    s67 = s2;
    s69 = s2;

    if (controlState.a2b2 || controlState.j2k2)
        rs->set();

    if (1.0 - MN)
        rs->reset();

    s01 = static_cast<double>(rs->getState());

    s1 = s01 * MN;

    s2 = s1 * controlState.i2g2 * GH;

    s45 = 1.0 - s2;
    s43 = s2;
    s410 = s2;
}
