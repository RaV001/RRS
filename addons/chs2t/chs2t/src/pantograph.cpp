//------------------------------------------------------------------------------
//
//      Магистральный пассажирский электровоз постоянного тока ЧС2т.
//      Дополнение для Russian Railway Simulator (RRS)
//
//      (c) RRS development team:
//          Дмитрий Притыкин (maisvendoo),
//          Николай Авилкин (avilkin.nick)
//
//      Дата: 21/08/2019
//
//------------------------------------------------------------------------------

#include "pantograph.h"

//------------------------------------------------------------------------------
// Конструктор
//------------------------------------------------------------------------------
Pantograph::Pantograph(QObject* parent) : Device(parent)
  , tau(0)
  , h(0)
  , state(false)

{

}

//------------------------------------------------------------------------------
// Деструктор
//------------------------------------------------------------------------------
Pantograph::~Pantograph()
{

}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
void Pantograph::ode_system(const state_vector_t& Y, state_vector_t& dYdt, double t)
{

}

//------------------------------------------------------------------------------
//Загрузка данных из конфигурационного файла
//------------------------------------------------------------------------------
void Pantograph::load_config(CfgReader& cfg)
{
    cfg.getDouble("Pantograph", "height", hMax);
    cfg.getDouble("Pantograph", "speed", V);
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
void Pantograph::preStep(state_vector_t& Y, double t)
{

}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
void Pantograph::stepKeysControl(double t, double dt)
{
    double s0 = static_cast<double>(state);

    double s2 = s0 * hs_n(h - hMax);

    double s3 = (1.0 - s0) * hs_p(h);

    double s1 = s2 - s3;

    Uout = Uks * hs_p(h - hMax);

    h += s1 * V * dt;
}
