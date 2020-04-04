#include    "starter-generator.h"

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
StarterGenerator::StarterGenerator(QObject *parent) : Device(parent)
  , Ua(0.0)
  , Uf(0.0)
  , Ia(0.0)
  , If(0.0)
  , Ra(0.039)
  , Rf(6.4)
  , Ta(0.1)
  , Tf(0.1)
  , is_motor(true)
{

}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
StarterGenerator::~StarterGenerator()
{

}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
void StarterGenerator::preStep(state_vector_t &Y, double t)
{
    Q_UNUSED(t)

    Ia = Y[0];
    If = Y[1];
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
void StarterGenerator::ode_system(const state_vector_t &Y,
                                  state_vector_t &dYdt,
                                  double t)
{
    if (is_motor)
    {
        ode_system_motor(Y, dYdt, t);
    }
    else
    {
        ode_system_generator(Y, dYdt, t);
    }
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
void StarterGenerator::load_config(CfgReader &cfg)
{

}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
void StarterGenerator::ode_system_motor(const state_vector_t &Y,
                                        state_vector_t &dYdt,
                                        double t)
{
    Q_UNUSED(t)
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
void StarterGenerator::ode_system_generator(const state_vector_t &Y,
                                            state_vector_t &dYdt,
                                            double t)
{
    Q_UNUSED(t)
}