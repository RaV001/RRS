#include    "tep70.h"

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
void TEP70::stepOilSystem(double t, double dt)
{
    electro_oil_pump->step(t, dt);
}