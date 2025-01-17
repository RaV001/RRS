#include    "hysteresis.h"

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
Hysteresis::Hysteresis(double min_value,
                       double max_value,
                       bool init_state,
                       QObject *parent)
    : QObject(parent)
    , min(min_value)
    , max(max_value)
{
    if (init_state)
        state.set();
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
Hysteresis::~Hysteresis()
{

}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
void Hysteresis::setRange(double min_value, double max_value)
{
    min = min_value;
    max = max_value;
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
void Hysteresis::setValue(double value)
{
    if (value <= min)
        state.reset();

    if (value >= max)
        state.set();
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
bool Hysteresis::getState() const
{
    return state.getState();
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
sound_state_t Hysteresis::getSoundState(size_t idx) const
{
    return state.getSoundState(idx);
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
float Hysteresis::getSoundSignal(size_t idx) const
{
    return state.getSoundSignal(idx);
}
