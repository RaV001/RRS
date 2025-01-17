#include    "loco-crane.h"

#include    <QLibrary>

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
LocoCrane::LocoCrane(QObject *parent) : BrakeDevice(parent)
  , pos(0.0)
  , is_release(0.0)
  , pFL(0.0)
  , pBC(0.0)
  , pIL(0.0)
  , QFL(0.0)
  , QBC(0.0)
  , QIL(0.0)
{
    sounds.resize(NUM_SOUNDS, sound_state_t());
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
LocoCrane::~LocoCrane()
{

}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
void LocoCrane::setHandlePosition(double position)
{
    pos = position;
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
void LocoCrane::setFLpressure(double value)
{
    pFL = value;
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
double LocoCrane::getFLflow() const
{
    return QFL;
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
void LocoCrane::setBCpressure(double value)
{
    pBC = value;
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
double LocoCrane::getBCflow() const
{
    return QBC;
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
void LocoCrane::setILpressure(double value)
{
    pIL = value;
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
double LocoCrane::getILflow() const
{
    return QIL;
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
void LocoCrane::release(bool is_release)
{
    this->is_release = static_cast<double>(is_release);
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
sound_state_t LocoCrane::getSoundState(size_t idx) const
{
    if (idx < sounds.size())
        return sounds[idx];
    return Device::getSoundState();
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
float LocoCrane::getSoundSignal(size_t idx) const
{
    if (idx < sounds.size())
        return sounds[idx].createSoundSignal();
    return Device::getSoundSignal();
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
LocoCrane *loadLocoCrane(QString lib_path)
{
    LocoCrane *crane = nullptr;

    QLibrary lib(lib_path);

    if (lib.load())
    {
        GetLocoCrane getLocoCrane = reinterpret_cast<GetLocoCrane>(lib.resolve("getLocoCrane"));

        if (getLocoCrane)
        {
            crane = getLocoCrane();
        }
    }

    return crane;
}
