#include    "airdistributor.h"

#include    <QLibrary>

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
AirDistributor::AirDistributor(QObject *parent) : BrakeDevice(parent)
  , pBP(0.0)
  , pBC(0.0)
  , pSR(0.0)
  , QBP(0.0)
  , QBC(0.0)
  , QSR(0.0)
{

}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
AirDistributor::~AirDistributor()
{

}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
void AirDistributor::setBPpressure(double value)
{
    pBP = value;
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
double AirDistributor::getBPflow() const
{
    return QBP;
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
void AirDistributor::setBCpressure(double value)
{
    pBC = value;
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
double AirDistributor::getBCflow() const
{
    return QBC;
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
void AirDistributor::setSRpressure(double value)
{
    pSR = value;
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
double AirDistributor::getSRflow() const
{
    return QSR;
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
AirDistributor *loadAirDistributor(QString lib_path)
{
    AirDistributor *airdist = nullptr;

    QLibrary lib(lib_path);

    if (lib.load())
    {
        GetAirDistributor getAirDistributor = reinterpret_cast<GetAirDistributor>(lib.resolve("getAirDistributor"));

        if (getAirDistributor)
        {
            airdist = getAirDistributor();
        }
    }

    return airdist;
}
