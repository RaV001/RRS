#ifndef     CONNECTOR_SPEEDMAP_H
#define     CONNECTOR_SPEEDMAP_H

#include    "topology-connector-device.h"

//------------------------------------------------------------------------------
// Автосцепка са-3
//------------------------------------------------------------------------------
class ConnectorSpeedMap : public ConnectorDevice
{
public:

    ConnectorSpeedMap(QObject *parent = Q_NULLPTR);

    ~ConnectorSpeedMap();

    virtual void step(double t, double dt);

private:

    void load_config(CfgReader &cfg);
};

#endif // CONNECTOR_SPEEDMAP_H