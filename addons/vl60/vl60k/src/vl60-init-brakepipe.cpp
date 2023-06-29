#include    "vl60.h"

#include    <QDir>

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
void VL60k::initBrakepipe(QString module_path)
{
    Q_UNUSED(module_path)

    // Тормозная магистраль
    double volume = length * 0.0343 * 0.0343 * Physics::PI / 4.0;
    brakepipe = new Reservoir(volume);

    // Концевые краны
    anglecock_tm_fwd = new PneumoAngleCock();
    anglecock_tm_fwd->read_config("pneumo-anglecock");
    anglecock_tm_bwd = new PneumoAngleCock();
    anglecock_tm_bwd->read_config("pneumo-anglecock");
}
