#ifndef     VEHICLE_EXTERIOR_H
#define     VEHICLE_EXTERIOR_H

#include    <osg/MatrixTransform>
#include    "animations-list.h"
#include    "display-container.h"

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
struct vehicle_exterior_t
{
    osg::ref_ptr<osg::MatrixTransform> transform = new osg::MatrixTransform();
    osg::ref_ptr<osg::Node> cabine = nullptr;
    osg::Vec3d  position = osg::Vec3d(0.0, 0.0, 0.0);
    osg::Vec3d  orth = osg::Vec3d(0.0, 0.0, 0.0);
    osg::Vec3d  up = osg::Vec3d(0.0, 0.0, 1.0);
    osg::Vec3d  right = osg::Vec3d(0.0, 0.0, 0.0);
    osg::Vec3d  attitude = osg::Vec3d(0.0, 0.0, 0.0);
    osg::Vec3d  driver_pos = osg::Vec3d(0.0, 0.0, 0.0);
    int         train_id = 0;
    int         orientation = 1;
    int         prev_vehicle = -1;
    int         next_vehicle = -1;

    animations_t *anims = new animations_t();
    displays_t   *displays = new displays_t();
    std::vector<size_t> sounds_id = {};

    vehicle_exterior_t()
    {

    }
};

#endif // VEHICLE_EXTERIOR_H
