#include    "train-manipulator.h"

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
TrainManipulator::TrainManipulator(settings_t settings, QObject *parent)
    : AbstractManipulator(parent)
    , settings(settings)
    , rel_pos(osg::Vec3d(settings.ext_cam_init_shift,
                         settings.ext_cam_init_height,
                         settings.ext_cam_init_dist))
    , angle_H(static_cast<double>(osg::DegreesToRadians(settings.ext_cam_init_angle_H)))
    , angle_V(static_cast<double>(osg::DegreesToRadians(settings.ext_cam_init_angle_V)))
{

}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
osg::Matrixd TrainManipulator::getMatrix() const
{
    osg::Matrixd matrix;

    matrix *= osg::Matrixd::rotate(-cp.attitude.x(), osg::Vec3d(1.0, 0.0, 0.0));
    matrix *= osg::Matrixd::rotate(-cp.attitude.z() + cp.is_orient_bwd * osg::PI, osg::Vec3d(0.0, 0.0, 1.0));
    matrix *= osg::Matrixd::translate(cp.position);

    return matrix;
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
osg::Matrixd TrainManipulator::getInverseMatrix() const
{
    osg::Matrixd invMatrix = osg::Matrixd::inverse(getMatrix());

    invMatrix *= osg::Matrixd::rotate(osg::PI_2 + angle_H, osg::Vec3d(0, 1, 0));
    invMatrix *= osg::Matrixd::rotate(angle_V, osg::Vec3d(1, 0, 0));
    invMatrix *= osg::Matrixd::translate(-rel_pos);

    return invMatrix;
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
bool TrainManipulator::performMovementRightMouseButton(const double eventTimeDelta,
                                                       const double dx,
                                                       const double dy)
{
    Q_UNUSED(eventTimeDelta)

    double k1 = static_cast<double>(settings.ext_cam_rot_coeff);

    angle_H += k1 * dx;
    angle_V += k1 * dy;

    return false;
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
bool TrainManipulator::handleMouseWheel(const osgGA::GUIEventAdapter &ea,
                                        osgGA::GUIActionAdapter &aa)
{
    Q_UNUSED(aa)

    osgGA::GUIEventAdapter::ScrollingMotion sm = ea.getScrollingMotion();

    float speed = settings.ext_cam_speed;

    if ( (ea.getModKeyMask() == osgGA::GUIEventAdapter::MODKEY_LEFT_SHIFT) ||
         (ea.getModKeyMask() == osgGA::GUIEventAdapter::MODKEY_RIGHT_SHIFT) )
    {
        speed = settings.ext_cam_speed_coeff * speed;
    }

    switch (sm)
    {
    case osgGA::GUIEventAdapter::SCROLL_UP:

        if (  (ea.getModKeyMask() == osgGA::GUIEventAdapter::MODKEY_LEFT_CTRL) ||
              (ea.getModKeyMask() == osgGA::GUIEventAdapter::MODKEY_RIGHT_CTRL))
        {
            rel_pos.y() -= speed * delta_time;
        }
        else
        {
            rel_pos.z() += speed * delta_time;
        }

        break;

    case osgGA::GUIEventAdapter::SCROLL_DOWN:

        if (  (ea.getModKeyMask() == osgGA::GUIEventAdapter::MODKEY_LEFT_CTRL) ||
              (ea.getModKeyMask() == osgGA::GUIEventAdapter::MODKEY_RIGHT_CTRL))
        {
            rel_pos.y() += speed * delta_time;
        }
        else
        {
            rel_pos.z() -= speed * delta_time;
        }

        break;

    default:

        break;
    }

    if (rel_pos.z() <= settings.ext_cam_min_dist)
        rel_pos.z() = settings.ext_cam_min_dist;

    return false;
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
bool TrainManipulator::handleKeyDown(const osgGA::GUIEventAdapter &ea,
                                     osgGA::GUIActionAdapter &aa)
{
    Q_UNUSED(aa)

    switch (ea.getKey())
    {
    case osgGA::GUIEventAdapter::KEY_Left:

        rel_pos.x() -= settings.ext_cam_speed * delta_time;
        break;

    case osgGA::GUIEventAdapter::KEY_Right:

        rel_pos.x() += settings.ext_cam_speed * delta_time;
        break;
    }

    return false;
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
TrainManipulator::~TrainManipulator()
{

}
