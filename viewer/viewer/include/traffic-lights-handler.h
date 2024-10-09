#ifndef     TRAFFIC_LIGHTS_HANDLER_H
#define     TRAFFIC_LIGHTS_HANDLER_H

#include    <QObject>
#include    <QMap>

#include    <osgGA/GUIEventHandler>
#include    <osg/PagedLOD>

#include    <traffic-light.h>
#include    <settings.h>

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
class TrafficLightsHandler : public QObject, public osgGA::GUIEventHandler
{
    Q_OBJECT

public:

    TrafficLightsHandler(QObject *parent = Q_NULLPTR);

    ~TrafficLightsHandler();

    virtual bool handle(const osgGA::GUIEventAdapter &ea,
                        osgGA::GUIActionAdapter &aa);

    void deserialize(QByteArray &data);

    void load_signal_models(const settings_t &settings);

    osg::Group *getSignalsGroup()
    {
        if (signals_group.valid())
        {
            return signals_group.get();
        }

        return nullptr;
    }

private:

    QMap<QString, TrafficLight *> traffic_lights;

    osg::ref_ptr<osg::Group> signals_group = new osg::Group;

    QMap<QString, osg::ref_ptr<osg::PagedLOD>> signal_nodes;

    void printSignalInfo(TrafficLight *tl);
};

#endif
