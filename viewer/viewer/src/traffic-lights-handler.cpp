#include    <traffic-lights-handler.h>
#include    <QBuffer>

#include    <iostream>
#include    <QDir>
#include    <QDirIterator>
#include    <QFile>
#include    <QFileInfo>
#include    <filesystem.h>
#include    <config-reader.h>
#include    <osgDB/ReadFile>
#include    <osg/MatrixTransform>
#include    <osg/BlendFunc>
#include    <osg/AlphaFunc>
#include    <osg/PolygonMode>
#include    <osg/Depth>

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
TrafficLightsHandler::TrafficLightsHandler(QObject *parent)
    : QObject(parent)
    , osgGA::GUIEventHandler()
{

}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
TrafficLightsHandler::~TrafficLightsHandler()
{

}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
bool TrafficLightsHandler::handle(const osgGA::GUIEventAdapter &ea,
                                  osgGA::GUIActionAdapter &aa)
{
    switch (ea.getEventType())
    {
    case osgGA::GUIEventAdapter::FRAME:
        {
            for (auto tl = traffic_lights_fwd.begin(); tl != traffic_lights_fwd.end(); ++tl)
            {
                TrafficLight *traffic_light = tl.value();
                traffic_light->update();
            }

            for (auto tl = traffic_lights_bwd.begin(); tl != traffic_lights_bwd.end(); ++tl)
            {
                TrafficLight *traffic_light = tl.value();
                traffic_light->update();
            }

            break;
        }
    default: break;
    }

    return false;
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
void TrafficLightsHandler::deserialize(QByteArray &data)
{
    QBuffer buff(&data);
    buff.open(QIODevice::ReadOnly);
    QDataStream stream(&buff);

    size_t data_size = 0;
    stream >> data_size;

    std::cout << "Line signals : " << data_size << std::endl;

    // Очищаем список сигналов
    traffic_lights_fwd.clear();
    traffic_lights_bwd.clear();

    for (size_t i = 0; i < data_size; ++i)
    {
        QByteArray tmp_data;
        stream >> tmp_data;

        TrafficLight *tl = new TrafficLight;
        tl->deserialize(tmp_data);

        if (tl->getConnectorName().isEmpty())
        {
            continue;
        }

        printSignalInfo(tl);

        (tl->getSignalDirection() == -1) ?
            traffic_lights_bwd.insert(tl->getConnectorName(), tl) :
            traffic_lights_fwd.insert(tl->getConnectorName(), tl);
    }

    stream >> data_size;

    std::cout << "Enter signals : " << data_size << std::endl;

    for (size_t i = 0; i < data_size; ++i)
    {
        QByteArray tmp_data;
        stream >> tmp_data;

        TrafficLight *tl = new TrafficLight;
        tl->deserialize(tmp_data);

        if (tl->getConnectorName().isEmpty())
        {
            continue;
        }

        printSignalInfo(tl);

        (tl->getSignalDirection() == -1) ?
            traffic_lights_bwd.insert(tl->getConnectorName(), tl) :
            traffic_lights_fwd.insert(tl->getConnectorName(), tl);
    }

    stream >> data_size;

    std::cout << "Exit signals : " << data_size << std::endl;

    for (size_t i = 0; i < data_size; ++i)
    {
        QByteArray tmp_data;
        stream >> tmp_data;

        TrafficLight *tl = new TrafficLight;
        tl->deserialize(tmp_data);

        if (tl->getConnectorName().isEmpty())
        {
            continue;
        }

        printSignalInfo(tl);

        (tl->getSignalDirection() == -1) ?
            traffic_lights_bwd.insert(tl->getConnectorName(), tl) :
            traffic_lights_fwd.insert(tl->getConnectorName(), tl);
    }
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
void TrafficLightsHandler::create_pagedLODs(const settings_t &settings)
{
    FileSystem &fs = FileSystem::getInstance();

    std::string path = fs.combinePath(settings.route_dir_full_path, "topology");
    path = fs.combinePath(path, "models-config.xml");

    ConfigReader cfg_reader;    
    if (cfg_reader.load(path))
    {
        cfg_reader.getValue("Models", "SignalModelsDir", models_dir);
        cfg_reader.getValue("Models", "SignalAnimationsDir", animations_dir);
    }

    std::string models_path = fs.getDataDir();
    models_path = fs.combinePath(models_path, "models");
    models_path = fs.combinePath(models_path, models_dir);

    QDir models(QString(models_path.c_str()));
    QDirIterator models_files(models.path(), QStringList() << "*.osgt", QDir::NoDotAndDotDot | QDir::Files);

    while (models_files.hasNext())
    {
        QString fullPath = models_files.next();
        QFileInfo fileInfo(fullPath);

        QString model_base_name = fileInfo.baseName();

        /*osg::ref_ptr<osg::PagedLOD> pagedLOD = new osg::PagedLOD;
        pagedLOD->addDescription(model_base_name.toStdString());
        pagedLOD->setFileName(0, fullPath.toStdString());
        pagedLOD->setRange(0, 0.0f, settings.view_distance);
        pagedLOD->setRangeMode(osg::LOD::RangeMode::DISTANCE_FROM_EYE_POINT);

        signal_nodes.insert(model_base_name, pagedLOD);*/

        /*osg::ref_ptr<osg::Node> model = osgDB::readNodeFile(fullPath.toStdString());

        osg::StateSet *ss = model->getOrCreateStateSet();
        model->setDataVariance(osg::Object::DYNAMIC);*/

        signal_nodes_paths.insert(model_base_name, fullPath);
    }
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
void TrafficLightsHandler::slotUpdateSignal(QByteArray data)
{
    QBuffer buff(&data);
    buff.open(QIODevice::ReadOnly);
    QDataStream stream(&buff);

    QString conn_name = "";
    int signal_dir = 0;

    stream >> conn_name;
    stream >> signal_dir;

    if (conn_name.isEmpty())
    {
        return;
    }

    TrafficLight *tl = (signal_dir == -1) ?
                            traffic_lights_bwd.value(conn_name, nullptr) :
                            traffic_lights_fwd.value(conn_name, nullptr);

    if (tl == nullptr)
    {
        return;
    }

    tl->deserialize(data);    
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
void TrafficLightsHandler::slotUpdateBusyData(QByteArray &data)
{
    data.clear();
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
void TrafficLightsHandler::load_signal_models(const settings_t &settings)
{
    for (auto it = traffic_lights_fwd.begin(); it != traffic_lights_fwd.end(); ++it)
    {
        TrafficLight *tl = it.value();
        load_signal_model(tl, settings);
    }

    for (auto it = traffic_lights_bwd.begin(); it != traffic_lights_bwd.end(); ++it)
    {
        TrafficLight *tl = it.value();
        load_signal_model(tl, settings);
    }
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
void TrafficLightsHandler::load_signal_model(TrafficLight *tl, const settings_t &settings)
{
    if (!signal_nodes_paths.value(tl->getModelName(), "").isEmpty())
    {
        osg::ref_ptr<osg::MatrixTransform> transform = new osg::MatrixTransform;

        osg::Matrixd m1 = osg::Matrixd::translate(tl->getPosition());

        int sd = tl->getSignalDirection();

        osg::Vec3d o = tl->getOrth();
        osg::Vec3d r = tl->getRight();
        osg::Vec3d u = tl->getUp();


        osg::Matrixd m2(r.x(), -o.x(), u.x(), 0,
                        -r.y(), o.y(), u.y(), 0,
                        r.z(), o.z(), u.z(), 0,
                        0, 0, 0, 1);

        QString node_path = signal_nodes_paths.value(tl->getModelName(), "");

        osg::ref_ptr<osg::Node> signal_node = osgDB::readNodeFile(node_path.toStdString());
        signal_node->setDataVariance(osg::Object::DYNAMIC);

        TrafficLight *traffic_light = tl;
        traffic_light->setNode(signal_node.get());
        traffic_light->load_animations(animations_dir);

        animation_mangers.push_back(new AnimationManager(traffic_light->getAnimationsListPtr()));

        transform->setMatrix(m2 * m1);
        transform->addChild(signal_node.get());
        signals_group->addChild(transform);
    }
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
void TrafficLightsHandler::printSignalInfo(TrafficLight *tl)
{
    QString msg = QString("Signal at connector %1 is initialized. Letter: %2 | position: {%3, %4, %5} | direction: %6 {%7, %8, %9}")
                      .arg(tl->getConnectorName())
                      .arg(tl->getLetter())
                      .arg(tl->getPosition().x(), 8, 'f', 1)
                      .arg(tl->getPosition().y(), 8, 'f', 1)
                      .arg(tl->getPosition().z(), 8, 'f', 1)
                      .arg(tl->getSignalDirection() == -1 ? "BWD" : "FWD")
                      .arg(tl->getOrth().x(), 6, 'f', 3)
                      .arg(tl->getOrth().y(), 6, 'f', 3)
                      .arg(tl->getOrth().z(), 6, 'f', 3);

    std::cout << msg.toStdString() << std::endl;
}
