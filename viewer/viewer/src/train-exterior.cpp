#include    "train-exterior.h"

#include    "config-reader.h"
#include    "get-value.h"
#include    "filesystem.h"

#include    "vehicle-loader.h"

#include    <osgViewer/Viewer>

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
TrainExteriorHandler::TrainExteriorHandler(MotionPath *routePath, const std::string &train_config)
    : osgGA::GUIEventHandler ()
    , cur_vehicle(0)
    , routePath(routePath)
    , trainExterior(new osg::Group)
    , ref_time(0.0)
    , start_time(0.0)
{
    load(train_config);
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
bool TrainExteriorHandler::handle(const osgGA::GUIEventAdapter &ea,
                                  osgGA::GUIActionAdapter &aa)
{
    switch (ea.getEventType())
    {
    case osgGA::GUIEventAdapter::FRAME:
        {
            osgViewer::Viewer *viewer = dynamic_cast<osgViewer::Viewer *>(&aa);

            if (!viewer)
                break;

            double time = viewer->getFrameStamp()->getReferenceTime();
            double delta_time = time - start_time;
            ref_time += delta_time;
            start_time = time;

            moveTrain(ref_time, nd);

            moveCamera(viewer);

            break;
        }

    case osgGA::GUIEventAdapter::USER:
        {
            const network_data_t *nd = dynamic_cast<const network_data_t *>(ea.getUserData());

            if (nd->route_id != 0)
                processServerData(nd);
            else
            {
                this->nd.count = 0;
                ref_time = 0.0;
            }

            break;
        }

    case osgGA::GUIEventAdapter::KEYDOWN:
        {
            switch (ea.getKey())
            {
            case osgGA::GUIEventAdapter::KEY_Page_Down:

                cur_vehicle++;

                if (cur_vehicle > static_cast<int>(vehicles_ext.size() - 1))
                    cur_vehicle = 0;

                break;

            case osgGA::GUIEventAdapter::KEY_Page_Up:

                cur_vehicle--;

                if (cur_vehicle < 0)
                    cur_vehicle = static_cast<int>(vehicles_ext.size() - 1);

                break;
            }

            break;
        }

    default:

        break;
    }

    return false;
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
osg::Group *TrainExteriorHandler::getExterior()
{
    if (trainExterior.valid())
        return trainExterior.get();

    return nullptr;
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
void TrainExteriorHandler::load(const std::string &train_config)
{
    if (train_config.empty())
        return;

    FileSystem &fs = FileSystem::getInstance();
    std::string path = fs.combinePath(fs.getTrainsDir(), train_config + ".xml");

    ConfigReader cfg(path);

    if (!cfg.isOpenned())
        return;

    osgDB::XmlNode *config_node = cfg.getConfigNode();

    if (config_node == nullptr)
        return;

    for (auto it = config_node->children.begin(); it != config_node->children.end(); ++it)
    {
        osgDB::XmlNode *child = *it;

        if (child->name == "Vehicle")
        {
            int count = 0;

            osgDB::XmlNode *count_node = cfg.findSection(child, "Count");

            if (count_node != nullptr)
                getValue(count_node->contents, count);

            std::string module_config_name = "";

            osgDB::XmlNode *module_config_node = cfg.findSection(child, "ModuleConfig");

            if (module_config_node != nullptr)
            {
                getValue(module_config_node->contents, module_config_name);
            }

            osg::ref_ptr<osg::Group> vehicle_model = loadVehicle(module_config_name);

            for (int i = 0; i < count; ++i)
            {
                vehicle_exterior_t vehicle_ext;
                vehicle_ext.transform = new osg::MatrixTransform;
                vehicle_ext.transform->addChild(vehicle_model.get());

                vehicles_ext.push_back(vehicle_ext);
                trainExterior->addChild(vehicle_ext.transform.get());
            }
        }
    }
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
void TrainExteriorHandler::moveTrain(double ref_time, const network_data_t &nd)
{
    float t = static_cast<float>(ref_time) / nd.delta_time;

    for (size_t i = 0; i < vehicles_ext.size(); i++)
    {
        float coord = (1 - t) * nd.te[i].coord_begin + nd.te[i].coord_end * t;
        vehicles_ext[i].position = routePath->getPosition(coord, vehicles_ext[i].attitude);
        vehicles_ext[i].coord = coord;


        osg::Matrix  matrix;
        matrix *= osg::Matrix::rotate(static_cast<double>(vehicles_ext[i].attitude.x()), osg::Vec3(1.0f, 0.0f, 0.0f));
        matrix *= osg::Matrix::rotate(static_cast<double>(-vehicles_ext[i].attitude.z()), osg::Vec3(0.0f, 0.0f, 1.0f));
        matrix *= osg::Matrix::translate(vehicles_ext[i].position);


        vehicles_ext[i].transform->setMatrix(matrix);
    }
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
void TrainExteriorHandler::processServerData(const network_data_t *server_data)
{
    for (size_t i = 0; i < server_data->te.size(); ++i)
    {
        switch (nd.count)
        {
        case 0:

            nd.te[i].coord_begin = nd.te[i].coord_end = server_data->te[i].coord_end;
            nd.delta_time = server_data->delta_time;
            break;

        case 1:

            nd.te[i].coord_end = server_data->te[i].coord_end;
            nd.delta_time = server_data->delta_time;
            break;

        default:

            nd.te[i].coord_begin = vehicles_ext[i].coord;
            nd.te[i].coord_end = server_data->te[i].coord_end;
            nd.delta_time = server_data->delta_time;
            break;
        }
    }

    nd.count++;
    ref_time = 0.0;
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
void TrainExteriorHandler::moveCamera(osgViewer::Viewer *viewer)
{
    osg::Matrix viewMatrix;

    float coord = vehicles_ext[cur_vehicle].coord;
    osg::Vec3 position;
    osg::Vec3 attitude;

    position = routePath->getPosition(coord, attitude);

    position.z() += 3.0f;
    attitude.x() = -osg::PIf / 2.0f + attitude.x();

    viewMatrix = osg::Matrix::translate(position *= -1.0f);
    viewMatrix *= osg::Matrix::rotate(static_cast<double>(attitude.x()), osg::Vec3(1.0f, 0.0f, 0.0f));
    viewMatrix *= osg::Matrix::rotate(static_cast<double>(attitude.z()), osg::Vec3(0.0f, 1.0f, 0.0f));

    viewer->getCamera()->setViewMatrix(viewMatrix);
}