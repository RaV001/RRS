#include    "train.h"

#include    "CfgReader.h"
#include    "physics.h"
#include    "Journal.h"

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
Train::Train(QObject *parent) : OdeSystem(parent)
{

}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
Train::~Train()
{

}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
bool Train::init(const init_data_t &init_data)
{
    solver_config = init_data.solver_config;

    dir = init_data.direction;

    coeff_to_wheel_rail_friction = init_data.coeff_to_wheel_rail_friction;

    // Solver loading
    FileSystem &fs = FileSystem::getInstance();
    QString solver_path = QString(fs.getLibraryDir().c_str()) + fs.separator() + solver_config.method;

    train_motion_solver = loadSolver(solver_path);

    Journal::instance()->info(QString("Created Solver object at address: 0x%1")
                              .arg(reinterpret_cast<quint64>(train_motion_solver), 0, 16));

    if (train_motion_solver == Q_NULLPTR)
    {
        Journal::instance()->error("Solver " + solver_path + " is't found");
        return false;
    }

    Journal::instance()->info("Loaded solver: " + solver_path);

    QString full_config_path = QString(fs.getTrainsDir().c_str()) +
            fs.separator() +
            init_data.train_config + ".xml";

    Journal::instance()->info("Train config from file: " + full_config_path);

    // Loading of train
    if (!loadTrain(full_config_path, init_data))
    {
        Journal::instance()->error("Train is't loaded");
        return false;
    }

    Journal::instance()->info("==== State vector ====");

    // State vector initialization
    y.resize(ode_order);
    dydt.resize(ode_order);

    Journal::instance()->info(QString("Allocated memory for %1 ODE's").arg(ode_order));

    Journal::instance()->info(QString("State vector address: 0x%1")
                              .arg(reinterpret_cast<quint64>(y.data()), 0, 16));

    Journal::instance()->info(QString("State vector derivative address: 0x%1")
                              .arg(reinterpret_cast<quint64>(dydt.data()), 0, 16));

    for (size_t i = 0; i < y.size(); i++)
        y[i] = dydt[i] = 0;

    // Loading of joints
    if (!loadTrainJoints())
    {
        Journal::instance()->error("Joints aren't loaded");
        return false;
    }

    // Set initial conditions
    Journal::instance()->info("==== Initial conditions ====");
    setInitConditions(init_data);

    initVehiclesBrakes();

    return true;
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
void Train::couple(double current_distance, bool is_coupling_to_head, bool is_other_coupled_by_head, Train *other_train)
{
    if (other_train == nullptr)
        return;

    // ПЕ поезда, с которым сцепляемся
    std::vector<Vehicle *> other_vehicles = *(other_train->getVehicles());

    // Вектор состояния поезда, с которым сцепляемся
    // TODO // Вектор состояния устроен не так просто, надо копировать по вайклам
    // TODO // И пересчитать каждый y[idx] на новую систему координат данного поезда
    // TODO // И дистанцию между поездами к моменту сцепления учесть
    // TODO // И дуговую координату в контроллере топологии заново инициализировать
    // TODO // И новый индекс в векторе состония вайклам задать
    state_vector_t other_y = other_train->getStateVector();
    double other_begin = other_y[0];
    std::vector<double> other_veh_distances;
    for (size_t i = 1; i < other_vehicles.size(); ++i)
    {
        size_t other_idx = other_vehicles[i]->getStateIndex();
        double other_coord = other_y[other_idx];
        other_veh_distances.push_back(abs(other_coord - other_begin));
        other_begin = other_coord;
    }

    double train1_coord_begin = y[0];
    Journal::instance()->info(QString("Vehicle   0 (#%1) coordinate[  0]: %2 (  0.000)")
                                  .arg(vehicles.front()->getModelIndex(), 4)
                                  .arg(y[0], 7, 'f', 3));
    for (size_t i = 1; i < vehicles.size(); ++i)
    {
        size_t state_idx = vehicles[i]->getStateIndex();
        double coord = y[state_idx] - train1_coord_begin;
        Journal::instance()->info(QString("Vehicle %1 (#%2) coordinate[%3]: %4 (%5)")
                                      .arg(i, 3)
                                      .arg(vehicles[i]->getModelIndex(), 4)
                                      .arg(vehicles[i]->getStateIndex(), 4)
                                      .arg(y[state_idx], 7, 'f', 3)
                                      .arg(coord, 7, 'f', 3));
    }

    double train2_coord_begin = other_y[0];
    Journal::instance()->info(QString("Vehicle   0 (#%1) coordinate[  0]: %2 (  0.000)")
                                  .arg(other_vehicles.front()->getModelIndex(), 4)
                                  .arg(other_y[0], 7, 'f', 3));
    for (size_t i = 1; i < other_vehicles.size(); ++i)
    {
        size_t state_idx = other_vehicles[i]->getStateIndex();
        double coord = other_y[state_idx] - train2_coord_begin;
        Journal::instance()->info(QString("Vehicle %1 (#%2) coordinate[%3]: %4 (%5)")
                                      .arg(i, 3)
                                      .arg(other_vehicles[i]->getModelIndex(), 4)
                                      .arg(other_vehicles[i]->getStateIndex(), 4)
                                      .arg(other_y[state_idx], 7, 'f', 3)
                                      .arg(coord, 7, 'f', 3));
    }

    // Массив межвагонных связей поезда, с которым сцепляемся
    std::vector<std::vector<Joint *>> other_joints_list = other_train->getJoints();

    Vehicle *veh;
    device_list_t *cons;
    Vehicle *other_veh;
    device_list_t *other_cons;

    // Соединяем ПЕ в общий массив
    if (is_coupling_to_head)
    {
        veh = *(vehicles.begin());
        cons = (veh->getOrientation() == -1) ?
                                  veh->getBwdConnectors() :
                                  veh->getFwdConnectors();

        std::vector<Vehicle *> new_vehicles;
        std::vector<std::vector<Joint *>> new_joints_list;
        state_vector_t new_y;
        size_t new_ode_order = 0;

        if (is_other_coupled_by_head)
        {
            other_veh = *(other_vehicles.begin());
            other_cons = (other_veh->getOrientation() == -1) ?
                             other_veh->getBwdConnectors() :
                             other_veh->getFwdConnectors();

            double distance = current_distance + veh->getLength() / 2.0 + other_veh->getLength() / 2.0;
            other_veh_distances.insert(other_veh_distances.begin(), distance);

            for (size_t i = other_vehicles.size(); i > 0; --i)
            {
                Vehicle *vehicle = other_vehicles[i - 1];
                new_vehicles.push_back(vehicle);

                size_t old_idx = vehicle->getStateIndex();
                size_t s = vehicle->getDegressOfFreedom();
                for (size_t j = old_idx; j < old_idx + 2 * s; ++j)
                {
                    new_y.push_back(other_y[j]);
                }

                vehicle->setDirection(dir);
                vehicle->setOrientation(-vehicle->getOrientation());
                vehicle->setTrainIndex(train_idx);

                vehicle->setStateIndex(new_ode_order);
                new_ode_order += 2 * s;
            }

            // Новые поездные координаты для прицепленных ПЕ
            double train_coord = y[0];
            for (size_t i = 0; i < other_vehicles.size(); ++i)
            {
                Vehicle *vehicle = other_vehicles[i];
                size_t model_idx = vehicle->getModelIndex();
                size_t idx = vehicle->getStateIndex();

                // На всякий случай актуализируем положение ПЕ в топологии
                // по старой дуговой координате
                topology->getVehicleController(model_idx)->setCoord(new_y[idx]);

                // Новая дуговая координата
                new_y[idx] = train_coord + dir * other_veh_distances[i];
                topology->getVehicleController(model_idx)->setInitCoord(new_y[idx]);
                train_coord = new_y[idx];
            }

            for (size_t i = other_joints_list.size(); i > 0; --i)
            {
                new_joints_list.push_back(other_joints_list[i - 1]);
                for (auto joint : other_joints_list[i - 1])
                {
                    joint->swapDevicesLinks();
                }
            }
        }
        else
        {
            //Временно
            return;
            // TODO //
            other_veh = *(other_vehicles.end() - 1);
            other_cons = (other_veh->getOrientation() == -1) ?
                             other_veh->getFwdConnectors() :
                             other_veh->getBwdConnectors();

            new_vehicles = other_vehicles;
            new_joints_list = other_joints_list;
            new_y = other_y;
        }

        // Создаём новый массив межвагонных связей между крайними ПЕ сцепляемых поездов
        std::vector<Joint *> joints;

        if ((cons->empty()) || (other_cons->empty()))
        {
            Journal::instance()->warning(QString("#%1 or #%2 have no connectors. Created empty array of joints.")
                                             .arg(veh->getModelIndex())
                                             .arg(other_veh->getModelIndex()));
        }
        else
        {
            loadJoints(cons, other_cons, joints);
        }

        if (joints.empty())
        {
            Journal::instance()->warning(QString("No joints beetween #%1 and #%2. Created empty array of joints.")
                                             .arg(veh->getModelIndex())
                                             .arg(other_veh->getModelIndex()));
        }
        else
        {
            Journal::instance()->info(QString("Created %1 joints beetween #%2 and #%3")
                                          .arg(joints.size())
                                          .arg(veh->getModelIndex())
                                          .arg(other_veh->getModelIndex()));
        }
        new_joints_list.push_back(joints);

        for (size_t i = 0; i < vehicles.size(); ++i)
        {
            Vehicle *vehicle = vehicles[i];

            size_t new_idx = vehicle->getStateIndex() + new_ode_order;
            vehicle->setStateIndex(new_idx);
        }

        for (size_t i = 0; i < y.size(); ++i)
        {
            new_y.push_back(y[i]);
        }

        vehicles.insert(vehicles.begin(), new_vehicles.begin(), new_vehicles.end());
        joints_list.insert(joints_list.begin(), new_joints_list.begin(), new_joints_list.end());
        y = new_y;
        //y.insert(y.begin(), new_y.begin(), new_y.end());

        ode_order += new_ode_order;
        train_motion_solver->setODEsize(ode_order);
        dydt.resize(ode_order);
    }
    else
    {
        //Временно
        return;
        // TODO //
        Vehicle *veh = *(vehicles.end() - 1);
        device_list_t *cons = (veh->getOrientation() == -1) ?
                                  veh->getFwdConnectors() :
                                  veh->getBwdConnectors();

        Vehicle *other_veh;
        device_list_t *other_cons;
        if (is_other_coupled_by_head)
        {
            other_veh = *(other_vehicles.begin());
            other_cons = (other_veh->getOrientation() == -1) ?
                             other_veh->getBwdConnectors() :
                             other_veh->getFwdConnectors();
        }
        else
        {
            other_veh = *(other_vehicles.end() - 1);
            other_cons = (other_veh->getOrientation() == -1) ?
                             other_veh->getFwdConnectors() :
                             other_veh->getBwdConnectors();
        }

        // Создаём новый массив межвагонных связей между крайними ПЕ сцепляемых поездов
        std::vector<Joint *> joints;

        if ((cons->empty()) || (other_cons->empty()))
        {
            Journal::instance()->warning(QString("#%1 or #%2 have no connectors. Created empty array of joints.")
                                             .arg(veh->getModelIndex())
                                             .arg(other_veh->getModelIndex()));
        }
        else
        {
            loadJoints(cons, other_cons, joints);
        }

        if (joints.empty())
        {
            Journal::instance()->warning(QString("No joints beetween #%1 and #%2. Created empty array of joints.")
                                             .arg(veh->getModelIndex())
                                             .arg(other_veh->getModelIndex()));
        }
        else
        {
            Journal::instance()->info(QString("Created %1 joints beetween #%2 and #%3")
                                          .arg(joints.size())
                                          .arg(veh->getModelIndex())
                                          .arg(other_veh->getModelIndex()));
        }
        joints_list.push_back(joints);

        if (is_other_coupled_by_head)
        {
            for (size_t i = other_vehicles.size(); i > 0; --i)
                vehicles.push_back(other_vehicles[i - 1]);
            for (size_t i = other_joints_list.size(); i > 0; --i)
                joints_list.push_back(other_joints_list[i - 1]);
            for (size_t i = other_y.size(); i > 0; --i)
                y.push_back(other_y[i - 1]);
        }
        else
        {
            for (size_t i = 0; i < other_vehicles.size(); ++i)
                vehicles.push_back(other_vehicles[i]);
            for (size_t i = 0; i < other_joints_list.size(); ++i)
                joints_list.push_back(other_joints_list[i]);
            for (size_t i = 0; i < other_y.size(); ++i)
                y.push_back(other_y[i]);
        }
    }

    Journal::instance()->info(QString("Trains coupled! New size of vehicles %1, joints %2, state_vector %3")
                                  .arg(vehicles.size(), 4)
                                  .arg(joints_list.size(), 4)
                                  .arg(y.size(), 4));
    double train_coord_begin = y[0];
    Journal::instance()->info(QString("Vehicle   0 (#%1) coordinate[  0]: %2 (  0.000)")
                                  .arg(vehicles.front()->getModelIndex(), 4)
                                  .arg(y[0], 7, 'f', 3));
    for (size_t i = 1; i < vehicles.size(); ++i)
    {
        size_t state_idx = vehicles[i]->getStateIndex();
        double coord = y[state_idx] - train_coord_begin;
        Journal::instance()->info(QString("Vehicle %1 (#%2) coordinate[%3]: %4 (%5)")
                                      .arg(i, 3)
                                      .arg(vehicles[i]->getModelIndex(), 4)
                                      .arg(vehicles[i]->getStateIndex(), 4)
                                      .arg(y[state_idx], 7, 'f', 3)
                                      .arg(coord, 7, 'f', 3));
    }

}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
void Train::setTrainIndex(size_t idx)
{
    train_idx = idx;
    for (auto vehicle : vehicles)
    {
        vehicle->setTrainIndex(idx);
    }
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
size_t Train::getTrainIndex() const
{
    return train_idx;
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
void Train::calcDerivative(state_vector_t &Y, state_vector_t &dYdt, double t, double dt)
{
    auto begin = vehicles.begin();
    auto end = vehicles.end();

    for (auto it = begin; it != end; ++it)
    {
        Vehicle *vehicle = *it;
        size_t idx = vehicle->getStateIndex();
        size_t s = vehicle->getDegressOfFreedom();

        state_vector_t a = vehicle->getAcceleration(Y, t, dt);

        memcpy(dYdt.data() + idx, Y.data() + idx + s, sizeof(double) * s);
        memcpy(dYdt.data() + idx + s, a.data(), sizeof(double) * s);
    }
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
void Train::preStep(double t)
{
    (void) t;

    auto begin = vehicles.begin();
    auto end = vehicles.end();

    for (auto it = begin; it != end; ++it)
    {
        Vehicle *vehicle = *it;
        size_t model_idx = vehicle->getModelIndex();
        size_t idx = vehicle->getStateIndex();

        topology->getVehicleController(model_idx)->setDirection(dir * vehicle->getOrientation());
        topology->getVehicleController(model_idx)->setCoord(y[idx]);
        *vehicle->getProfilePoint() = topology->getVehicleController(model_idx)->getPosition();
        vehicle->setFrictionCoeff(coeff_to_wheel_rail_friction);

        vehicle->integrationPreStep(y, t);
    }
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
bool Train::step(double t, double &dt)
{
    vehiclesStep(t, dt);

    // Train dynamics simulation
    bool done = true;
    double tau = 0.0;
    double _dt = dt / static_cast<double>(solver_config.num_sub_step);
    for (size_t i = 0; i < solver_config.num_sub_step; ++i)
    {
        done &= train_motion_solver->step(this, y, dydt, t, _dt,
                                          solver_config.max_step,
                                          solver_config.local_error);
        t +=_dt;
        tau +=_dt;
    }
    dt = tau;

    return done;
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
void Train::vehiclesStep(double t, double dt)
{
    bool is_joints = (vehicles.size() > 1);
    auto joints_it = joints_list.begin();
    auto begin = vehicles.begin();
    auto end = vehicles.end();

    for (auto it = begin; it != end; ++it)
    {
        // Если в поезде больше одной единицы ПС - просчитываем связи между ПС
        if ( is_joints && ( it != end - 1) )
        {
            std::vector<Joint *> joints = *joints_it;
            if (!joints.empty())
            {
                for (auto joint : joints)
                {
                    joint->step(t, dt);
                }
            }
            ++joints_it;
        }

        Vehicle *vehicle = *it;
        vehicle->integrationStep(y, t, dt);
    }
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
void Train::inputProcess()
{
    auto end = vehicles.end();
    auto begin = vehicles.begin();

    for (auto i = begin; i != end; ++i)
    {
        Vehicle *vehicle = *i;
        vehicle->keyProcess();
        vehicle->hardwareProcess();
    }
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
void Train::postStep(double t)
{
    //(void) t;
    auto begin = vehicles.begin();
    auto end = vehicles.end();

    for (auto it = begin; it != end; ++it)
    {
        Vehicle *vehicle = *it;
        vehicle->integrationPostStep(y, t);
    }
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
Vehicle *Train::getFirstVehicle() const
{
    return *vehicles.begin();
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
Vehicle *Train::getLastVehicle() const
{
    return *(vehicles.end() - 1);
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
state_vector_t Train::getStateVector()
{
    return y;
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
std::vector<std::vector<Joint *> > Train::getJoints()
{
    return joints_list;
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
double Train::getVelocity(size_t i) const
{
    if (i < vehicles.size())
    {
        size_t idx = vehicles[i]->getStateIndex();
        size_t s = vehicles[i]->getDegressOfFreedom();
        return y[idx + s];
    }

    return 0.0;
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
double Train::getMass() const
{
    return trainMass;
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
double Train::getLength() const
{
    return trainLength;
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
size_t Train::getVehiclesNumber() const
{
    return vehicles.size();
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
QString Train::getClientName()
{
    return client_name;
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
QString Train::getTrainID()
{
    return train_id;
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
int Train::getDirection() const
{
    return dir;
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
std::vector<Vehicle *> *Train::getVehicles()
{
    return &vehicles;
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
bool Train::loadTrain(QString cfg_path, const init_data_t &init_data)
{
    CfgReader cfg;
    FileSystem &fs = FileSystem::getInstance();

    Journal::instance()->info("==== Train loading ====");

    if (cfg.load(cfg_path))
    {
        // Get charging pressure and no air flag
        if (!cfg.getDouble("Common", "ChargingPressure", charging_pressure))
        {
            charging_pressure = 0.5;
        }

        if (!cfg.getDouble("Common", "InitMainResPressure", init_main_res_pressure))
        {
            init_main_res_pressure = 0.9;
        }

        if (!cfg.getBool("Common", "NoAir", no_air))
        {
            no_air = false;
        }

        if (!cfg.getString("Common", "ClientName", client_name))
        {
            client_name = "";
        }

        if (!cfg.getString("Common", "TrainID", train_id))
        {
            train_id = "";
        }

        QDomNode vehicle_node = cfg.getFirstSection("Vehicle");

        if (vehicle_node.isNull())
            Journal::instance()->error("There are not Vehicle sections in train config");

        ode_order = 0;

        while (!vehicle_node.isNull())
        {
            Journal::instance()->info("==== Vehicle's group loading ====");

            QString module_lib_name = "";
            if (!cfg.getString(vehicle_node, "Module", module_lib_name))
            {
                Journal::instance()->error("Section with Module library name is not found");
                break;
            }

            QString module_lib_dir = module_lib_name;
            if (!cfg.getString(vehicle_node, "ModuleDir", module_lib_dir))
            {
                Journal::instance()->error("Section with Module directory is not found, using directory with the same name as Module library");
                module_lib_dir = module_lib_name;
            }

            // Calculate module library path
            QString relModulePath = QString(fs.combinePath(module_lib_dir.toStdString(), module_lib_name.toStdString()).c_str());

            QString module_cfg_name = "";
            if (!cfg.getString(vehicle_node, "ModuleConfig", module_cfg_name))
            {
                Journal::instance()->error("Section with Config file name is not found");
                break;
            }

            QString module_cfg_dir = module_cfg_name;
            if (!cfg.getString(vehicle_node, "ModuleConfigDir", module_cfg_dir))
            {
                Journal::instance()->error("Section with Config directory is not found, using directory with the same name as Config file");
                module_cfg_dir = module_cfg_name;
            }

            // Calculate config file path
            QString relConfigPath = QString(fs.combinePath(module_cfg_dir.toStdString(), module_cfg_name.toStdString()).c_str());

            // Vehicles count
            int n_vehicles = 0;
            if (!cfg.getInt(vehicle_node, "Count", n_vehicles))
            {
                n_vehicles = 0;
                Journal::instance()->warning("Count of vehicles " + module_lib_name + " is not found. Vehicle willn't loaded");
            }

            // Orientation of vehicles group
            bool isForward = true;
            if (!cfg.getBool(vehicle_node, "IsOrientationForward", isForward))
            {
                isForward = true;
                Journal::instance()->warning("Orientations of vehicles " + module_lib_name + " is not found.");
            }
            int orient;
            if (isForward)
                orient = 1;
            else
                orient = -1;

            // Payload coefficient of vehicles group
            double payload_coeff = 0;
            if (!cfg.getDouble(vehicle_node, "PayloadCoeff", payload_coeff))
            {
                payload_coeff = 0;
            }

            for (int i = 0; i < n_vehicles; i++)
            {
                Journal::instance()->info("==== Vehicle loading ====");

                Vehicle *vehicle = loadVehicle(QString(fs.getModulesDir().c_str()) +
                                               fs.separator() +
                                               relModulePath);

                if (vehicle == Q_NULLPTR)
                {
                    Journal::instance()->error("Vehicle " + module_lib_name + " is't loaded");
                    break;
                }

                Journal::instance()->info(QString("Created Vehicle object at address: 0x%1")
                                          .arg(reinterpret_cast<quint64>(vehicle), 0, 16));

                vehicle->setModuleDir(module_lib_dir);
                vehicle->setModuleName(module_lib_name);
                vehicle->setConfigDir(module_cfg_dir);
                vehicle->setConfigName(module_cfg_name);
                vehicle->setRouteDir(init_data.route_dir_name);

                vehicle->setStateIndex(ode_order);
                vehicle->setPayloadCoeff(payload_coeff);
                vehicle->setDirection(dir);
                vehicle->setOrientation(orient);

                vehicle->init(QString(fs.getVehiclesDir().c_str()) + fs.separator() + relConfigPath + ".xml");

                trainMass += vehicle->getMass();
                trainLength += vehicle->getLength();

                size_t s = vehicle->getDegressOfFreedom();
                ode_order += 2 * s;

                if (vehicles.size() !=0)
                {
                    Vehicle *prev =  *(vehicles.end() - 1);
                    if (prev->getOrientation() > 0)
                        prev->setNextVehicle(vehicle);
                    else
                        prev->setPrevVehicle(vehicle);
                    if (vehicle->getOrientation() > 0)
                        vehicle->setPrevVehicle(prev);
                    else
                        vehicle->setNextVehicle(prev);
                }

                vehicles.push_back(vehicle);
            }

            vehicle_node = cfg.getNextSection();
        }
/*
        for (auto it = vehicles.begin(); it != vehicles.end(); ++it)
        {
            Vehicle *vehicle = *it;
            connect(this, &Train::sendDataToVehicle,
                    vehicle, &Vehicle::receiveData, Qt::DirectConnection);
        }*/
    }
    else
    {
        Journal::instance()->error("File " + cfg_path + " is't found");
    }

    // Check train is't empty and return
    return vehicles.size() != 0;
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
bool Train::loadTrainJoints()
{
    Journal::instance()->info("==== Joints loading ====");

    joints_list.clear();

    size_t num_joints = vehicles.size() - 1;

    if (num_joints == 0)
    {
        Journal::instance()->info("There is only one vehicle! No joints needed");
        return true;
    }

    size_t i = 0;
    auto begin = vehicles.begin();
    auto end = vehicles.end();

    for (auto it = begin; it != end - 1; ++it)
    {
        ++i;

        // Pair of neighbor vehicles, co-directional with route
        Vehicle *veh_fwd;
        Vehicle *veh_bwd;
        if (dir > 0)
        {
            veh_fwd = *it;
            veh_bwd = *(it+1);
        }
        else
        {
            veh_fwd = *(it+1);
            veh_bwd = *it;
        }

        // Get connectors list from ahead vehicle
        device_list_t *cons_fwd;
        if (dir * veh_fwd->getOrientation() > 0)
            cons_fwd = veh_fwd->getBwdConnectors();
        else
            cons_fwd = veh_fwd->getFwdConnectors();

        // Get connectors list from behind vehicle
        device_list_t *cons_bwd;
        if (dir * veh_bwd->getOrientation() > 0)
            cons_bwd = veh_bwd->getFwdConnectors();
        else
            cons_bwd = veh_bwd->getBwdConnectors();

        // Create array with joints between these two vehicle
        std::vector<Joint *> joints;

        if ((cons_fwd->empty()) || (cons_bwd->empty()))
        {
            joints_list.push_back(joints);
            Journal::instance()->warning(QString("#%1 or #%2 have no connectors. Created empty array of joints.")
                                      .arg(i - 1)
                                      .arg(i));
            continue;
        }

        loadJoints(cons_fwd, cons_bwd, joints);

        if (joints.empty())
        {
            Journal::instance()->warning(QString("No joints beetween #%1 and #%2. Created empty array of joints.")
                                         .arg(i - 1)
                                         .arg(i));
        }
        else
        {
            Journal::instance()->info(QString("Created %1 joints beetween #%2 and #%3")
                                      .arg(joints.size())
                                      .arg(i - 1)
                                      .arg(i));
        }

        // Add joints array to list of all joints
        joints_list.push_back(joints);
    }

    // Check there are joints for each pair of neighbor vehicles
    return joints_list.size() == num_joints;
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
void Train::loadJoints(device_list_t *cons_fwd, device_list_t *cons_bwd, std::vector<Joint *> &joints)
{
    // First try link connectors with the same name
    for (auto con_fwd_it = cons_fwd->begin(); con_fwd_it != cons_fwd->end(); ++con_fwd_it)
    {
        Device *con_fwd = *con_fwd_it;
        QString name_fwd = con_fwd->getName();

        for (auto con_bwd_it = cons_bwd->begin(); con_bwd_it != cons_bwd->end(); ++con_bwd_it)
        {
            Device *con_bwd = *con_bwd_it;
            QString name_bwd = con_bwd->getName();

            if (name_fwd == name_bwd)
            {
                loadJointModule(con_fwd, con_bwd, joints);
                break;
            }
        }
    }

    // Try link any connectors
    for (auto con_fwd_it = cons_fwd->begin(); con_fwd_it != cons_fwd->end(); ++con_fwd_it)
    {
        Device *con_fwd = *con_fwd_it;
        if (con_fwd->isLinked())
            continue;

        for (auto con_bwd_it = cons_bwd->begin(); con_bwd_it != cons_bwd->end(); ++con_bwd_it)
        {
            Device *con_bwd = *con_bwd_it;
            if (con_bwd->isLinked())
                continue;

            loadJointModule(con_fwd, con_bwd, joints);

            if (con_fwd->isLinked())
                break;
        }
    }
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
void Train::loadJointModule(Device *con_fwd, Device *con_bwd, std::vector<Joint *> &joints)
{
    CfgReader cfg;
    FileSystem &fs = FileSystem::getInstance();

    QString name_fwd = con_fwd->getName();
    QString name_bwd = con_bwd->getName();
    QString joint_cfg_name = QString("joint-" + name_fwd + "-" + name_bwd);
//    Journal::instance()->info(QString("Try to load config: " + joint_cfg_name));

    // Check forward connector's custom config directory
    QString cfg_dir = QString(fs.getVehiclesDir().c_str());
    QString fwd_cfg_subdir = con_fwd->getCustomConfigDir();

    QString joint_cfg_path = cfg_dir;
    joint_cfg_path += fs.separator() + fwd_cfg_subdir;
    joint_cfg_path += fs.separator() + joint_cfg_name + ".xml";

    if (!cfg.load(joint_cfg_path))
    {
        // Check backward connector's custom config directory
        QString bwd_cfg_subdir = con_bwd->getCustomConfigDir();

        joint_cfg_path = cfg_dir;
        joint_cfg_path += fs.separator() + bwd_cfg_subdir;
        joint_cfg_path += fs.separator() + joint_cfg_name + ".xml";

        if (!cfg.load(joint_cfg_path))
        {
            // Check default directory of devices configuration files
            joint_cfg_path = QString(fs.getDevicesDir().c_str());
            joint_cfg_path += fs.separator() + joint_cfg_name + ".xml";

            if (!cfg.load(joint_cfg_path))
            {
                return;
            }
        }
    }

    Journal::instance()->info("Loaded file: " + joint_cfg_path);
    QString secName = "Joint";

    QString joint_module_dir;
    if (cfg.getString(secName, "ModuleDir", joint_module_dir))
    {
        joint_module_dir = QString(fs.combinePath(fs.getModulesDir(), joint_module_dir.toStdString()).c_str());
    }
    else
    {
        joint_module_dir = QString(fs.getModulesDir().c_str());
    }

    QString joint_module_name = "";
    cfg.getString(secName, "ModuleName", joint_module_name);

    Joint *joint = loadJoint(QString(joint_module_dir +
                             fs.separator() + joint_module_name));
    if (joint == Q_NULLPTR)
        return;

    Journal::instance()->info("Loaded joint model from: " + joint_module_name + ".dll");

    joint->setLink(con_fwd, 0);
    joint->setLink(con_bwd, 1);
    joint->read_config(joint_cfg_path);

    joints.push_back(joint);
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
void Train::setInitConditions(const init_data_t &init_data)
{
    for (size_t i = 0; i < vehicles.size(); i++)
    {
        Vehicle *vehicle = vehicles[i];

        size_t s = vehicle->getDegressOfFreedom();
        size_t idx = vehicle->getStateIndex();

        y[idx + s] = init_data.init_velocity / Physics::kmh;
        for (size_t j = 1; j < s; j++)
        {
            double wheel_radius = vehicle->getWheelDiameter(j - 1) / 2.0;
            y[idx + s + j] = y[idx + s] / wheel_radius;
        }
    }

    double x0 = 0.0 - dir * this->getFirstVehicle()->getLength() / 2.0;
    y[0] = x0;

    vehicles[0]->setTrainCoord(x0);

    Journal::instance()->info(QString("Vehicle[%2] coordinate: %1").arg(y[0]).arg(0, 3));

    for (size_t i = 1; i < vehicles.size(); i++)
    {
        double Li_1 = vehicles[i-1]->getLength();
        size_t idxi_1 = vehicles[i-1]->getStateIndex();

        double Li = vehicles[i]->getLength();
        size_t idxi = vehicles[i]->getStateIndex();

        y[idxi] = y[idxi_1] - dir *(Li + Li_1) / 2;

        vehicles[i]->setTrainCoord(y[idxi]);

        Journal::instance()->info(QString("Vehicle[%2] coordinate: %1").arg(y[idxi]).arg(i, 3));
    }
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
void Train::initVehiclesBrakes()
{
    Journal::instance()->info("Initialization of vehicles brake devices...");

    for (size_t i = 0; i < vehicles.size(); ++i)
    {
        if (no_air)
        {
            vehicles[i]->initBrakeDevices(charging_pressure, 0.0, init_main_res_pressure);
        }
        else
        {
            double pBP = charging_pressure;
            vehicles[i]->initBrakeDevices(charging_pressure, pBP, init_main_res_pressure);
        }
    }
}
