//------------------------------------------------------------------------------
//
//      Train motion model simulation control
//      (c) maisvendoo, 02/09/2018
//      Developer: Dmitry Pritykin
//
//------------------------------------------------------------------------------
/*!
 * \file
 * \brief Train motion model simulation control
 * \copyright maisvendoo
 * \author Dmitry Pritykin
 * \date 02/09/2018
 */

#include    "model.h"

#include    <QTime>

#include    <CfgReader.h>
#include    <Journal.h>
#include    <JournalFile.h>

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
Model::Model(QObject *parent) : QObject(parent)  
{
    simulator_info_t tmp_si = simulator_info_t();
    memory_sim_info.setKey(SHARED_MEMORY_SIM_INFO);
    if (memory_sim_info.create(sizeof(simulator_info_t)))
    {
        Journal::instance()->info("Created shared memory for simulator info");
        memcpy(memory_sim_info.data(), &tmp_si, sizeof (simulator_info_t));
    }
    else
    {
        if (memory_sim_info.attach())
        {
            Journal::instance()->info("Attach to shared memory for simulator info");
            memcpy(memory_sim_info.data(), &tmp_si, sizeof (simulator_info_t));
        }
        else
        {
            Journal::instance()->error("No shared memory for simulator info");
        }
    }

    memory_sim_update.setKey(SHARED_MEMORY_SIM_UPDATE);
    if (memory_sim_update.create(sizeof(simulator_update_t)))
    {
        Journal::instance()->info("Created shared memory for simulator update data");
    }
    else
    {
        if (memory_sim_update.attach())
        {
            Journal::instance()->info("Attach to shared memory for simulator update data");
        }
        else
        {
            Journal::instance()->error("No shared memory for simulator update data");
        }
    }

    controlled_t tmp_c = controlled_t();
    memory_controlled.setKey(SHARED_MEMORY_CONTROLLED);
    if (memory_controlled.create(sizeof(controlled_t)))
    {
        Journal::instance()->info("Created shared memory for info about controlled vehicle");
        memcpy(memory_controlled.data(), &tmp_c, sizeof (controlled_t));
    }
    else
    {
        if (memory_controlled.attach())
        {
            Journal::instance()->info("Attach to shared memory for info about controlled vehicle");
            memcpy(memory_controlled.data(), &tmp_c, sizeof (controlled_t));
        }
        else
        {
            Journal::instance()->error("No shared memory for info about controlled vehicle");
        }
    }

    keys_data.setKey(SHARED_MEMORY_KEYS_DATA);
    if (keys_data.create(sizeof(KEYS_DATA_BYTEARRAY_SIZE)))
    {
        Journal::instance()->info("Created shared memory for keysboard processing");
    }
    else
    {
        if (keys_data.attach())
        {
            Journal::instance()->info("Attach to shared memory for keysboard processing");
        }
        else
        {
            Journal::instance()->error("No shared memory for keyboard data. Unable process keyboard");
        }
    }    
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
Model::~Model()
{
    memory_sim_info.detach();
    memory_sim_update.detach();
    memory_controlled.detach();
    keys_data.detach();
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
bool Model::init(const simulator_command_line_t &command_line)
{
    // Check is debug print allowed
    is_debug_print = command_line.debug_print.is_present;

    init_data_t init_data;

    // Load initial data configuration
    loadInitData(init_data);

    init_datas.push_back(init_data);

    // Override init data by command line
    overrideByCommandLine(init_data, command_line);

    // Read solver configuration
    configSolver(init_data.solver_config);    

    // Load route topology
    initTopology(init_data);

    // Create all trains
    for (size_t i = 0; i < init_datas.size(); ++i)
    {
        Train *train = addTrain(init_datas[i]);
        trains.push_back(train);
    }

    Journal::instance()->info("==== Info to shared memory ====");
    simulator_info_t   info_data;
    info_data.num_updates = 1;
    info_data.route_info.route_dir_name_length = init_data.route_dir_name.size();
    init_data.route_dir_name.toWCharArray(info_data.route_info.route_dir_name);
    Journal::instance()->info("Ready route info for shared memory");
    Journal::instance()->info("Route dir name: " + QString::fromWCharArray(info_data.route_info.route_dir_name));

    info_data.num_vehicles = vehicles.size();
    size_t i = 0;
    for (auto it = vehicles.begin(); it != vehicles.end(); ++it)
    {
        QString dir = (*it)->getConfigDir();
        info_data.vehicles_info[i].vehicle_config_dir_length = dir.size();
        dir.toWCharArray(info_data.vehicles_info[i].vehicle_config_dir);

        QString file = (*it)->getConfigName();
        info_data.vehicles_info[i].vehicle_config_file_length = file.size();
        file.toWCharArray(info_data.vehicles_info[i].vehicle_config_file);

        ++i;
    }
    Journal::instance()->info("Ready vehicles info for shared memory");

    if (memory_sim_info.lock())
    {
        memcpy(memory_sim_info.data(), &info_data, sizeof (simulator_info_t));
        memory_sim_info.unlock();
        Journal::instance()->info("Set info to shared memory");
    }
    else
    {
        Journal::instance()->error("Can't lock shared memory");
    }

    start_time = init_data.solver_config.start_time;
    stop_time = init_data.solver_config.stop_time;
    dt = init_data.solver_config.step;
    integration_time_interval = init_data.integration_time_interval;

    initControlPanel("control-panel");

    initSimClient("virtual-railway");

    //initTraffic(init_data);

    initTcpServer();

    Journal::instance()->info("Simulator model is initialized successfully");

    return true;
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
void Model::start()
{
    if (!isStarted())
    {
        is_simulation_started = true;
        t = start_time;

        connect(&simTimer, &ElapsedTimer::process, this, &Model::process, Qt::DirectConnection);
        simTimer.setInterval(static_cast<quint64>(integration_time_interval));
        simTimer.start();
    }
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
bool Model::isStarted() const
{
    return is_simulation_started;
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
void Model::outMessage(QString msg)
{
    fputs(qPrintable(msg + "\n"), stdout);
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
void Model::controlProcess()
{
    control_panel->process();
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
void Model::findNearestVehicles()
{
    struct founded_distance
    {
        size_t train_idx;   ///< Индекс поезда в симуляции
        bool from_head;     ///< Признак сближения с другим поездом головой или хвостом
        double distance;    ///< Дистанция между поездами
    };

    // Массив для всех найденных пар близкорасположенных поездов
    QMap<size_t, founded_distance> nearest_trains;
    std::vector<size_t> trains_idx_to_delete;

    for (size_t train_idx = 0; train_idx < trains.size(); ++train_idx)
    {
        Train* train = trains[train_idx];
        if (train == nullptr)
            continue;

        int train_dir = train->getDirection();

        // От каждого поезда ищем вперёд и назад по топологии
        for (int dir_it : {1, -1})
        {
            // Индекс крайней ПЕ в поезде, от которой начинаем поиск
            int idx = (train_dir == dir_it) ?
                              train->getFirstVehicle()->getModelIndex() :
                              train->getLastVehicle()->getModelIndex();

            // Ищем другую ПЕ в пределах 10 метров, и дистанцию до неё в данный момент
            double current_distance = 0.0;
            int nearest_idx = topology->getVehicleController(idx)->getNearestVehicle(
                current_distance, DISTANCE_TO_COUPLE_TRAINS, dir_it);

            // Если ничего не нашли - дальше делать нечего
            if (nearest_idx == -1)
                continue;

            // Создаём число из индексов найденной пары ПЕ, в порядке возрастания
            size_t idx_pair = (idx < nearest_idx) ?
                                  MAX_NUM_VEHICLES * idx + nearest_idx :
                                  MAX_NUM_VEHICLES * nearest_idx + idx;

            // Поскольку предполагается, что поиск найдёт каждую пару ПЕ дважды,
            // то проверяем что эта пара уже найдена в предыдущих поездах
            if (nearest_trains.contains(idx_pair))
            {
                // Найденную дважды пару ПЕ соединяем в один поезд
                founded_distance fd = nearest_trains.value(idx_pair);

                // Но проверяем, если поезд замкнулся сам на себя - игнорируем
                if (fd.train_idx == train_idx)
                {
                    nearest_trains.remove(idx_pair);
                    break;
                }

                Journal::instance()->info(QString("t = %1s Founded vehicles #%2 and #%3 at distance %4 (%5) m")
                                              .arg(t)
                                              .arg(idx)
                                              .arg(nearest_idx)
                                              .arg(fd.distance, 7, 'f', 3)
                                              .arg(current_distance, 7, 'f', 3));
                Journal::instance()->info(QString("t = %1s Connect trains #%2 (from %3) and #%4 (from %5)")
                                              .arg(t)
                                              .arg(fd.train_idx)
                                              .arg(fd.from_head ? "head" : "tail")
                                              .arg(train_idx)
                                              .arg((train_dir == dir_it) ? "head" : "tail"));
                trains[fd.train_idx]->couple(current_distance, fd.from_head, (train_dir == dir_it), train);

                // Поезд прицеплен и больше не нужен
                trains_idx_to_delete.push_back(train_idx);

                // Найденная пара ПЕ тоже не нужна
                nearest_trains.remove(idx_pair);
                break;
            }
            else
            {
                // Сохраняем найденную пару ПЕ
                founded_distance fd;
                fd.train_idx = train_idx;
                fd.from_head = (train_dir == dir_it);
                fd.distance = current_distance;
                nearest_trains.insert(idx_pair, fd);
            }
        }
    }

    if (trains_idx_to_delete.empty())
        return;

    // Удаляем прицепленные поезда
    size_t min_idx = trains.size();
    for (auto train_idx : trains_idx_to_delete)
    {
        if (min_idx < train_idx)
            min_idx = train_idx;

        delete trains[train_idx];
        trains.erase(trains.begin() + train_idx);
        Journal::instance()->info(QString("Delete train %1")
                                      .arg(train_idx, 3));
    }

    // Назначаем новые порядковые индексы поездам после уменьшения массива
    for (size_t train_idx = min_idx; train_idx < trains.size(); ++train_idx)
    {
        Journal::instance()->info(QString("Train %1 now %2")
                                      .arg(trains[train_idx]->getTrainIndex(), 3)
                                      .arg(train_idx, 3));
        trains[train_idx]->setTrainIndex(train_idx);
    }
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
void Model::findFarthestVehicles()
{
    for (auto train : trains)
    {
        Train *uncoupled_train = train->uncouple(DISTANCE_TO_UNCOUPLE_TRAINS);
        if (uncoupled_train != nullptr)
        {
            Journal::instance()->info(QString("Uncoupled new train %1 ")
                                          .arg(trains.size(), 3));
            uncoupled_train->setTrainIndex(trains.size());
            trains.push_back(uncoupled_train);
        }
    }
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
void Model::preStep(double t)
{
    for (auto train : trains)
        train->preStep(t);
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
bool Model::step(double t, double &dt)
{
    bool step_correct = true;
    for (auto train : trains)
        step_correct &= train->step(t, dt);
    return step_correct;
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
void Model::postStep(double t)
{
    for (auto train : trains)
        train->postStep(t);
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
void Model::debugPrint()
{
    QString debug_info = QString("t = %1 realtime_delay = %2 time_step = %3 x = %10 v[first] = %4 v[last] = %5 trac = %6 pos = %7 eq_press = %8 bp_press = %9 pos = %11\n")
            .arg(t)
            .arg(realtime_delay)
            .arg(dt)
            .arg(trains[0]->getFirstVehicle()->getVelocity() * 3.6)
            .arg(trains[0]->getLastVehicle()->getVelocity() * 3.6)
            .arg(static_cast<double>(trains[0]->getFirstVehicle()->getAnalogSignal(0)))
            .arg(static_cast<int>(trains[0]->getFirstVehicle()->getAnalogSignal(3)))
            .arg(static_cast<double>(trains[0]->getFirstVehicle()->getAnalogSignal(2)))
            .arg(static_cast<double>(trains[0]->getFirstVehicle()->getAnalogSignal(4)))
            .arg(trains[0]->getFirstVehicle()->getTrainCoord())
            .arg(static_cast<double>(trains[0]->getFirstVehicle()->getAnalogSignal(20)));

    fputs(qPrintable(debug_info), stdout);
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
void Model::loadInitData(init_data_t &init_data)
{
    Journal::instance()->info("==== Init data loading ====");

    CfgReader cfg;
    FileSystem &fs = FileSystem::getInstance();
    QString cfg_path = QString(fs.getConfigDir().c_str()) + fs.separator() + "init-data.xml";

    if (cfg.load(cfg_path))
    {
        QString secName = "InitData";

        if (!cfg.getString(secName, "RouteDirectory", init_data.route_dir_name))
        {
            init_data.route_dir_name = "experimental-polygon";
        }

        if (!cfg.getString(secName, "TrainConfig", init_data.train_config))
        {
            init_data.train_config = "vl60pk-1543";
        }

        if (!cfg.getString(secName, "TrajectoryName", init_data.trajectory_name))
        {
            init_data.trajectory_name = "route1_0001_1";
        }

        if (!cfg.getInt(secName, "Direction", init_data.direction))
        {
            init_data.direction = 1;
        }

        if (!cfg.getDouble(secName, "InitCoord", init_data.init_coord))
        {
            init_data.init_coord = 780.0;
        }

        if (!cfg.getDouble(secName, "InitVelocity", init_data.init_velocity))
        {
            init_data.init_velocity = 0.0;
        }

        if (!cfg.getDouble(secName, "CoeffToWheelRailFriction", init_data.coeff_to_wheel_rail_friction))
        {
            init_data.coeff_to_wheel_rail_friction = 1.0;
        }

        if (!cfg.getInt(secName, "IntegrationTimeInterval", init_data.integration_time_interval))
        {
            init_data.integration_time_interval = 15;
        }

        if (!cfg.getInt(secName, "ControlTimeInterval", init_data.control_time_interval))
        {
            init_data.control_time_interval = 15;
        }

        control_delay = static_cast<double>(init_data.control_time_interval) / 1000.0;

        if (!cfg.getBool(secName, "DebugPrint", init_data.debug_print))
        {
            init_data.debug_print = false;
        }

        Journal::instance()->info("Loaded settings from: " + cfg_path);
    }
    else
    {
        Journal::instance()->error("File " + cfg_path + " not found");
    }
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
void Model::overrideByCommandLine(init_data_t &init_data,
                                  const simulator_command_line_t &command_line)
{
    Journal::instance()->info("==== Command line processing ====");

    if (command_line.route_dir.is_present)
    {
        init_data.route_dir_name = command_line.route_dir.value;
    }

    if (command_line.debug_print.is_present)
    {
        init_data.debug_print = command_line.debug_print.value;
    }

    if (!command_line.train_config.is_present)
    {
        Journal::instance()->info("Command line is empty. Apply init_data.xml config");
        return;
    }

    init_data_t id;
    init_datas.clear();

    if (command_line.route_dir.is_present)
    {
        init_data.route_dir_name = command_line.route_dir.value;
    }

    if (command_line.debug_print.is_present)
    {
        init_data.debug_print = command_line.debug_print.value;
    }

    if (!command_line.train_config.is_present)
    {
        Journal::instance()->info("Command line is empty. Apply init_data.xml config");        
        return;
    }

    for (size_t i = 0; i < command_line.train_config.value.size(); ++i)
    {
        id.route_dir_name = init_data.route_dir_name;
        id.train_config = command_line.train_config.value[i];

        if (command_line.init_coord.is_present)
            id.init_coord = command_line.init_coord.value[i];

        if (command_line.direction.is_present)
            id.direction = command_line.direction.value[i];

        if (command_line.trajectory_name.is_present)
            id.trajectory_name = command_line.trajectory_name.value[i];

        init_datas.push_back(id);
    }

    Journal::instance()->info("Apply command line settinds");
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
void Model::configSolver(solver_config_t &solver_config)
{
    Journal::instance()->info("==== Solver configuration ====");

    CfgReader cfg;
    FileSystem &fs = FileSystem::getInstance();
    QString cfg_path = QString(fs.getConfigDir().c_str()) + fs.separator() + "solver.xml";

    if (cfg.load(cfg_path))
    {
        QString secName = "Solver";

        if (!cfg.getString(secName, "Method", solver_config.method))
        {
            solver_config.method = "euler";
        }
        Journal::instance()->info("Integration method: " + solver_config.method);

        if (!cfg.getDouble(secName, "StartTime", solver_config.start_time))
        {
            solver_config.start_time = 0;
        }
        Journal::instance()->info("Start time: " + QString("%1").arg(solver_config.start_time));

        if (!cfg.getDouble(secName, "StopTime", solver_config.stop_time))
        {
            solver_config.stop_time = 10.0;
        }
        Journal::instance()->info("Stop time: " + QString("%1").arg(solver_config.stop_time));

        if (!cfg.getDouble(secName, "InitStep", solver_config.step))
        {
            solver_config.step = 3e-3;
        }
        Journal::instance()->info("Initial integration step: " + QString("%1").arg(solver_config.step));

        if (!cfg.getDouble(secName, "MaxStep", solver_config.max_step))
        {
            solver_config.max_step = 3e-3;
        }
        Journal::instance()->info("Maximal integration step: " + QString("%1").arg(solver_config.max_step));

        int tmp = 1;
        if (!cfg.getInt(secName, "SubStepNum", tmp))
        {
            solver_config.num_sub_step = 1;
        }
        else
        {
            solver_config.num_sub_step = static_cast<size_t>(tmp);
        }
        Journal::instance()->info("Number of substep: " + QString("%1").arg(solver_config.num_sub_step));

        if (!cfg.getDouble(secName, "LocalError", solver_config.local_error))
        {
            solver_config.local_error = 1e-5;
        }
        Journal::instance()->info("Local error of solution: " + QString("%1").arg(solver_config.local_error));
    }
    else
    {
        Journal::instance()->error("File " + cfg_path + " not found");
    }
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
void Model::initControlPanel(QString cfg_path)
{
    CfgReader cfg;
    FileSystem &fs = FileSystem::getInstance();
    QString full_path = QString(fs.getConfigDir().c_str()) + fs.separator() + cfg_path + ".xml";

    if (cfg.load(full_path))
    {
        QString secName = "ControlPanel";
        QString module_name = "";

        bool is_allow = true;

        cfg.getBool(secName, "Allow", is_allow);

        if (!is_allow)
        {
            return;
        }

        if (!cfg.getString(secName, "Plugin", module_name))
            return;

        control_panel = Q_NULLPTR;
        QString module_path = QString(fs.getPluginsDir().c_str()) + fs.separator() + module_name;
        control_panel = loadInterfaceDevice(module_path);

        if (control_panel == Q_NULLPTR)
            return;

        QString config_dir = "";

        if (!cfg.getString(secName, "ConfigDir", config_dir))
            return;

        config_dir = QString(fs.toNativeSeparators(config_dir.toStdString()).c_str());

        if (!control_panel->init(QString(fs.getConfigDir().c_str()) + fs.separator() + config_dir))
            return;

        int request_interval = 0;

        if (!cfg.getInt(secName, "RequestInterval", request_interval))
            request_interval = 100;

        controlTimer.setInterval(request_interval);
        connect(&controlTimer, &QTimer::timeout, this, &Model::controlProcess);

        int v_idx = 0;

        if (!cfg.getInt(secName, "Vehicle", v_idx))
            v_idx = 0;

        Vehicle *vehicle = trains[0]->getVehicles()->at(static_cast<size_t>(v_idx));

        connect(vehicle, &Vehicle::sendFeedBackSignals,
                control_panel, &VirtualInterfaceDevice::receiveFeedback);

        connect(control_panel, &VirtualInterfaceDevice::sendControlSignals,
                vehicle, &Vehicle::getControlSignals);

        controlTimer.start();
    }
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
void Model::initSimClient(QString cfg_path)
{
    /*if (trains[0]->getTrainID().isEmpty())
        return;

    if (trains[0]->getClientName().isEmpty())
        return;

    CfgReader cfg;
    FileSystem &fs = FileSystem::getInstance();
    QString full_path = QString(fs.getConfigDir().c_str()) + fs.separator() + cfg_path + ".xml";

    if (cfg.load(full_path))
    {
        QString secName = "VRServer";
        tcp_config_t tcp_config;

        cfg.getString(secName, "HostAddr", tcp_config.host_addr);
        int port = 0;

        if (!cfg.getInt(secName, "Port", port))
        {
            port = 1993;
        }

        tcp_config.port = static_cast<quint16>(port);
        tcp_config.name = trains[0]->getClientName();

        sim_client = new SimTcpClient();
        connect(this, &Model::getRecvData, sim_client, &SimTcpClient::getRecvData);
        sim_client->init(tcp_config);
        sim_client->start();

        Journal::instance()->info("Started virtual railway TCP-client...");

        connect(&networkTimer, &QTimer::timeout, this, &Model::virtualRailwayFeedback);
        networkTimer.start(100);
    }
    else
    {
        Journal::instance()->error("There is no virtual railway configuration in file " + full_path);
    }*/
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
Train *Model::addTrain(const init_data_t &init_data)
{
    Journal::instance()->info("==== Train initialization ====");
    Train *train = new Train();
    train->setTopology(topology);
    Journal::instance()->info(QString("Created Train object at address: 0x%1")
                                  .arg(reinterpret_cast<quint64>(train), 0, 16));

    if (train->init(init_data))
    {
        Journal::instance()->info(QString("Train initialized successfully"));

        train->setTrainIndex(trains.size());
        for (auto vehicle : *(train->getVehicles()))
        {
            vehicle->setModelIndex(vehicles.size());
            vehicles.push_back(vehicle);
        }

        topology_pos_t tp;
        tp.traj_name = init_data.trajectory_name;
        tp.traj_coord = init_data.init_coord;
        tp.dir = init_data.direction;

        if (topology->addTrain(tp, train->getVehicles()))
        {
            Journal::instance()->info("Train added to topology successfully");
        }
        else
        {
            Journal::instance()->critical("CAN'T INITIALIZE TRAIN AT TOPOLOGY");
            return nullptr;
        }

        return train;
    }
    else
    {
        Journal::instance()->error("Can't initialize Train");
        return nullptr;
    }
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
void Model::initTraffic(const init_data_t &init_data)
{
    traffic_machine = new TrafficMachine();

    FileSystem &fs = FileSystem::getInstance();
    std::string route_dir_path = fs.combinePath(fs.getRouteRootDir(), init_data.route_dir_name.toStdString());

    if (!traffic_machine->init(route_dir_path.c_str()))
    {
        Journal::instance()->error("Failed traffic initialization in route" +
                                   QString(route_dir_path.c_str()));
    }
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
void Model::initTopology(const init_data_t &init_data)
{
    Journal::instance()->info("==== Route topology loading ====");

    if (topology->load(init_data.route_dir_name))
    {
        Journal::instance()->info("Loaded topology for route " + init_data.route_dir_name);
    }
    else
    {
        Journal::instance()->error("FAILED TOPOLOGY LOAD!!!");
        exit(0);
    }
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
void Model::initTcpServer()
{
    Journal::instance()->info("==== TCP server initialization ====");

    FileSystem &fs = FileSystem::getInstance();
    std::string cfg_path = fs.getConfigDir() + fs.separator() + "tcp-server.xml";

    tcp_server->init(QString(cfg_path.c_str()));

    connect(tcp_server, &TcpServer::setTopologyData, this, &Model::slotGetTopologyData);

    tcp_simulator_update.vehicles.resize(vehicles.size());

    connect(tcp_server, &TcpServer::setSwitchState, topology, &Topology::getSwitchState);
    connect(topology, &Topology::sendSwitchState, tcp_server, &TcpServer::slotSendSwitchState);

    connect(topology, &Topology::sendTrajBusyState, tcp_server, &TcpServer::slotSendTrajBusyState);

    connect(tcp_server, &TcpServer::setSignalsData, this, &Model::slotGetSignalsData);

    for (auto signal : topology->getSignalsData()->line_signals)
    {
        connect(signal, &Signal::sendDataUpdate, tcp_server, &TcpServer::slotUpdateSignal);
    }

    for (auto signal : topology->getSignalsData()->enter_signals)
    {
        connect(signal, &Signal::sendDataUpdate, tcp_server, &TcpServer::slotUpdateSignal);
    }

    connect(tcp_server, &TcpServer::openSignal, topology, &Topology::slotOpenSignal);

    connect(tcp_server, &TcpServer::closeSignal, topology, &Topology::slotCloseSignal);

    for (auto signal : topology->getSignalsData()->exit_signals)
    {
        connect(signal, &Signal::sendDataUpdate, tcp_server, &TcpServer::slotUpdateSignal);
    }

    Journal::instance()->info("TCP server is initialized successfully");
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
void Model::tcpFeedBack()
{
    if (tcp_simulator_update.vehicles.empty())
    {
        return;
    }

    tcp_simulator_update.time = t;
    tcp_simulator_update.current_vehicle = current_vehicle;
    tcp_simulator_update.controlled_vehicle = controlled_vehicle;

    tcp_simulator_update.trains.resize(trains.size());
    int i = 0;
    for (auto train : trains)
    {
        tcp_simulator_update.trains[i].first_vehicle_id = train->getFirstVehicle()->getModelIndex();
        tcp_simulator_update.trains[i].last_vehicle_id = train->getLastVehicle()->getModelIndex();

        ++i;
    }

    i = 0;
    for (auto vehicle : vehicles)
    {
        profile_point_t *pp = vehicle->getProfilePoint();

        tcp_simulator_update.vehicles[i].position_x = pp->position.x;
        tcp_simulator_update.vehicles[i].position_y = pp->position.y;
        tcp_simulator_update.vehicles[i].position_z = pp->position.z;
        tcp_simulator_update.vehicles[i].orth_x = pp->orth.x;
        tcp_simulator_update.vehicles[i].orth_y = pp->orth.y;
        tcp_simulator_update.vehicles[i].orth_z = pp->orth.z;
        tcp_simulator_update.vehicles[i].up_x = pp->up.x;
        tcp_simulator_update.vehicles[i].up_y = pp->up.y;
        tcp_simulator_update.vehicles[i].up_z = pp->up.z;

        tcp_simulator_update.vehicles[i].train_id = vehicle->getTrainIndex();
        int orient = vehicle->getOrientation();
        tcp_simulator_update.vehicles[i].orientation = orient;
        if (orient == -1)
        {
            tcp_simulator_update.vehicles[i].next_vehicle =
                (vehicle->getPrevVehicle() == nullptr) ?
                    -1 :
                    vehicle->getPrevVehicle()->getModelIndex();

            tcp_simulator_update.vehicles[i].prev_vehicle =
                (vehicle->getNextVehicle() == nullptr) ?
                    -1 :
                    vehicle->getNextVehicle()->getModelIndex();
        }
        else
        {
            tcp_simulator_update.vehicles[i].next_vehicle =
                (vehicle->getNextVehicle() == nullptr) ?
                    -1 :
                    vehicle->getNextVehicle()->getModelIndex();

            tcp_simulator_update.vehicles[i].prev_vehicle =
                (vehicle->getPrevVehicle() == nullptr) ?
                    -1 :
                    vehicle->getPrevVehicle()->getModelIndex();
        }

        tcp_simulator_update.vehicles[i].length = vehicle->getLength();

        tcp_simulator_update.vehicles[i].analogSignal = vehicle->getAnalogSignals();

        if (current_vehicle == i)
        {
            tcp_simulator_update.currentDebugMsg = vehicle->getDebugMsg();
        }

        if (controlled_vehicle == i)
        {
            tcp_simulator_update.controlledDebugMeg = vehicle->getDebugMsg();
        }

        ++i;
    }

    tcp_server->setSimulatorData(tcp_simulator_update.serialize());
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
void Model::virtualRailwayFeedback()
{
    /*if (sim_client == Q_NULLPTR)
        return;

    if (!sim_client->isConnected())
        return;

    sim_dispatcher_data_t disp_data;
    emit getRecvData(disp_data);

    alsn_info_t alsn_info;
    alsn_info.code_alsn = disp_data.code_alsn;
    alsn_info.num_free_block = disp_data.num_free_block;
    alsn_info.response_code = disp_data.response_code;
    alsn_info.signal_dist = disp_data.signal_dist;
    strcpy(alsn_info.current_time, disp_data.current_time);

    train->getFirstVehicle()->setASLN(alsn_info);

    sim_train_data_t train_data;
    strcpy(train_data.train_id, train->getTrainID().toStdString().c_str());
    train_data.direction = train->getDirection();
    train_data.coord = train->getFirstVehicle()->getRailwayCoord();
    train_data.speed = train->getFirstVehicle()->getVelocity();

    sim_client->sendTrainData(train_data);*/
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
void Model::sharedMemoryFeedback()
{
    update_data.time = t;
    update_data.current_vehicle = current_vehicle;
    update_data.controlled_vehicle = controlled_vehicle;

    update_data.num_trains = trains.size();
    int i = 0;
    for (auto train : trains)
    {
        update_data.trains[i].first_vehicle_id = train->getFirstVehicle()->getModelIndex();
        update_data.trains[i].last_vehicle_id = train->getLastVehicle()->getModelIndex();

        ++i;
    }

    i = 0;
    for (auto vehicle : vehicles)
    {
        profile_point_t *pp = vehicle->getProfilePoint();

        update_data.vehicles[i].position_x = pp->position.x;
        update_data.vehicles[i].position_y = pp->position.y;
        update_data.vehicles[i].position_z = pp->position.z;
        update_data.vehicles[i].orth_x = pp->orth.x;
        update_data.vehicles[i].orth_y = pp->orth.y;
        update_data.vehicles[i].orth_z = pp->orth.z;
        update_data.vehicles[i].up_x = pp->up.x;
        update_data.vehicles[i].up_y = pp->up.y;
        update_data.vehicles[i].up_z = pp->up.z;

        update_data.vehicles[i].train_id = vehicle->getTrainIndex();
        int orient = vehicle->getOrientation();
        update_data.vehicles[i].orientation = orient;
        if (orient == -1)
        {
            update_data.vehicles[i].next_vehicle =
                (vehicle->getPrevVehicle() == nullptr) ?
                    -1 :
                    vehicle->getPrevVehicle()->getModelIndex();

            update_data.vehicles[i].prev_vehicle =
                (vehicle->getNextVehicle() == nullptr) ?
                    -1 :
                    vehicle->getNextVehicle()->getModelIndex();
        }
        else
        {
            update_data.vehicles[i].next_vehicle =
                (vehicle->getNextVehicle() == nullptr) ?
                    -1 :
                    vehicle->getNextVehicle()->getModelIndex();

            update_data.vehicles[i].prev_vehicle =
                (vehicle->getPrevVehicle() == nullptr) ?
                    -1 :
                    vehicle->getPrevVehicle()->getModelIndex();
        }

        std::copy(vehicle->getAnalogSignals().begin(),
                  vehicle->getAnalogSignals().end(),
                  update_data.vehicles[i].analogSignal.begin());

        if (current_vehicle == i)
        {
            QString msg = vehicle->getDebugMsg();
            msg.resize(DEBUG_STRING_SIZE, QChar(' '));
            msg.toWCharArray(update_data.currentDebugMsg);
        }

        if (controlled_vehicle == i)
        {
            if (data.size() != 0)
                vehicle->setKeysData(data);

            QString msg = vehicle->getDebugMsg();
            msg.resize(DEBUG_STRING_SIZE, QChar(' '));
            msg.toWCharArray(update_data.controlledDebugMsg);
        }
        else
        {
            if (prev_controlled_vehicle == i)
            {
                vehicle->resetKeysData();
                prev_controlled_vehicle = controlled_vehicle;
            }
        }
        ++i;
    }

    if (memory_sim_update.lock())
    {
        memcpy(memory_sim_update.data(), &update_data, sizeof (simulator_update_t));
        memory_sim_update.unlock();
    }
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
void Model::controlStep(double &control_time, const double control_delay)
{
    if (control_time >= control_delay)
    {
        control_time = 0;

        if (memory_controlled.lock())
        {
            controlled_t *c = static_cast<controlled_t *>(memory_controlled.data());

            if (c == nullptr)
            {
                memory_controlled.unlock();
                return;
            }

            current_vehicle = c->current_vehicle;
            controlled_vehicle = c->controlled_vehicle;

            memory_controlled.unlock();
        }

        if (keys_data.lock())
        {
            data.resize(keys_data.size());
            memcpy(data.data(), keys_data.data(), static_cast<size_t>(keys_data.size()));

            keys_data.unlock();
        }
    }

    control_time += dt;
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
void Model::process()
{
    double integration_time = static_cast<double>(integration_time_interval) / 1000.0;
    tau = tau - integration_time;

    topology->step(t, integration_time);

    findNearestVehicles();

    findFarthestVehicles();

    // Integrate all ODE in train motion model
    do
    {
        preStep(t);

        controlStep(control_time, control_delay);

        is_step_correct = step(t, dt);

        tau += dt;
        t += dt;

        postStep(t);
    }
    while ( (tau < 0.0) && is_step_correct );

    // Feedback to viewer
    sharedMemoryFeedback();

    tcpFeedBack();

    for (auto train : trains)
        train->inputProcess();

    // Debug print, is allowed
    if (is_debug_print)
        debugPrint();
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
void Model::slotGetTopologyData(QByteArray &topology_data)
{
    topology_data = topology->serialize();
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
void Model::slotGetSignalsData(QByteArray &signals_data)
{
    signals_data = topology->getSignalsData()->serialize();
}
