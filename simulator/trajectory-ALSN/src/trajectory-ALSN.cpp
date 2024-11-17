#include    "trajectory-ALSN.h"
#include    "ALSN-coil.h"
#include    "topology-connector-device.h"
#include    "trajectory.h"
#include    "connector.h"

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
TrajectoryALSN::TrajectoryALSN(QObject *parent) : TrajectoryDevice(parent)
{
    name = QString("ALSN");
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
TrajectoryALSN::~TrajectoryALSN()
{

}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
void TrajectoryALSN::step(double t, double dt)
{
    (void) t;
    (void) dt;

    if (vehicles_devices.empty())
    {
        clear_code();
        return;
    }

    // Координаты занятого участка траектории
    // (от начала первой ПЕ до конца последней ПЕ);
    // между ними сигнала АЛСН нет,
    // так как он зашунтирован колёсными парами ПЕ
    double busy_begin_coord;
    double busy_end_coord;
    trajectory->getBusyCoords(busy_end_coord, busy_begin_coord);

    // Задаём приёмным катушкам информацию о следующем светофоре,
    // а возле начала и конца занятого участка - и код АЛСН
    for (auto device : vehicles_devices)
    {
        if (device.device->getOutputSignal(CoilALSN::OUTPUT_DIRECTION) == 1.0)
        {
            // Литер следующего светофора
            size_t liter_size = min(static_cast<size_t>(next_liter_fwd.size()),
                                    static_cast<size_t>(CoilALSN::INPUT_LITER_MAX_SIZE));
            device.device->setInputSignal(CoilALSN::INPUT_LITER_SIZE,
                                          static_cast<double>(liter_size));
            if (liter_size > 0)
            {
                for (size_t i = 0; i < liter_size; ++i)
                {
                    device.device->setInputSignal(CoilALSN::INPUT_LITER_BEGIN + i,
                                                  static_cast<double>(next_liter_fwd.at(i).unicode()));
                }

                // Расстояние до следующего светофора, м
                device.device->setInputSignal(CoilALSN::INPUT_NEXT_DISTANCE,
                                             distance_fwd + (trajectory->getLength() - device.coord));
            }
            else
            {
                // Если следующий светофор неизвестен, неизвестно и расстояние
                device.device->setInputSignal(CoilALSN::INPUT_NEXT_DISTANCE, 0.0);
            }

            // Проверяем координату с запасом в 1 метр
            if ((busy_begin_coord - device.coord) < 1.0)
            {
                // Несущая частота сигнала, Гц
                device.device->setInputSignal(CoilALSN::INPUT_FREQUENCY, frequency);
                // Кодовый сигнал
                device.device->setInputSignal(CoilALSN::INPUT_CODE, static_cast<double>(code_from_fwd));
            }
            else
            {
                // Сигнал отсутствует
                device.device->setInputSignal(CoilALSN::INPUT_FREQUENCY, 0.0);
                device.device->setInputSignal(CoilALSN::INPUT_CODE, 0.0);
            }
        }
        if (device.device->getOutputSignal(CoilALSN::OUTPUT_DIRECTION) == -1.0)
        {
            // Литер следующего светофора
            size_t liter_size = min(static_cast<size_t>(next_liter_bwd.size()),
                                    static_cast<size_t>(CoilALSN::INPUT_LITER_MAX_SIZE));
            device.device->setInputSignal(CoilALSN::INPUT_LITER_SIZE,
                                        static_cast<double>(liter_size));
            if (liter_size > 0)
            {
                for (size_t i = 0; i < liter_size; ++i)
                {
                    device.device->setInputSignal(CoilALSN::INPUT_LITER_BEGIN + i,
                                                static_cast<double>(next_liter_bwd.at(i).unicode()));
                }

                // Расстояние до следующего светофора, м
                device.device->setInputSignal(CoilALSN::INPUT_NEXT_DISTANCE,
                                            distance_bwd + device.coord);
            }
            else
            {
                // Если следующий светофор неизвестен, неизвестно и расстояние
                device.device->setInputSignal(CoilALSN::INPUT_NEXT_DISTANCE, 0.0);
            }

            // Проверяем координату с запасом в 1 метр
            if ((busy_end_coord - device.coord) > -1.0)
            {
                // Несущая частота сигнала, Гц
                device.device->setInputSignal(CoilALSN::INPUT_FREQUENCY, frequency);
                // Кодовый сигнал
                device.device->setInputSignal(CoilALSN::INPUT_CODE, static_cast<double>(code_from_bwd));
            }
            else
            {
                // Сигнал отсутствует
                device.device->setInputSignal(CoilALSN::INPUT_FREQUENCY, 0.0);
                device.device->setInputSignal(CoilALSN::INPUT_CODE, 0.0);
            }
        }
    }

    clear_code();
    return;
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
void TrajectoryALSN::setSignalInfoFwd(ALSN code, double distance, QString liter)
{
    if (frequency == 0.0)
    {
        code_from_fwd = ALSN::NO_CODE;
    }
    else
    {
        code_from_fwd = code;
    }

    distance_fwd = distance;
    next_liter_fwd = liter;

    ALSN code_to_next = code_from_fwd;

    // Если траектория занята, дальше код не проходит
    if (trajectory->isBusy())
        code_to_next = ALSN::NO_CODE;

    // Переход к рельсовым цепям предыдущей траектории
    // Модуль коннектора к предыдущей траектории
    auto conn_device = getBwdConnectorDevice();
    if (conn_device == nullptr)
        return;

    // Проверяем стрелку на взрез
    Connector *conn = conn_device->getConnector();
    if (conn->getFwdTraj() != trajectory)
        return;

    // Предыдущая траектория
    TrajectoryALSN *traj_ALSN = dynamic_cast<TrajectoryALSN *>(
        conn_device->getBwdTrajectoryDevice());
    if (traj_ALSN == nullptr)
        return;

    // Передаём информацию дальше
    traj_ALSN->setSignalInfoFwd(code_to_next, distance + trajectory->getLength(), liter);
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
void TrajectoryALSN::setSignalInfoBwd(ALSN code, double distance, QString liter)
{
    if (frequency == 0.0)
    {
        code_from_bwd = ALSN::NO_CODE;
    }
    else
    {
        code_from_bwd = code;
    }

    distance_bwd = distance;
    next_liter_bwd = liter;

    ALSN code_to_next = code_from_bwd;

    // Если траектория занята, дальше код не проходит
    if (trajectory->isBusy())
        code_to_next = ALSN::NO_CODE;

    // Переход к рельсовым цепям следующей траектории
    // Модуль коннектора к следующей траектории
    auto conn_device = getFwdConnectorDevice();
    if (conn_device == nullptr)
        return;

    // Проверяем стрелку на взрез
    Connector *conn = conn_device->getConnector();
    if (conn->getBwdTraj() != trajectory)
        return;

    // Следующая траектория
    TrajectoryALSN *traj_ALSN = dynamic_cast<TrajectoryALSN *>(
        conn_device->getFwdTrajectoryDevice());
    if (traj_ALSN == nullptr)
        return;

    // Передаём информацию дальше
    traj_ALSN->setSignalInfoBwd(code_to_next, distance + trajectory->getLength(), liter);
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
void TrajectoryALSN::load_config(CfgReader &cfg)
{
    cfg.getDouble("ALSN", "Frequency", frequency);
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
void TrajectoryALSN::clear_code()
{
    // Очистка
    code_from_fwd = ALSN::NO_CODE;
    distance_fwd = 0.0;
    next_liter_fwd = "";
    code_from_bwd = ALSN::NO_CODE;
    distance_bwd = 0.0;
    next_liter_bwd = "";
}

GET_TRAJECTORY_DEVICE(TrajectoryALSN)
