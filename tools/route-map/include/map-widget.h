#ifndef     MAP_WIDGET_H
#define     MAP_WIDGET_H

#include    <QMenu>
#include    <QMouseEvent>
#include    <QMap>
#include    <topology-types.h>
#include    <trajectory.h>
#include    <simulator-update-struct.h>
#include    <switch-label.h>
#include    <signals-data-types.h>
#include    <signal-label.h>

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
class MapWidget : public QWidget
{
    Q_OBJECT

public:

    MapWidget(QWidget *parent = Q_NULLPTR);

    ~MapWidget();

    traj_list_t *traj_list = Q_NULLPTR;

    conn_list_t *conn_list = Q_NULLPTR;

    tcp_simulator_update_t *train_data = Q_NULLPTR;

    topology_stations_list_t *stations = Q_NULLPTR;

    QMap<QString , SwitchLabel *> switch_labels;

    signals_data_t *signals_data = Q_NULLPTR;

    QMap<QString, SignalLabel *> signal_labels_fwd;

    QMap<QString, SignalLabel *> signal_labels_bwd;

    void resize(int width, int height);

    void setSwitchLength(double value);

    void setSignalRadius(double value);

    void SetSignalOffset(double value);

    double getScale() const
    {
        return scale;
    }

    QPoint getMousePos() const
    {
        return mouse_pos;
    }

private:

    /// Масштаб отображения карты
    double scale = 1.0;

    /// Шаг увеличения масштаба
    double scale_inc_step_coeff = sqrt(2.0);
    /// Шаг уменьшения масштаба
    double scale_dec_step_coeff = 1.0 / sqrt(2.0);

    /// Текущее смещение координат
    QPoint map_shift;

    /// Положение курсора в момент последнего нажатия ЛКМ
    QPoint mouse_pos;

    /// Смещение координат до движения курсора с зажатой ЛКМ
    QPoint prev_map_shift;

    /// Перемещение вслед за ПЕ
    bool folow_vehicle = true;

    /// Длина отрисовки выбранной траектории стрелки, м
    double switch_length = 35.0;

    /// Радиус у схематичного отображения сигналов светофоров, м
    double signal_radius = 2.0;

    /// Смещение схематичного светофора вправо от оси пути, м
    double signal_offset = 2.5;

    void paintEvent(QPaintEvent *event);

    void drawTrajectory(Trajectory *traj);

    void drawTrain(tcp_simulator_update_t *train_data);

    void drawVehicle(simulator_vehicle_update_t &vehicle, QColor color);

    void drawConnectors(conn_list_t *conn_list);

    void drawConnector(Connector *conn);

    void drawStations(topology_stations_list_t *stations);

    void drawLineSignal(Signal *signal);

    void drawSignals(signals_data_t *signals_data);

    void drawEnterSignal(Signal *signal);

    void drawExitSignal(Signal *signal);

    QPoint coord_transform(dvec3 traj_point);

    void wheelEvent(QWheelEvent *event);

    void mouseMoveEvent(QMouseEvent *event);

    void mousePressEvent(QMouseEvent *event);

    void mouseReleaseEvent(QMouseEvent *event);
};

#endif
