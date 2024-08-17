#ifndef     TOPOLOGY_TYPES_H
#define     TOPOLOGY_TYPES_H

#include    <QString>
#include    <QMap>

#include    <trajectory.h>

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
using traj_list_t = QMap<QString, Trajectory *>;

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
using conn_list_t = QMap<QString, Connector *>;

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
struct topology_pos_t
{
    QString     traj_name = "";
    double      traj_coord = 0.0;
    int         dir = 1;

    topology_pos_t()
    {

    }
};

#endif