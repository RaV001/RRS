#ifndef     CONNECTOR_H
#define     CONNECTOR_H

#include    <QObject>

#include    <trajectory.h>
#include    <topology-types.h>
#include    <CfgReader.h>

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
class Connector : public QObject
{
public:

    Connector(QObject *parent);

    virtual ~Connector();

    virtual Trajectory *getFwdTraj() const
    {
        return fwdTraj;
    }

    virtual Trajectory *getBwdTraj() const
    {
        return bwdTraj;
    }

    virtual void setState(int state)
    {
        this->state = state;
    }

    virtual void load_config(CfgReader &cfg,
                             QDomNode secNode,
                             traj_list_t &traj_list);

protected:

    Trajectory *fwdTraj = Q_NULLPTR;

    Trajectory *bwdTraj = Q_NULLPTR;

    int state = 0;

    QString name = "";
};

#endif
