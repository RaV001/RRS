#ifndef     ACTIVE_TRAIN_H
#define     ACTIVE_TRAIN_H

#include    <train-info.h>
#include    <waypoint.h>

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
struct active_train_t
{
    train_info_t train_info;
    train_position_t train_position;

    active_train_t()
    {

    }
};

#endif
