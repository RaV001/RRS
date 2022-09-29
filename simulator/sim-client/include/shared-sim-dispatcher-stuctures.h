#ifndef SHARED_SIM_DISPATCHER_STRUCTURES_H
#define SHARED_SIM_DISPATCHER_STRUCTURES_H

#pragma pack(push, 1)
struct sim_dispatcher_data_t
{

public:

    short   code_alsn;
    int     num_free_block;
    int     response_code;
    char    current_time[9];
    char    signal_name[23];
    double  signal_dist;
    int     power_type;
    float   voltage;

    // Конструктор
    sim_dispatcher_data_t()
        : code_alsn(4)
        , num_free_block(0)
        , response_code(0)
        , current_time("00:00:00")
        , signal_name("")
        , signal_dist(0.0)
        , power_type(0)
        , voltage(0.0f)
    {

    }
};
#pragma pack(pop)

#pragma pack(push, 1)
struct sim_train_data_t
{

public:

    double  coord;
    int     direction;
    int     trainLength;
    double  speed;
    char    train_id[9];

    sim_train_data_t()
        : coord(0.0)
        , direction(0)
        , trainLength(0)
        , speed(0)
        , train_id("")        
    {

    }
};
#pragma pack(pop)

#endif // SHARED_SIM_DISPATCHER_STRUCTURES_H
