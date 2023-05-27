#ifndef     AUTOMATIC_TRAIN_STOP_H
#define     AUTOMATIC_TRAIN_STOP_H

#include    "brake-device.h"

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
class DEVICE_EXPORT AutoTrainStop : public BrakeDevice
{
public:

    AutoTrainStop(QObject *parent = Q_NULLPTR);

    virtual ~AutoTrainStop();

    void powerOn(bool on);

    void keyOn(bool on);

    bool getStateKey() const;

    /// Задать давление от питательной магистрали
    void setFLpressure(double value);

    /// Поток в питательную магистраль
    double getFLflow() const;

    /// Задать давление от тормозной магистрали
    void setBPpressure(double value);

    /// Поток в тормозную магистраль
    double getBPflow() const;

    /// Автостопное экстренное торможение
    virtual bool getEmergencyBrakeContact() const;

protected:

    double is_powered;

    double is_key_on;

    double pFL;
    double pBP;

    double QFL;
    double QBP;
};

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
typedef AutoTrainStop* (*GetAutoTrainStop)();

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
#define GET_AUTO_TRAIN_STOP(ClassName) \
    extern "C" Q_DECL_EXPORT AutoTrainStop *getAutoTrainStop() \
    { \
        return new (ClassName) (); \
    }

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
extern "C" Q_DECL_EXPORT AutoTrainStop *loadAutoTrainStop(QString lib_path);

#endif // AUTOMATIC_TRAIN_STOP_H
