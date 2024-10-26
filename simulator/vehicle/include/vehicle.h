//------------------------------------------------------------------------------
//
//      Vehicle base class
//      (c) maisvendoo, 03/09/2018
//      Developer: Dmitry Pritykin
//
//------------------------------------------------------------------------------
/*!
 * \file
 * \brief  Vehicle base class
 * \copyright maisvendoo
 * \author Dmitry Pritykin
 * \date 03/09/2018
 */

#ifndef     VEHICLE_H
#define     VEHICLE_H

#include    <QObject>
#include    <QtGlobal>
#include    <QMap>
#include    <QMutex>

#include    "solver-types.h"
#include    "key-symbols.h"

#include    "vehicle-signals.h"
#include    "control-signals.h"
#include    "feedback-signals.h"

#include    "profile-point.h"
#include    "device-list.h"

#if defined(VEHICLE_LIB)
    #define VEHICLE_EXPORT  Q_DECL_EXPORT
#else
    #define VEHICLE_EXPORT  Q_DECL_IMPORT
#endif

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
class VEHICLE_EXPORT Vehicle : public QObject
{
    Q_OBJECT

public:

    /// Constructor
    explicit Vehicle(QObject *parent = Q_NULLPTR);
    /// Destructor
    virtual ~Vehicle();

    /// Vehicle initialization
    void init(QString cfg_path);

    /// Set vehicle module directory
    void setModuleDir(QString module_dir);
    /// Set Vehicle module name
    void setModuleName(QString module_name);
    /// Set vehicle configuration file directory
    void setConfigDir(QString config_dir);
    /// Set Vehicle configuration file name
    void setConfigName(QString config_name);
    /// Set current route directory
    void setRouteDir(QString route_dir);

    /// Set vehicle index
    void setModelIndex(size_t idx);

    /// Set vehicle state index
    void setStateIndex(size_t idx);

    /// Set inclination
    void setProfilePoint(profile_point_t point_data);

    /// Set friction coefficient between wheel and rail
    void setFrictionCoeff(double value);

    /// Set direction
    void setDirection(int dir);

    /// Set orientation
    void setOrientation(int orient);

    /// Set forward coupling force
    void addForwardForce(double value);

    /// Set backward coupling force
    void addBackwardForce(double value);

    /// Set active common force
    void setActiveCommonForce(size_t idx, double value);

    /// Set reactive common force
    void setReactiveCommonForce(size_t idx, double value);

    /// Set payload level
    void setPayloadCoeff(double payload_coeff);

    void setTrainCoord(double value);

    void setVelocity(double value);

    void setWheelAngle(size_t i, double value);
    void setWheelOmega(size_t i, double value);

    void setPrevVehicle(Vehicle *vehicle);
    void setNextVehicle(Vehicle *vehicle);

    /// Get vehicle module directory
    QString getModuleDir() const;
    /// Get Vehicle module name
    QString getModuleName() const;
    /// Get vehicle configuration file directory
    QString getConfigDir() const;
    /// Get Vehicle configuration file name
    QString getConfigName() const;

    /// Get vehicle index
    size_t getModelIndex() const;

    /// Get vehicle index
    size_t getStateIndex() const;

    profile_point_t *getProfilePoint();

    /// Get orientation
    int getOrientation() const;

    /// Get vehicle mass
    double getMass() const;

    /// Get vehicle length
    double getLength() const;

    /// Get degrees of freedom
    size_t getDegressOfFreedom() const;

    /// Get number of axis
    size_t getNumAxis() const;

    /// Get wheel diameter
    double getWheelDiameter(size_t i) const;

    double getTrainCoord() const;

    double getVelocity() const;

    double getWheelAngle(size_t i);

    double getWheelOmega(size_t i);

    Vehicle *getPrevVehicle();
    Vehicle *getNextVehicle();

    float getAnalogSignal(size_t i);
    std::array<float, MAX_ANALOG_SIGNALS> getAnalogSignals();

    device_list_t *getFwdConnectors();
    device_list_t *getBwdConnectors();
    device_coord_list_t *getRailwayConnectors();

    /// Common acceleration calculation
    virtual state_vector_t getAcceleration(state_vector_t &Y, double t, double dt);

    ///
    void integrationPreStep(state_vector_t &Y, double t);

    virtual void keyProcess();

    virtual void hardwareProcess();

    ///
    void integrationStep(state_vector_t &Y, double t, double dt);

    ///
    void integrationPostStep(state_vector_t &Y, double t);

    QString getDebugMsg() const;

    /// Init vehicle brake devices
    virtual void initBrakeDevices(double p0, double pTM, double pFL);

    void setUks(double value);

    void setCurrentKind(int value);

    void setKeysData(QByteArray data);

    void resetKeysData();

public slots:

    void getControlSignals(control_signals_t control_signals);

signals:

    void sendFeedBackSignals(feedback_signals_t feedback_signals);

protected:

    /// Vehicle configuration file directory
    QString module_dir = "";
    /// Vehicle configuration file name
    QString module_name = "";
    /// Vehicle configuration file directory
    QString config_dir = "";
    /// Vehicle configuration file name
    QString config_name = "";
    /// Current route directory
    QString route_dir = "";

    /// Vehicle index
    size_t  model_idx = 0;

    /// Vehicle ODE system index
    size_t  state_idx = 0;

    /// Empty vehicle mass (without payload)
    double  empty_mass = 25000.0;
    /// Full payload mass
    double  payload_mass = 65000.0;
    /// Payload coefficient (0.0 - empty, 1.0 - full payload)
    double  payload_coeff = 0.0;
    /// Full vehicle mass
    double  full_mass = 25000.0;
    /// Length between coupling's axis
    double  length = 13.92;

    // Main resistant's coefficients
    double  b0 = 0.7;
    double  b1 = 8.0;
    double  b2 = 0.08;
    double  b3 = 0.002;
    double  q0 = full_mass / 4.0;
    double  W_coef = (b0 + b1 / q0) * Physics::g / 1000.0;
    double  W_coef_v = (b2 / q0) * Physics::g * Physics::kmh / 1000.0;
    double  W_coef_v2 = (b3 / q0) * Physics::g * Physics::kmh * Physics::kmh / 1000.0;
    double  W_coef_curv = 700.0 * Physics::g / 1000.0;

    // Wheels model's coefficients
    double  psi_a = 0.0;
    double  psi_b = 30.0;
    double  psi_c = 100.0;
    double  psi_d = 1.0;
    double  psi_e = 0.0;

    /// Numder of axis
    size_t              num_axis = 4;
    /// Wheels rotation angles
    std::vector<double> wheel_rotation_angle = {0.0, 0.0, 0.0, 0.0};
    /// Wheels angular velocities
    std::vector<double> wheel_omega = {0.0, 0.0, 0.0, 0.0};
    /// Wheel diameter
    std::vector<double> wheel_diameter = {0.95, 0.95, 0.95, 0.95};
    /// Wheel radius
    std::vector<double> rk = {0.475, 0.475, 0.475, 0.475};
    /// Axis moment of inertia
    std::vector<double> J_axis = {100.0, 100.0, 100.0, 100.0};
    /// Vertical axis load
    std::vector<double> axis_load = {full_mass/4.0, full_mass/4.0, full_mass/4.0, full_mass/4.0};
    /// Friction coefficient between wheel and rail
    std::vector<double> psi = {psi_a+psi_b/psi_c, psi_a+psi_b/psi_c, psi_a+psi_b/psi_c, psi_a+psi_b/psi_c};
    /// Friction coefficient changing
    double              psi_coeff = 1.0;

    /// Forward coupling force
    double  F_fwd = 0.0;
    /// Backward coupling force
    double  F_bwd = 0.0;
    /// Gravity force from profile inclination
    double  F_g = 0.0;

    /// Number of degrees of freedom
    size_t  s = num_axis + 1;

    /// Train coordinate
    double train_coord = 0.0;
    /// Body velocity
    double velocity = 0.0;

    profile_point_t profile_point_data = profile_point_t();

    /// Railway motion direction
    int     dir = 1;
    /// Vehicle orientation
    int     orient = 1;

    QString DebugMsg = "";

    Vehicle *prev_vehicle = nullptr;
    Vehicle *next_vehicle = nullptr;

    /// Напряжение в КС
    double      Uks = 25000.0;

    /// Род тока в КС
    int         current_kind = 1;

    /// Active common forces
    state_vector_t  Q_a = {0.0, 0.0, 0.0, 0.0, 0.0};
    /// Reactive common forces
    state_vector_t  Q_r = {0.0, 0.0, 0.0, 0.0, 0.0};
    /// Vehicle common acceleration
    state_vector_t  acceleration = {0.0, 0.0, 0.0, 0.0, 0.0};

    /// Keyboard state
    QMap<int, bool> keys;
    QMutex          keys_mutex;

    /// Analog signals for output
    std::array<float, MAX_ANALOG_SIGNALS>   analogSignal;

    /// List of devices - forward connectors
    device_list_t forward_connectors;
    /// List of devices - backward connectors
    device_list_t backward_connectors;
    /// List of devices - railway connectors
    device_coord_list_t railway_connectors;

    control_signals_t   control_signals;

    feedback_signals_t  feedback_signals;

    /// User defined initialization
    virtual void initialization();

    /// User defined configuration load
    virtual void loadConfig(QString cfg_path);

    /// Add device to forward connectors
    void addFwdConnector(Device *device);
    /// Add device to backward connectors
    void addBwdConnector(Device *device);
    /// Add device to railway connectors
    void addRailwayConnector(Device *device, double distance_from_center = 0.0);

    /// User defined step prepare
    virtual void preStep(double t);

    /// Internal ODE integration step
    virtual void step(double t, double dt);

    /// User define step result processing
    virtual void postStep(double t);

    virtual void hardwareOutput();

    /// Recalculate coefficients for default main resistant formula
    virtual void mainResistCoeffs();

    /// Calculate main resistant to motion
    virtual double mainResist(double velocity);

    /// Calculate wheel-rail friction coefficient
    virtual double wheelrailFriction(double velocity);

    /* Modkeys extended functions */

    bool isShift() const;

    bool isControl() const;

    bool isAlt() const;

    bool getKeyState(int key) const;

private:

    /// Default configuration load
    void loadConfiguration(QString cfg_path);

    /// Load main resistence coefficients
    void loadMainResist(QString cfg_path, QString main_resist_cfg);
    /// Load wheel-rail friction coefficients
    void loadWheelRailFriction(QString cfg_path, QString wheel_rail_friction_cfg);
};

/*!
 * \typedef
 * \brief Signature for getVehicle() function
 */
//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
typedef Vehicle* (*GetVehicle)();

/*!
 * \def
 * \brief Macro for generate getVehicle() function
 */
//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
#define GET_VEHICLE(ClassName) \
    extern "C" Q_DECL_EXPORT Vehicle *getVehicle() \
    {\
        return new (ClassName)(); \
    }

/*!
 * \fn
 * \brief Load vehicle from library
 */
//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
extern "C" Q_DECL_EXPORT Vehicle *loadVehicle(QString lib_path);

#endif // VEHICLE_H
