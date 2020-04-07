#include    "tep70.h"

#include    <CfgReader.h>

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
TEP70::TEP70() : Vehicle()
  , km(nullptr)
  , battery(nullptr)
  , kontaktor_fuel_pump(nullptr)
  , fuel_tank(nullptr)
  , electro_fuel_pump(nullptr)
  , disel(nullptr)
  , ru8(nullptr)
  , kontaktor_oil_pump(nullptr)
  , oilpump_time_relay(nullptr)
  , starter_time_relay(nullptr)
  , electro_oil_pump(nullptr)
  , starter_generator(nullptr)
  , kontaktor_starter(nullptr)
  , ru10(nullptr)
  , ru6(nullptr)
  , ru42(nullptr)
  , ru7(nullptr)
  , ru15(nullptr)
  , mv6(nullptr)
  , vtn(nullptr)
  , ru4(nullptr)
  , rv4(nullptr)
  , rv9(nullptr)
  , krn(nullptr)
  , voltage_regulator(nullptr)
  , main_reservoir(nullptr)
  , motor_compressor(nullptr)
  , press_reg(nullptr)
  , ru18(nullptr)
  , ktk1(nullptr)
  , ktk2(nullptr)
  , rv6(nullptr)
  , ubt367m(nullptr)
  , krm(nullptr)
  , kvt(nullptr)
  , vr(nullptr)
  , evr(nullptr)
  , zpk(nullptr)
  , rd304(nullptr)
  , pneumo_splitter(nullptr)
  , fwd_trolley(nullptr)
  , bwd_trolley(nullptr)
  , zr(nullptr)
  , epk(nullptr)
  , ept_converter(nullptr)
  , ept_pass_control(nullptr)
  , button_disel_start(false)
  , button_brake_release(false)
  , button_svistok(false)
  , button_tifon(false)
  , button_RB1(false)
  , Ucc(0.0)
  , Icc(0.0)
  , charge_press(0.5)
{

}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
TEP70::~TEP70()
{

}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
void TEP70::initBrakeDevices(double p0, double pTM, double pFL)
{
    main_reservoir->setY(0, pFL);
    charge_press = p0;

    krm->setChargePressure(charge_press);
    krm->init(pTM, pFL);
    kvt->init(pTM, pFL);
    vr->init(pTM, pFL);
    zr->init(pTM, pFL);

    ubt367m->setState(1);
    ubt367m->setCombineCranePos(0);
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
void TEP70::initialization()
{
    initCabineControls();

    initControlCircuit();

    initFuelSystem();

    initDisel();

    initOilSystem();

    initPneumoBrakeSystem();

    initEPT();

    initSounds();
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
void TEP70::step(double t, double dt)
{
    Q_UNUSED(t)
    Q_UNUSED(dt)

    stepCabineControls(t, dt);

    stepControlCircuit(t, dt);

    stepFuelSystem(t, dt);

    stepDisel(t, dt);

    stepOilSystem(t, dt);

    stepPneumoBrakeSystem(t, dt);

    stepEPT(t, dt);

    stepSignalsOutput(t, dt);

    debugOutput(t, dt);    
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
void TEP70::loadConfig(QString cfg_path)
{
    CfgReader cfg;

    if (cfg.load(cfg_path))
    {
        QString secName = "Vehicle";

        double fuel_capacity = 0;
        cfg.getDouble(secName, "FuelCapacity", fuel_capacity);

        double fuel_level = 0;
        cfg.getDouble(secName, "FuelLevel", fuel_level);

        fuel_tank = new FuelTank();
        fuel_tank->setCapacity(fuel_capacity);
        fuel_tank->setFuelLevel(fuel_level);
    }
}

GET_VEHICLE(TEP70)
