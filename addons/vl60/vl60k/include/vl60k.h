//------------------------------------------------------------------------------
//
//      Магистральный электровоз переменного тока ВЛ60.
//      Дополнение для Russian Railway Simulator (RRS)
//
//      (c) RRS development team:
//          Дмитрий Притыкин (maisvendoo),
//          Роман Бирюков (РомычРЖДУЗ)
//
//      Дата: 28/03/2019
//
//------------------------------------------------------------------------------
#ifndef     VL60K_H
#define     VL60K_H

#include    "vl60k-headers.h"

/*!
 * \class
 * \brief Основной класс, описывающий весь электровоз
 */
//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
class VL60k : public Vehicle
{
public:

    /// Конструктор
    VL60k();

    /// Деструктор
    ~VL60k();

    /// Инициализация тормозных приборов
    void initBrakeDevices(double p0, double pBP, double pFL);

private:

    enum
    {
        /// Объем главного резервуара (ГР), литров
        MAIN_RESERVOIR_VOLUME = 1200
    };

    float   pant1_pos = 0.0;
    float   pant2_pos = 0.0;
    float   gv_pos = 0.0;
    bool    gv_return = false;

    /// Зарядное давление
    double  charge_press = 0.0;

    /// Передаточное число редуктора
    double  ip = 3.83;

    /// Подключение рукавов магистрали тормозных цилиндров к импульсной магистрали
    bool bc_hose_to_impulse_line = true;

    /// Имя модуля сцепного устройства
    QString coupling_module_name = "sa3";
    /// Имя конфига сцепного устройства
    QString coupling_config_name = "sa3";
    /// Имя модуля поездного крана
    QString brake_crane_module_name = "krm395";
    /// Имя конфига поездного крана
    QString brake_crane_config_name = "krm395";
    /// Имя модуля локомотивного крана
    QString loco_crane_module_name = "kvt254";
    /// Имя конфига локомотивного крана
    QString loco_crane_config_name = "kvt254";
    /// Имя модуля воздухораспределителя
    QString airdist_module_name = "vr483";
    /// Имя конфига воздухорапределителя
    QString airdist_config_name = "vr483";

    /// Регистратор, для записи параметров
    Registrator *reg = nullptr;

    /// Сцепка спереди
    Coupling *coupling_fwd = Q_NULLPTR;
    /// Сцепка сзади
    Coupling *coupling_bwd = Q_NULLPTR;

    /// Расцепной рычаг спереди
    OperatingRod *oper_rod_fwd = Q_NULLPTR;
    /// Расцепной рычаг сзади
    OperatingRod *oper_rod_bwd = Q_NULLPTR;

    // Дальний ряд тумблеров приборной панели машиниста
//    /// Тригер тумблера "Прожектор яркий"
//    Trigger proj2_tumbler;
//    /// Тригер тумблера "Прожектор тусклый"
//    Trigger proj1_tumbler;
//    /// Тригер тумблера "Радиостанция"
//    Trigger radio_tumbler;
    /// Триггер тумблера "Цепи управления"
    Trigger cu_tumbler;
    /// Триггер тумблера "Токоприемник задний"
    Trigger pant2_tumbler;
    /// Триггер тумблера "Токоприемник передний"
    Trigger pant1_tumbler;
    /// Тригер тумблера "Токоприемники"
    Trigger pants_tumbler;
    /// Тригер тумблена "ГВ вкл. Возврат защиты"
    Trigger gv_return_tumbler;
    /// Триггер тумблера "ГВ вкл/выкл"
    Trigger gv_tumbler;

    // Ближний ряд тумблеров приборной панели машиниста
    enum
    {
        NUM_MOTOR_FANS = 6,
        MV1 = 0,
        MV2 = 1,
        MV3 = 2,
        MV4 = 3,
        MV5 = 4,
        MV6 = 5
    };

//    /// Тригер тумблера "Автоматическая подача песка"
//    Trigger autosand_tumbler;
    /// Триггеры тумблеров "Вентилятор 1-6"
    std::array<Trigger, NUM_MOTOR_FANS> mv_tumblers;
    /// Тригер тумблера "Компрессор"
    Trigger mk_tumbler;
    /// Тригер тумблера "Фазорасщепитель"
    Trigger fr_tumbler;

    enum
    {
        NUM_RB = 3,
        RB_1 = 0,
        RBP = 1,
        RBS = 2
    };

    /// Триггеры рукояток бдительности
    std::array<Trigger, NUM_RB>  rb;

    enum
    {
        NUM_PANTOGRAPHS = 2,
        WIRE_VOLTAGE = 25000
    };

    /// Токоприемники
    std::array<Pantograph *, NUM_PANTOGRAPHS>   pantographs;

    /// Главный выключатель (ГВ)
    ProtectiveDevice    *main_switch = Q_NULLPTR;

    /// Механизм киловольтметра КС
    Oscillator      *gauge_KV_ks = Q_NULLPTR;

    /// Тяговый трансформатор
    TracTransformer *trac_trans = Q_NULLPTR;

    /// Асинхронный расщепитель фаз
    PhaseSplitter   *phase_spliter = Q_NULLPTR;

    /// Мотор-вентиляторы
    std::array<ACMotorFan *, NUM_MOTOR_FANS> motor_fans;

    /// Мотор-компрессор
    ACMotorCompressor *motor_compressor = Q_NULLPTR;

    /// Регулятор давления в ГР
    PressureRegulator *press_reg = Q_NULLPTR;

    /// Главный резервуар
    Reservoir   *main_reservoir = Q_NULLPTR;

    /// Концевой кран питательной магистрали спереди
    PneumoAngleCock *anglecock_fl_fwd = Q_NULLPTR;

    /// Концевой кран питательной магистрали сзади
    PneumoAngleCock *anglecock_fl_bwd = Q_NULLPTR;

    /// Рукав питательной  магистрали спереди
    PneumoHose      *hose_fl_fwd = Q_NULLPTR;

    /// Рукав питательной  магистрали сзади
    PneumoHose      *hose_fl_bwd = Q_NULLPTR;

    /// Блокировочное устройство УБТ усл.№367м
    BrakeLock   *brake_lock = Q_NULLPTR;

    /// Поездной кран машиниста усл.№395
    BrakeCrane  *brake_crane = Q_NULLPTR;

    /// Кран впомогательного тормоза усл.№254
    LocoCrane   *loco_crane = Q_NULLPTR;

    /// Тормозная магистраль
    Reservoir   *brakepipe = Q_NULLPTR;

    /// Воздухораспределитель
    AirDistributor  *air_dist = Q_NULLPTR;

    /// Запасный резервуар
    Reservoir   *supply_reservoir = Q_NULLPTR;

    /// Концевой кран тормозной магистрали спереди
    PneumoAngleCock *anglecock_bp_fwd = Q_NULLPTR;

    /// Концевой кран тормозной магистрали сзади
    PneumoAngleCock *anglecock_bp_bwd = Q_NULLPTR;

    /// Рукав тормозной магистрали спереди
    PneumoHose  *hose_bp_fwd = Q_NULLPTR;

    /// Рукав тормозной магистрали сзади
    PneumoHose  *hose_bp_bwd = Q_NULLPTR;

    /// Импульсная магистраль с ложным тормозным цилиндром
    Reservoir  *impulse_line = Q_NULLPTR;

    /// Тройник подключения тележек к магистрали тормозных цилиндров
    PneumoSplitter  *bc_splitter = Q_NULLPTR;

    enum
    {
        NUM_TROLLEYS = 2,
        NUM_AXIS_PER_TROLLEY = 3,
        TROLLEY_FWD = 0,
        TROLLEY_BWD = 1
    };

    /// Тормозные механизмы тележек
    std::array<BrakeMech *, NUM_TROLLEYS> brake_mech;

    /// Концевой кран магистрали тормозных цилиндров спереди
    PneumoAngleCock  *anglecock_bc_fwd = Q_NULLPTR;

    /// Концевой кран магистрали тормозных цилиндров сзади
    PneumoAngleCock  *anglecock_bc_bwd = Q_NULLPTR;

    /// Рукав магистрали тормозных цилиндров спереди
    PneumoHose  *hose_bc_fwd = Q_NULLPTR;

    /// Рукав магистрали тормозных цилиндров сзади
    PneumoHose  *hose_bc_bwd = Q_NULLPTR;

    /// Контроллер машиниста
    ControllerKME_60_044    *controller = Q_NULLPTR;

    /// Главный контроллер (переключение обмоток тягового трансформатора)
    EKG_8G                  *main_controller = Q_NULLPTR;

    enum
    {
        NUM_VU = 2,
        VU1 = 0,
        VU2 = 1
    };

    /// Выпрямительные установки
    std::array<Rectifier *, NUM_VU> vu;

    /// Механизм киловольтметра ТЭД
    Oscillator  *gauge_KV_motors = Q_NULLPTR;

    enum
    {
        NUM_MOTORS = 6,
        TED1 = 0,
        TED2 = 1,
        TED3 = 2,
        TED4 = 3,
        TED5 = 4,
        TED6 = 5
    };

    /// Тяговые электродвигатели
    std::array<DCMotor *, NUM_MOTORS>  motor;

    /// Реле перегрузки ТЭД
    std::array<OverloadRelay *, NUM_MOTORS> overload_relay;

    /// Линейные контакторы ТЭД
    std::array<Trigger, NUM_MOTORS> line_contactor;

    /// Ограничения скорости на путевой инфраструктуре для кабины А
    SpeedMap    *speedmap_fwd = Q_NULLPTR;
    /// Ограничения скорости на путевой инфраструктуре для кабины Б
    SpeedMap    *speedmap_bwd = Q_NULLPTR;

    /// Приёмная катушка АЛСН для кабины А
    CoilALSN    *coil_ALSN_fwd = Q_NULLPTR;
    /// Приёмная катушка АЛСН для кабины Б
    CoilALSN    *coil_ALSN_bwd = Q_NULLPTR;

    /// Локомотивный скоростемер
    SL2M    *speed_meter = Q_NULLPTR;

    /// Свисток и тифон
    TrainHorn   *horn = Q_NULLPTR;

    /// Система подачи песка
    SandingSystem   *sand_system = Q_NULLPTR;

    std::vector<Trigger *> triggers;
    Timer   *autoStartTimer = Q_NULLPTR;
    size_t  start_count = 0;

    /// Устройство безопасности УКБМ
    SafetyDevice *safety_device = Q_NULLPTR;

    /// Электропневматический клапан автостопа
    AutoTrainStop *epk = Q_NULLPTR;

    /// Ключ ЭПК
    Trigger key_epk;

    enum
    {
        NUM_LC_CONTACTS = 3,
        LC_SELF = 0,
        LC_TED = 1,
        LC_TED_LAMP = 2
    };

    /// Линейные контакторы ТЭД
    std::array<Relay *, NUM_MOTORS> linear_contactor;

    double U_bat = 55.0;

    DecoderALSN *alsn_decoder = new DecoderALSN;

    /// Общая инициализация локомотива
    void initialization();

    /// Инициализация сцепных устройств
    void initCouplings(const QString &modules_dir, const QString &custom_cfg_dir);

    /// Инициализация токоприемников
    void initPantographs(const QString &modules_dir, const QString &custom_cfg_dir);

    /// Иницаализация высоковольтной части схемы (ГВ, тяговый трансформатор)
    void initHighVoltageScheme(const QString &modules_dir, const QString &custom_cfg_dir);

    /// Инициализация вспомогательных машин (ФР, МК, МВ1 - МВ6)
    void initSupplyMachines(const QString &modules_dir, const QString &custom_cfg_dir);

    /// Инициализация питательной магистрали
    void initPneumoSupply(const QString &modules_dir, const QString &custom_cfg_dir);

    /// Инициализация приборов управления тормозами
    void initBrakesControl(const QString &modules_dir, const QString &custom_cfg_dir);

    /// Инициализация тормозного оборудования
    void initBrakesEquipment(const QString &modules_dir, const QString &custom_cfg_dir);

    /// Инициализация схемы управления тягой
    void initTractionControl(const QString &modules_dir, const QString &custom_cfg_dir);

    /// Инициализация приборов безопасности
    void initSafetyDevices(const QString &modules_dir, const QString &custom_cfg_dir);

    void initOtherEquipment(const QString &modules_dir, const QString &custom_cfg_dir);

    void initTriggers();

    /// Предварительные расчёты перед симуляцией
    void preStep(double t);

    /// Предварительный расчёт координат сцепных устройств
    void preStepCouplings(double t);

    /// Шаг симуляции всех систем электровоза
    void step(double t, double dt);

    /// Моделирование сцепных устройств
    void stepCouplings(double t, double dt);

    void stepPantographsControl(double t, double dt);

    void stepMainSwitchControl(double t, double dt);

    void stepTracTransformer(double t, double dt);

    void stepPhaseSplitter(double t, double dt);

    void stepMotorFans(double t, double dt);

    /// Моделирование питательной магистрали
    void stepPneumoSupply(double t, double dt);

    /// Моделирование приборов управления тормозами
    void stepBrakesControl(double t, double dt);

    /// Моделирование тормозного оборудования
    void stepBrakesEquipment(double t, double dt);

    void stepTractionControl(double t, double dt);

    void stepLineContactors(double t, double dt);

    void stepOtherEquipment(double t, double dt);

    void stepSoundSignalsOutput(double t, double dt);

    void lineContactorsControl(bool state);

    float isLineContactorsOff();

    void stepSignalsOutput(double t, double dt);

    /// Моделирование приборов безопасности
    void stepSafetyDevices(double t, double dt);

    double getTractionForce();

    bool getHoldingCoilState() const;

    /// Обработка нажатий клавиш
    void keyProcess();

    void debugPrint(double t, double dt);

    void load_brakes_config(QString path);

    void loadConfig(QString cfg_path);

private slots:

    void slotAutoStart();
};

#endif // VL60K_H

