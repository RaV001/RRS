#ifndef     PNEUMO_HOSE_DATA_H
#define     PNEUMO_HOSE_DATA_H

//------------------------------------------------------------------------------
// Сигналы рукавов пневмомагистралей
//------------------------------------------------------------------------------
enum
{
    HOSE_SIZE_OF_OUTPUTS = 6,       ///< Размер массива исходящих сигналов

    HOSE_OUTPUT_REF_STATE = 0,      ///< Управление рукавами: 1.0 - соединить, -1.0 - разъединить, 0.0 - оставить текущее состояние
    HOSE_OUTPUT_PIPE_PRESSURE = 1,  ///< Давление в пневматической магистрали
    HOSE_OUTPUT_FLOW_COEFF = 2,     ///< Коэффицент перетока через рукав
    HOSE_OUTPUT_LENGTH = 3,         ///< Длина рукава, м
    HOSE_OUTPUT_SIDE = 4,           ///< Смещение точки крепления рукава в сторону, м
    HOSE_OUTPUT_COORD = 5,          ///< Координата точки крепления руква на треке пути, м


    HOSE_SIZE_OF_INPUTS = 4,        ///< Размер массива входящих сигналов

    HOSE_INPUT_IS_CONNECTED = 0,    ///< Состояние соединения рукавов: 1.0 - соединены, 0.0 - разъединены
    HOSE_INPUT_FLOW_TO_PIPE = 1,    ///< Поток в пневматическую магистраль
    HOSE_INPUT_SIDE_ANGLE = 2,      ///< Угол отклонения рукава в сторону соседнего, радиан
    HOSE_INPUT_DOWN_ANGLE = 3,      ///< Угол свешивания рукава вниз с учётом натяжения от соседнего, радиан
};

//------------------------------------------------------------------------------
// Дополнительно: сигналы линий электропневматического тормоза
//------------------------------------------------------------------------------
enum
{
    HOSE_OUTPUT_EPB_LINES_NUM = 6,      ///< Сигнал с количеством линий управления ЭПТ

    BEGIN_OF_EPB_LINE_SIGNALS = 7,      ///< Индекс элемента - начала массива сигналов управления ЭПТ
    SIZE_OF_SIGNALS_PER_EPB_LINE = 3,   ///< Размер массива исходящих сигналов на каждую линию управления ЭПТ

    EPB_LINE_OUTPUT_VOLTAGE = 0,        ///< Напряжение
    EPB_LINE_OUTPUT_FREQUENCY = 1,      ///< Частота переменного напряжения
    EPB_LINE_OUTPUT_CURRENT = 2,        ///< Потребляемый ток


    HOSE_INPUT_EPB_CONNECTED_NUM = 6,   ///< Количество соединённых линий управления ЭПТ

    EPB_LINE_INPUT_VOLTAGE = 0,         ///< Напряжение
    EPB_LINE_INPUT_FREQUENCY = 1,       ///< Частота переменного напряжения
    EPB_LINE_INPUT_CURRENT = 2,         ///< Потребляемый ток
};

#endif // PNEUMO_HOSE_DATA_H
