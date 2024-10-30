#ifndef     NETWORK_DATA_TYPES_H
#define     NETWORK_DATA_TYPES_H

#include    <QByteArray>
#include    <QBuffer>
#include    <QDataStream>
#include    <QTcpSocket>

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
enum StructureType
{
    STYPE_EMPTY_DATA,
    STYPE_TOPOLOGY_DATA,
    STYPE_TRAIN_POSITION,
    STYPE_CONNECTOR_STATE,
    STYPE_TRAJ_BUSY_STATE,
    STYPE_SIGNALS_LIST,
    STYPE_SIGNAL_STATE,
    STYPE_OPEN_SIGNAL,
    STYPE_CLOSE_SIGNAL
};

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
struct network_data_t
{
    /// Тип передаваемой/принимаемой структуры
    StructureType   stype = STYPE_EMPTY_DATA;
    /// Размер данных
    qsizetype data_size = 0;
    /// Сериализованные данные
    QByteArray      data;

    /// Сериализуем, подготоваливая кадр, передаваемый по сети
    QByteArray serialize()
    {
        QByteArray tmp_data;
        QBuffer buff(&tmp_data);
        buff.open(QIODevice::WriteOnly);
        QDataStream stream(&buff);

        stream << data.size() + sizeof(data_size) + sizeof(stype);
        stream << stype;
        stream << data;

        return buff.data();
    }

    void deserialize(QByteArray &data)
    {
        QBuffer buff(&data);
        buff.open(QIODevice::ReadOnly);
        QDataStream stream(&buff);

        stream >> data_size;
        stream >> stype;        

        stream >> this->data;

        // Контрольная сериализация полученных данных
        QByteArray tmp = this->serialize();
        // Удаляем из полученного блока фактически сериализованное
        if (tmp.size() >= data.size())
        {
            data = data.mid(tmp.size());
        }
    }

    bool check_data(StructureType stype)
    {
        switch (stype)
        {
        case STYPE_EMPTY_DATA:
            return false;

        case STYPE_TOPOLOGY_DATA:
            return true;

        case STYPE_TRAIN_POSITION:
            return true;

        case STYPE_CONNECTOR_STATE:
            return true;

        case STYPE_TRAJ_BUSY_STATE:
            return true;

        case STYPE_SIGNALS_LIST:
            return true;

        case STYPE_SIGNAL_STATE:
            return true;

        case STYPE_OPEN_SIGNAL:
            return true;

        case STYPE_CLOSE_SIGNAL:
            return true;
        }

        return false;
    }
};

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
struct client_data_t
{
    QTcpSocket  *socket = Q_NULLPTR;
    network_data_t  received_data;
};

#endif
