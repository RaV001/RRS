#include    <tcp-client.h>
#include    <CfgReader.h>
#include    <iostream>
#include    <QTcpSocket>
#include    <QNetworkProxy>

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
TcpClient::TcpClient(QObject *parent) : QObject(parent)
{

}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
TcpClient::~TcpClient()
{

}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
bool TcpClient::init(const tcp_config_t &tcp_config)
{
    this->tcp_config = tcp_config;
    connectionTimer = new QTimer(this);
    connectionTimer->setInterval(tcp_config.reconnect_interval);
    connect(connectionTimer, &QTimer::timeout, this, &TcpClient::slotOnConnectionTimeout);
    connect(socket, &QTcpSocket::errorOccurred, this, &TcpClient::slotAcceptError);

    socket = new QTcpSocket(this);
    in.setDevice(socket);
    in.setVersion(QDataStream::Qt_4_0);

    connect(socket, &QTcpSocket::connected, this, &TcpClient::slotConnect);
    connect(socket, &QTcpSocket::destroyed, this, &TcpClient::slotDisconnect);
    connect(socket, &QTcpSocket::readyRead, this, &TcpClient::slotReceive);

    socket->setProxy(QNetworkProxy(QNetworkProxy::NoProxy));

    connectionTimer->start();

    return true;
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
void TcpClient::sendRequest(StructureType stype)
{
    network_data_t request;
    request.stype = stype;

    socket->write(request.serialize());
    socket->flush();
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
void TcpClient::sendSwitchState(QString conn_name, int state_fwd, int state_bwd)
{
    network_data_t request;
    request.stype = STYPE_CONNECTOR_STATE;

    QBuffer buff(&request.data);
    buff.open(QIODevice::WriteOnly);
    QDataStream stream(&buff);

    stream << conn_name << state_fwd << state_bwd;

    socket->write(request.serialize());
    socket->flush();
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
void TcpClient::sendSignalState(QString conn_name, int sig_dir, bool open)
{
    network_data_t request;

    if (open)
    {
        request.stype = STYPE_OPEN_SIGNAL;
    }
    else
    {
        request.stype = STYPE_CLOSE_SIGNAL;
    }

    QBuffer buff(&request.data);
    buff.open(QIODevice::WriteOnly);
    QDataStream stream(&buff);

    stream << conn_name;
    stream << sig_dir;

    socket->write(request.serialize());
    socket->flush();
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
bool TcpClient::isConnected() const
{
    if (socket == Q_NULLPTR)
    {
        return false;
    }

    if (socket->state() == QAbstractSocket::ConnectedState)
    {
        return true;
    }

    return false;
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
void TcpClient::connectToServer(const tcp_config_t &tcp_config)
{
    socket->abort();
    socket->connectToHost(QHostAddress(tcp_config.host_addr),
                          tcp_config.port,
                          QIODevice::ReadWrite);
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
void TcpClient::process_received_data(network_data_t &net_data)
{
    if (net_data.data.isEmpty())
    {
        return;
    }

    switch (net_data.stype)
    {
    case STYPE_TOPOLOGY_DATA:
        {
            qsizetype size = net_data.data.size();

            emit setTopologyData(net_data.data);
            break;
        }
    case STYPE_TRAIN_POSITION:
        emit setSimulatorData(net_data.data);
        break;

    case STYPE_CONNECTOR_STATE:
        emit setSwitchState(net_data.data);
        break;

    case STYPE_TRAJ_BUSY_STATE:
        emit setTrajBusyState(net_data.data);
        break;

    case STYPE_SIGNALS_LIST:
        emit setSignalsData(net_data.data);
        break;

    case STYPE_SIGNAL_STATE:
        emit updateSignal(net_data.data);
        break;

    default:

        break;
    }
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
void TcpClient::slotConnect()
{
    connectionTimer->stop();
    emit connected();
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
void TcpClient::slotDisconnect()
{
    socket->abort();
    connectionTimer->start();
    emit disconnect();
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
void TcpClient::slotOnConnectionTimeout()
{
    //if (!isConnected())
    //{
        this->connectToServer(tcp_config);
        //Journal::instance()->info("Try connect to server...");

        emit sendLogMessage("Try connect to server...");
    //}
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
void TcpClient::slotReceive()
{
    while (socket->bytesAvailable())
    {
        if (is_first_data)
        {

            QBuffer b(&recvBuff);
            b.open(QIODevice::ReadOnly);
            QDataStream stream(&b);

            stream >> wait_data_size;

            is_first_data = false;

            recvBuff.append(socket->readAll());
        }
        else
        {
            recvBuff.append(socket->readAll());
        }
    }

    while (recvBuff.size() > wait_data_size)
    {
        // Десириализуем принятые данные в структуру сетевого пакета
        received_data.deserialize(recvBuff);

        // Обработка принятого сетевого пакета
        process_received_data(received_data);        

        is_first_data = true;
    }
}

void TcpClient::slotAcceptError(QAbstractSocket::SocketError error)
{

}

void TcpClient::slotGetRecvBufferSize(qsizetype &size) const
{
    size = recvBuff.size();
}
