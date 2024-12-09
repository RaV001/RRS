#include    "converter.h"

#include    <QFile>
#include    "path-utils.h"

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
void ZDSimConverter::writeStations(const zds_start_km_data_t &waypoints)
{
    std::string path = compinePath(toNativeSeparators(topologyDir), FILE_STATIONS);

    QFile file_old(QString(path.c_str()));
    if (file_old.exists())
    {
        std::string backup = FILE_BACKUP_PREFIX + FILE_STATIONS + FILE_BACKUP_EXTENTION;
        file_old.rename( QString(compinePath(topologyDir, backup).c_str()) );
    }

    if (waypoints.empty())
        return;

    QFile file(QString(path.c_str()));
    if (!file.open(QIODevice::WriteOnly | QIODevice::Text))
        return;

    QTextStream stream(&file);
    stream.setEncoding(QStringConverter::Utf8);
    stream.setRealNumberNotation(QTextStream::FixedNotation);

    for (auto station = waypoints.begin(); station != waypoints.end(); ++station)
    {
        stream << station->name.c_str()
            << DELIMITER_SYMBOL << station->mean_point.x
            << DELIMITER_SYMBOL << station->mean_point.y
            << DELIMITER_SYMBOL << station->mean_point.z
            << "\n";
    }

    file.close();
}
