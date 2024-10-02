#include    "converter.h"

#include    <QFile>
#include    <iostream>
#include    <sstream>

#include    "path-utils.h"


void add_element(float x, std::vector<float> &array)
{
    float eps = 1.0f;

    if (array.size() != 0)
    {
        float last = *(array.end() - 1);

        if ( abs(last - x) >= eps )
            array.push_back(x);
    }
    else
    {
        array.push_back(x);
    }
}


//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
bool ZDSimConverter::readRouteMAP(const std::string &path, zds_route_map_data_t &map_data)
{
    if (path.empty())
        return false;

    QString data = fileToQString(path);
    if (data.isEmpty())
    {
        std::cout << "File " << path << " not opened" << std::endl;
        return false;
    }

    QTextStream stream(&data);
    return readRouteMAP(stream, map_data);
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
bool ZDSimConverter::readRouteMAP(QTextStream &stream, zds_route_map_data_t &map_data)
{
    bool may_add_info_to_last = false;
    while (!stream.atEnd())
    {
        QString line = stream.readLine();

        if (line.isEmpty())
            continue;

        // Пустое название объекта
        if (*(line.begin()) == ',')
            continue;

        // Строка с объектом заканчивается точкой с запятой, а если нет,
        // это может быть строка с информацией об объекте из предыдущей строки
        if (*(line.end() - 1) != ';')
        {
            if (may_add_info_to_last)
            {
                map_data.back()->obj_info = line.toStdString();
                may_add_info_to_last = false;
            }
            continue;
        }

        // Удаляем завершающую точку с запятой и разделяем строку на данные
        QStringList tokens = line.removeLast().split(',');
        if (tokens.size() < 7)
            continue;

        // Читаем информацию об объекте
        zds_object_position_t zds_object = zds_object_position_t();
        zds_object.obj_name = tokens[0].toStdString();
        zds_object.position.x = tokens[1].toDouble();
        zds_object.position.y = tokens[2].toDouble();
        zds_object.position.z = tokens[3].toDouble();
        zds_object.attitude.x = tokens[4].toDouble();
        zds_object.attitude.y = tokens[5].toDouble();
        zds_object.attitude.z = tokens[6].toDouble();

        map_data.push_back(new zds_object_position_t(zds_object));
        may_add_info_to_last = true;
    }

    return (!map_data.empty());
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
void ZDSimConverter::findSignalsAtMap()
{
    for (auto zds_obj : route_map_data)
    {
        // Игнорируем модели с пустой дополнительной информацией
        // поскольку нас интересуют светофоры с литерой
        if (zds_obj->obj_info.empty())
            continue;

        // Проходной, предвходной
        if ((zds_obj->obj_name == "signal_line") ||
            (zds_obj->obj_name == "signal_pred"))
        {
            zds_signal_position_t *signal = new zds_signal_position_t();
            signal->obj_name = zds_obj->obj_name;
            signal->position = zds_obj->position;
            signal->attitude = zds_obj->attitude;

            signal->type = "ab3line";
            signal->liter = zds_obj->obj_info;
            signals_line_data.push_back(signal);
        }

        // Входной
        if (zds_obj->obj_name == "signal_enter")
        {
            zds_signal_position_t *signal = new zds_signal_position_t();
            signal->obj_name = zds_obj->obj_name;
            signal->position = zds_obj->position;
            signal->attitude = zds_obj->attitude;

            signal->type = "ab3entr";
            signal->liter = zds_obj->obj_info;
            signals_enter_data.push_back(signal);
        }

        // Маршрутный/выходной, карликовый 5 линз, карликовый 3 линзы
        if ((zds_obj->obj_name == "signal_exit") ||
            (zds_obj->obj_name == "sig_k5p") ||
            (zds_obj->obj_name == "sig_k3p"))
        {
            zds_signal_position_t *signal = new zds_signal_position_t();
            signal->obj_name = zds_obj->obj_name;
            signal->position = zds_obj->position;
            signal->attitude = zds_obj->attitude;

            signal->type = "ab3exit";
            signal->liter = zds_obj->obj_info;
            signals_exit_data.push_back(signal);
        }

        // Повторительный, карликовый повторительный
        if ((zds_obj->obj_name == "sig_povt") ||
            (zds_obj->obj_name == "sig_povt_k"))
        {
            zds_signal_position_t *signal = new zds_signal_position_t();
            signal->obj_name = zds_obj->obj_name;
            signal->position = zds_obj->position;
            signal->attitude = zds_obj->attitude;

            signal->liter = zds_obj->obj_info;
            signals_povt_data.push_back(signal);
        }

        // Маневровый мачтовый, маневровый карликовый
        if ((zds_obj->obj_name == "sig_m2m") ||
            (zds_obj->obj_name == "sig_k2m"))
        {
            zds_signal_position_t *signal = new zds_signal_position_t();
            signal->obj_name = zds_obj->obj_name;
            signal->position = zds_obj->position;
            signal->attitude = zds_obj->attitude;

            signal->liter = zds_obj->obj_info;
            signals_maneurous_data.push_back(signal);
        }
    }

    // Привязка к главным путям
    for (auto map : {signals_line_data,
                     signals_enter_data,
                     signals_exit_data,
                     signals_povt_data,
                     signals_maneurous_data})
    {
        for (auto sig : map)
        {
            // Трек главного пути напротив светофора
            float coord;
            zds_track_t nearest_track = getNearestTrack(sig->position, tracks_data1, coord);
            bool near_end = (coord > (nearest_track.route_coord + 0.5 * nearest_track.length));
            int track_id1 = near_end ? nearest_track.prev_uid + 1 : nearest_track.prev_uid;

            // Точка главного пути напротив светофора
            double track_coord = coord - nearest_track.route_coord;
            dvec3 nearest_point = nearest_track.begin_point + nearest_track.orth * track_coord;
            dvec3 rho_right = sig->position - nearest_point;
            sig->distance_from_main = dot(rho_right, nearest_track.right);
            if (abs(sig->distance_from_main) < 6.0)
            {
                sig->route_num = 1;
                sig->track_id = track_id1;
            }
        }
    }

    // Вывод для отладки
    std::string path = compinePath(topologyDir, "signals_at_map.conf");

    QFile file(QString(path.c_str()));
    if (!file.open(QIODevice::WriteOnly | QIODevice::Text))
        return;

    QTextStream stream(&file);
    stream.setEncoding(QStringConverter::Utf8);
    stream.setRealNumberNotation(QTextStream::FixedNotation);

    for (auto map : {signals_line_data,
                     signals_enter_data,
                     signals_exit_data,
                     signals_povt_data,
                     signals_maneurous_data})
    {
        for (auto sig : map)
        {
            stream << sig->obj_name.c_str()
                   << DELIMITER_SYMBOL << sig->liter.c_str()
                   << DELIMITER_SYMBOL << sig->position.x
                   << DELIMITER_SYMBOL << sig->position.y
                   << DELIMITER_SYMBOL << sig->position.z
                   << DELIMITER_SYMBOL << sig->attitude.x
                   << DELIMITER_SYMBOL << sig->attitude.y
                   << DELIMITER_SYMBOL << sig->attitude.z;
            if (sig->route_num == -1)
            {
                stream << "\n"
                       << "NO_AT_MAIN"
                       << DELIMITER_SYMBOL
                       << DELIMITER_SYMBOL << "dist  " << sig->distance_from_main
                       << "\n";
            }
            else
            {
                stream << "\n"
                       << "MAIN:";
                if (sig->route_num == 1)
                    stream << "route1";
                if (sig->route_num == 2)
                    stream << "route2";

                stream << DELIMITER_SYMBOL
                       << DELIMITER_SYMBOL << "dist  " << sig->distance_from_main
                       << DELIMITER_SYMBOL << "dir" << sig->direction
                       << DELIMITER_SYMBOL << "track" << sig->track_id
                       << "\n";
            }
        }
    }

    file.close();
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
bool ZDSimConverter::findNeutralInsertions(std::vector<neutral_insertion_t> ni)
{
    std::vector<float> begins;
    std::vector<float> ends;

    for (auto zds_object : route_map_data)
    {
        if (zds_object->obj_name == "nvne")
        {
            neutral_insertion_t n_insertion;

            float begin_coord = 0;
            zds_track_t track = getNearestTrack(zds_object->position, tracks_data1, begin_coord);

            add_element(begin_coord, begins);
        }

        if (zds_object->obj_name == "nvke")
        {
            neutral_insertion_t n_insertion;

            float end_coord = 0;
            zds_track_t track = getNearestTrack(zds_object->position, tracks_data1, end_coord);

            add_element(end_coord, ends);
        }
    }

    std::sort(begins.begin(), begins.end(), std::less<float>());
    std::sort(ends.begin(), ends.end(), std::less<float>());

    if (begins.size() == ends.size())
    {
        float max_len = 500.0f;

        for (size_t i = 0; i < begins.size(); ++i)
        {
            neutral_insertion_t n_ins;

            n_ins.begin_coord = begins[i];
            n_ins.end_coord = ends[i];
            n_ins.length = abs(n_ins.end_coord - n_ins.begin_coord);

            if (n_ins.length <= max_len)
                ni.push_back(n_ins);
        }
    }

    return ni.size() != 0;
}
