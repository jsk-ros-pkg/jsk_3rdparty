#ifndef SMART_DEVICE_PROTOCOL_PACKET_UTIL_H__
#define SMART_DEVICE_PROTOCOL_PACKET_UTIL_H__

#include <variant>
#include <vector>

typedef std::variant<int32_t, float, std::string, bool> SDPData;
typedef std::tuple<std::string, std::string> SDPInterfaceDescription;

std::string get_serialization_format(const std::vector<SDPData> &data)
{
    std::string serialization_format;
    for (auto itr = data.begin(); itr != data.end(); ++itr)
    {
        if (std::holds_alternative<int32_t>(*itr))
        {
            serialization_format += "i";
        }
        else if (std::holds_alternative<float>(*itr))
        {
            serialization_format += "f";
        }
        else if (std::holds_alternative<std::string>(*itr))
        {
            if (std::get<std::string>(*itr).size() > 16)
            {
                serialization_format += "S";
            }
            else
            {
                serialization_format += "s";
            }
        }
        else if (std::holds_alternative<bool>(*itr))
        {
            serialization_format += "?";
        }
    }
    return serialization_format;
}

bool is_consistent_serialization_format(const std::string &serialization_format, const std::vector<SDPData> &data)
{
    if (serialization_format.size() != std::vector<SDPData>(data).size())
    {
        return false;
    }
    auto itr = data.begin();
    int index_sf = 0;
    while (itr != data.end())
    {
        switch (serialization_format[index_sf])
        {
        case 'i':
            if (not std::holds_alternative<int32_t>(*itr))
            {
                return false;
            }
            break;
        case 'f':
            if (not std::holds_alternative<float>(*itr))
            {
                return false;
            }
            break;
        case 'S':
        case 's':
            if (not std::holds_alternative<std::string>(*itr))
            {
                return false;
            }
            break;
        case '?':
        case 'b':
            if (not std::holds_alternative<bool>(*itr))
            {
                return false;
            }
            break;
        default:
            return false;
        }
        itr++;
        index_sf;
    }
    return true;
}

#endif // SMART_DEVICE_PROTOCOL_PACKET_UTIL_H__