#include "log.hpp"

#include <map>
#include <string>

using std::string;
using std::map;

SvgFrame DebugFrame(std::string name) {
    static map<string, int> counter;
    string frame_name = name + std::to_string(counter[name]) + ".svg";

    counter[name]++;

    return SvgFrame(2000, frame_name);
}
