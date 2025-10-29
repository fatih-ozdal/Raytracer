#include "parser.h"
#include <fstream>
#include <nlohmann/json.hpp>

using json = nlohmann::json;

Scene parser::loadFromJson(const string &filepath)
{
    std::ifstream f(filepath);
    json data = json::parse(f);


    Scene s;
    return s;
}
