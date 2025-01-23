#include "config.hpp"
#include <string>
#include <yaml-cpp/yaml.h>

namespace Configurator {
    Config load_config(std::string filename) { 
        Config config;
        YAML::Node config_node = YAML::LoadFile(filename);

        config.host = config_node["connection"]["host"].as<std::string>();
        config.depth_port = config_node["connection"]["depth_port"].as<int>();
        config.color_port = config_node["connection"]["color_port"].as<int>();
        config.height = config_node["camera"]["height"].as<int>();
        config.width = config_node["camera"]["width"].as<int>();

        return config;
    }
}