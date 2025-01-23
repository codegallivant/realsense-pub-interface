#pragma once
#include <string>

namespace Configurator {
    typedef struct Config {
        std::string host;
        int depth_port;
        int color_port;
        int height;
        int width;
    } Config;

    Config load_config(std::string filename);
}