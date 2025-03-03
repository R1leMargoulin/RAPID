#pragma once
#include <bits/stdc++.h>
#include <random>

#include <jsoncpp/json/json.h>
#include <fstream>
#include <iostream>
#include <string>


namespace math
{
    float random_float(float min, float max);
}

namespace structures{
    typedef struct transform2D
    {
        float x; // position on the x axis
        float y; // position on the y axis
        float w; // w rotation on the z axis
    }transform2D;

    typedef struct initConfig{
        std::string mapFile;
    }initConfig;

}

namespace utilFunctions{
    void readJsonFile(const std::string& filename);
}