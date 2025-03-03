#include "utils.h"


namespace math{
    float random_float(float min, float max) {
        // Seed with a real random value, if available
        std::random_device rd;

        // Create a random number generator
        std::mt19937 gen(rd());

        // Define a distribution in the range [min, max]
        std::uniform_real_distribution<> dis(min, max);

        // Generate a random float
        return dis(gen);
    }
}

namespace utilFunctions{
    void readJsonFile(const std::string& filename) {
        // Open the file stream within the function scope
        std::ifstream people_file(filename, std::ifstream::binary);
    
        if (!people_file.is_open()) {
            std::cerr << "Could not open the file!" << std::endl;
            return;
        }
    
        Json::Value people;
        people_file >> people;

        
        // Close the file stream
        people_file.close();
    }
}