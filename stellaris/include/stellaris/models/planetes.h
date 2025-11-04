#include <cstdint>
namespace stellaris::models {
  
  enum PlanetType {
    Sun,
    Earth,
    Moon
  };
  struct planet {
    PlanetType type;
    int mass;
    int radius;
    int meterPerSecond;
    float gravity;
    float density;
    uint32_t surfaceArea;
  };
  
}