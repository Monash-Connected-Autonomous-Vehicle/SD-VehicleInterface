#include "sd_auxiliary_controller.h"
#include <tuple>

namespace auxiliarycontroller {

bool GetHazardLightsRequest(uint8_t targetHazardLights) {
  return targetHazardLights == 2;
}

std::tuple<bool, bool> GetIndicatorsRequest(uint8_t targetIndicatorsCmd) {
  return {targetIndicatorsCmd == 2, targetIndicatorsCmd == 3};
}

} // namespace auxiliarycontroller
