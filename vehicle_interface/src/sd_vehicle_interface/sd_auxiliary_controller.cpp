#include "sd_auxiliary_controller.h"

namespace auxiliarycontroller {

bool GetHazardLightsRequest(uint8_t targetHazardLights) {
  return targetHazardLights == 2;
}

bool GetIndicatorLeftRequest(uint8_t target_indicators_cmd) {
  return target_indicators_cmd == 2;
}

bool GetIndicatorRightRequest(uint8_t target_indicators_cmd) {
  return target_indicators_cmd == 3;
}

} // namespace auxiliarycontroller
