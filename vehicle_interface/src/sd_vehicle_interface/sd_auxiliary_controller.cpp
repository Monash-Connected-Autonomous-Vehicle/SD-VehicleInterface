#include "sd_auxiliary_controller.h"

namespace auxiliarycontroller {

/**
 * Determines if hazard lights should be on from autoware message
 */
bool GetHazardLightsRequest(uint8_t targetHazardLights) {
  return targetHazardLights == 2;
}

} // namespace auxiliarycontroller
