#include <cstdint>
#include <tuple>
namespace auxiliarycontroller {

bool GetHazardLightsRequest(uint8_t targetHazardLights);

/**
 * Returns whether the left/right indicator should be on as (left, right)
 */
std::tuple<bool, bool> GetIndicatorsRequest(uint8_t targetIndicatorsCmd);

} // namespace auxiliarycontroller
