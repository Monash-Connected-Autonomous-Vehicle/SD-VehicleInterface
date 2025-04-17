#include <cstdint>
#include <tuple>

/**
 * Contains functions which interpret autoware targets.
 */
namespace auxiliarycontroller {

/**
 * Should the hazard lights be on?
 */
bool GetHazardLightsRequest(uint8_t targetHazardLights);

/**
 * Should the turning indicators be on?
 * Returns a tuple (left, right) which can be unpacked
 */
std::tuple<bool, bool> GetIndicatorsRequest(uint8_t targetIndicatorsCmd);

} // namespace auxiliarycontroller
