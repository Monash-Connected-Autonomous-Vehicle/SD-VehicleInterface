#include <cstdint>

/**
 * Contains functions which interpret autoware targets.
 */
namespace auxiliarycontroller {

/** Should the hazard lights be on? */
bool GetHazardLightsRequest(uint8_t targetHazardLights);

/** Should the left indicator be on? */
bool GetIndicatorLeftRequest(uint8_t targetIndicatorsCmd);

/** Should the right indicator be on? */
bool GetIndicatorRightRequest(uint8_t targetIndicatorsCmd);

} // namespace auxiliarycontroller
