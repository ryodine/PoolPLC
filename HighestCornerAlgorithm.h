/**
 * @file HighestCornerAlgorithm.h
 * @author Ryan Johnson (ryan@johnsonweb.us)
 * @brief Calculates the highest corner of the platform based on measured
 * inclinometer angles.
 * @version 0.1
 * @date 2020-05-26
 *
 * @copyright Copyright (c) 2020
 *
 */

#ifndef HIGHEST_CORNER_ALGO_H
#define HIGHEST_CORNER_ALGO_H

/**
 * @brief This class calculates the highest (or lowest) corner of a plane given
 * roll and pitch angles with configurable hysteresis.
 */
class HighestCornerAlgo {
  public:
    /**
     * @brief Construct a new Highest Corner Algo object
     *
     * The algorithm's hysteresis is much like a thermostat - it attempts to
     * level angles that exceed hystHigh until they fall below hystLow
     *
     * @param hystLow the lower bound for which to not attempt to correct beyond
     * @param hystHigh the upper bound for which to correct deviations that are
     * larger
     */
    HighestCornerAlgo(double hystLow, double hystHigh)
        : corners({false, false, false, false}), lowerbound(hystLow),
          upperbound(hystHigh){};
    void update(double roll, double pitch);

    /**
     * @brief Check if a given corner is high or low. Note: Corner 0 is the 1st
     * quadrant in an x-y coordinate plane, and the other corners go
     * counter-clockwise from there, with Corner 3 being quadrant 4.
     *
     * @param corner the corner to check
     * @param lowestCornerMode true if checking for lowest corner(s), default
     * false
     * @return true if the corner is high (or low if in lowest corner mode)
     * @return false if the corner is low (or high if in lowest corner mode)
     */
    bool getCorner(unsigned int corner, bool lowestCornerMode = false)
    {
        return corners[(((lowestCornerMode) ? 2 : 0) + corner) % 4];
    };

  private:
    bool corners[4];
    double lowerbound;
    double upperbound;
    void resetAll()
    {
        corners[0] = false;
        corners[1] = false;
        corners[2] = false;
        corners[3] = false;
    }
};

#endif
