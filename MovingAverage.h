/**
 * @file MovingAverage.h
 * @author Ryan Johnson (ryan@johnsonweb.us)
 * @brief Exponentially Weighted Moving Average Helper
 * @version 0.1
 * @date 2020-05-26
 *
 * @copyright Copyright (c) 2020
 *
 */

#ifndef MOVING_AVERAGE_GUARD_H
#define MOVING_AVERAGE_GUARD_H

/**
 * @brief helper class for calculating a moving average
 */
class MovingAverage {
  public:
    /**
     * @brief Construct a new Moving Average object
     *
     * @param startingPoint the initial value to average against
     * @param a the alpha parameter (0.0  to 1.0). Lower means more weight is
     * given to older data, i.e. a more heavy average
     */
    MovingAverage(float startingPoint, float a)
        : avg(startingPoint), alpha(a){};

    /**
     * @brief Adds a datapoint to the average
     * @param point the new sample point to add
     */
    void addPoint(float point) { avg = point * alpha + avg * (1.0 - alpha); };

    /**
     * @brief Get the cumulative average
     * @return float calculated average
     */
    float getAverage() { return avg; };

  private:
    float alpha; // Lots of smoothing is lower
    float avg;
};

#endif
