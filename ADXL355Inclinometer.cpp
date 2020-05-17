#include "ADXL355Inclinometer.h"
#include "FaultHandling.h"

Eigen::Vector2d Inclinometer::ADXL355Inclinometer::getData()
{
    ADXL355Measurement measure;
    accel.takeSample();
    measure = accel.getSample();

    // MAGNITUDE PLAUSIBILITY CHECK
    const double gBand = 0.25;
    double scale = sqrt(pow(measure.x / 16.0, 2) + pow(measure.y / 16.0, 2) +
                        pow(measure.z / 16.0, 2));
    if (scale > 1.0 + gBand || scale < 1.0 - gBand) {
        Fault::Handler::instance()->setFaultCode(
            Fault::ACCEL_IMPLAUSIBLE_READING);
    }
    else {
        Fault::Handler::instance()->unlatchFaultCode(
            Fault::ACCEL_IMPLAUSIBLE_READING);
    }

    // Apply EWMA filtering
    movingAverageFilter.addData(measure);
    measure = movingAverageFilter.getAverage();

    scale = sqrt(pow(measure.x / 16.0, 2) + pow(measure.y / 16.0, 2) +
                 pow(measure.z / 16.0, 2));

    Eigen::Vector3d normalized = Eigen::Vector3d(
        measure.x / scale, measure.y / scale, measure.z / scale);

    double pitch = atan(normalized[1] / normalized[2]);
    double roll = atan((-normalized[0]) /
                       sqrt(pow(normalized[1], 2) + pow(normalized[2], 2)));
    return Eigen::Vector2d(pitch, roll);
}