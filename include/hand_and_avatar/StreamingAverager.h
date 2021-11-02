#pragma once
#include <deque>
#include "Version.h"

namespace ark {
    /*
    * Averages streaming data to combate outliers. A sample frequency and rejection threshold is used to determined the best fit point at the current time frame.
    */
    class StreamingAverager
    {
    public:
        /*
        * Constructs a new instance of the streaming averager.
        * @param frequency number of previous points to use in the averaging computation (must be at least 1)
        * @param rejectionDistance maximum distance allowed between current point and previous point
        */
        StreamingAverager(int frequency, float rejectionDistance);

        /*
        * Adds new data point to the streaming averager.
        * @param pt point at the current frame
        * @return the average a the current frame
        */
        Vec3f addDataPoint(Vec3f pt);

        /*
        * Adds a empty data point to the stream of points.
        * If there are no points to add at the current frame, an empty point should be added to push out the old points
        */
        void addEmptyPoint();

    private:
        /*
        * Number of previous points to be used in the average.
        */
        int sampleFrequency;

        /*
        * Square of maximum jump distance allowed between current point and previous point
        */
        float rejectionThreshold;

        /*
        * The current average value
        */
        Vec3f currentValue;

        /*
        * Recently seen data points
        * @see sampleFrequency
        */
        std::deque<Vec3f> dataPoints;

        /*
        * Compute the current average.
        * @return (x,y,z) average at current frame
        */
        Vec3f getCurrentAverage();
    };
}