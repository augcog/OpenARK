#pragma once
#include "stdafx.h"

namespace ark {
    /*
    * Averages streaming data to combate outliers. A sample frequency and rejection threshold is used to determined the best fit point at the current time frame.
    */
    class StreamingAverager
    {
    public:
        /*
        * Constructs a new instance of the streaming averager.
        * @param frequency how many previous points to use in the averaging computation
        * @param rejectionDistance maximum jump distance allowed between current point and previous point
        */
        StreamingAverager(int frequency, double rejectionDistance);

        /*
        * Deconstructs an instance of the streaming averager.
        */
        ~StreamingAverager();

        /*
        * Adds new data point to the streaming averager.
        * @param pt point at the current frame
        * @return the average a the current frame
        */
        Point3f addDataPoint(Point3f pt);

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
        * Maximum jump distance allowed between current point and previous point
        */
        double rejectionThreshold;

        /*
        * The current average value
        */
        Point3f currentValue;

        /*
        * Recently seen data points
        * @see sampleFrequency
        */
        std::deque<Point3f> dataPoints;

        /*
        * Compute the current average.
        * @return (x,y,z) average at current frame
        */
        Point3f getCurrentAverage();
    };
}