#include "stdafx.h"
#include "version.h"
#include "StreamingAverager.h"
#include "Util.h"

namespace ark {
    StreamingAverager::StreamingAverager(int frequency, double rejectionDistance)
    {
        sampleFrequency = frequency;
        rejectionThreshold = rejectionDistance;
        dataPoints = std::deque<Point3f>();
    }

    Point3f StreamingAverager::addDataPoint(Point3f pt)
    {
        if (dataPoints.size() > 0 && util::euclideanDistance(pt, getCurrentAverage()) > rejectionThreshold)
        {
            addEmptyPoint();
            return getCurrentAverage();
        }

        if (dataPoints.size() >= sampleFrequency)
        {
            auto extra = dataPoints.front();
            dataPoints.pop_front();
            currentValue -= extra;
        }
        dataPoints.push_back(pt);
        currentValue += pt;

        return getCurrentAverage();
    }

    void StreamingAverager::addEmptyPoint()
    {
        if (dataPoints.size() > 0)
        {
            Point3f extra = dataPoints.front();
            dataPoints.pop_front();
            currentValue = currentValue - extra;
        }
    }

    Point3f StreamingAverager::getCurrentAverage()
    {
        Point3f average;
        if (dataPoints.size() != 0)
        {
            average[0] = currentValue[0] / dataPoints.size();
            average[1] = currentValue[1] / dataPoints.size();
            average[2] = currentValue[2] / dataPoints.size();
        }
        return average;
    }

    StreamingAverager::~StreamingAverager()
    {

    }
}