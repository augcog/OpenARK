#include "stdafx.h"
#include "Version.h"
#include "StreamingAverager.h"
#include "Util.h"

namespace ark {
    StreamingAverager::StreamingAverager(int frequency, float rejectionDistance) :
        sampleFrequency(frequency), rejectionThreshold(rejectionDistance * rejectionDistance)
    {
        ARK_ASSERT(frequency > 0, "Sampling frequency must be at least 1");
    }

    Vec3f StreamingAverager::addDataPoint(Vec3f pt)
    {
        if (util::norm(pt - getCurrentAverage()) > rejectionThreshold)
        {
            addEmptyPoint();
        }
        else {
            if (dataPoints.size() >= sampleFrequency)
            {
                currentValue -= dataPoints.front();
                dataPoints.pop_front();
            }

            dataPoints.push_back(pt);
            currentValue += pt;
        }

        return getCurrentAverage();
    }

    void StreamingAverager::addEmptyPoint()
    {
        if (dataPoints.empty()) return;
        currentValue -= dataPoints.front();;
        dataPoints.pop_front();
    }

    Vec3f StreamingAverager::getCurrentAverage()
    {
        return dataPoints.empty() ? 0 : currentValue / (int)dataPoints.size();
    }
}