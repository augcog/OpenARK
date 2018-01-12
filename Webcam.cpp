#include "stdafx.h"
#include "version.h"
#include "Webcam.h"

namespace ark {
    /***
    Opens a webcam and returns the handle
    ***/
    Webcam::Webcam(int code)
    {
        cap.release();
        cap.open(code);
    }

    void Webcam::update()
    {
        cap.read(frame);
    }

    /***
    Closes the webcam
    ***/
    Webcam::~Webcam()
    {
        cap.release();
    }
}