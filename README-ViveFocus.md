## Vive Focus Additional Instructions

- You will need to install Protocol Buffers 3.6.0, which you can build and install from: `https://github.com/protocolbuffers/protobuf/releases/tag/v3.6.0`.

- Build the project according to instructions in `documentation/` (use the file relevant to your platform)

- Calibrate your camera using `samplecode/calib.py` or your own calibration script. Then either copy the resulting `calib.yaml` to overwrite `calib_sample.yaml` in the project directory, or edit `main.cpp`, line 42 to change the calibration to use.

- Sample code for the Unity client side can be found in `samplecode/vive-focus-unity`

