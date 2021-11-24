# Install Azure Kinect SDK for OpenARK on Windows.

> Please contact xiaosx@berkeley.edu if you have any further questions regarding install k4a sdk for openark on windows

1. download pre-build sdk from [link](https://github.com/microsoft/Azure-Kinect-Sensor-SDK/blob/develop/docs/usage.md)

2. install pre-build sdk.

We won't be using the pre-build sdk, but there is a `.dll` package inside the pre-build sdk that we need.

3. git clone the sdk source file [github link](https://github.com/microsoft/Azure-Kinect-Sensor-SDK) into your local directory.

4. built k4a sdk from soruce. DO NOT follow the k4a install instruction and use ninja, you should use visual studio code.
    1. go to the search bar on the bottom left of windows. 
    2. search `command prompt`
    3. open it with `run as admin` ( on the right side of the search result, there is a place allow you to choose open mode). The admin mode enable you to install k4a sdk to `program file`. 

```shell
mkdir build && cd build
cmake -G "Visual Studio 16 2019" ..
cmake --build . --config Release --target install
```

the k4a sdk should be installed under your `C:\Program Files (x86)\K4A`

5. add enviroment variable
    1. go to the search bar on the bottom left of windows
    2. search `advanced system setting` and open the `View Advanced ...` app, click on `Enviroment Variables ...` under `Advanced`
    3. Find the `Path` under `System variables` & click to open it
    4. Add `C:\Program Files (x86)\K4A\bin` to the path

6. add `.dll` requirement
    1. copy `depthengine_2_0.dll` from `C:\Program Files\Azure Kinect SDK v1.4.1\sdk\windows-desktop\amd64\release\bin` 
    2. paste it under `C:\Program Files (x86)\K4A\bin`

## FAQ

1. No 'stdatomic.h'
    ```shell
    cannot open include file 'stdatomic.h' no such file or directory
    ```
    **fix** : downgrade CMake to 3.17
    
    > reference: [libsoundio can't build with CMake 3.18](https://github.com/microsoft/Azure-Kinect-Sensor-SDK/issues/1363)
