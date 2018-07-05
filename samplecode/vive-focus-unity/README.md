# OpenARK-VIVE-Focus

OpenARK for VIVE Focus - Minimal hand tracking example
Adapted from [Vivedu](https://github.com/vivedu/OpenARK-VIVE-Focus).

## Build & run instructions

Before doing anything, make sure that your have updated the server's IP address in `Assets/Config.cs`.

0. Connect the Vive Focus to the computer through a USB cable.
1. Open `OpenARK-VIVE-Focus-master` in `Unity`.
2. Select `File` -> `Bulid Settings`, switch the Platform to `Android`.
3. If the current scene is not already listed, click `Add Open Scenes`
4. Select `Build And Run`. For the first time you run it, name the APK to anything you want.
5. Put Focus on, quit the app, then go to `Settings > Settings > Apps > OpenARK Vive Focus` and select `Force Stop`. Finally navigate to `Permissions` and grant the app camera access.
6. `Build & Run` again or stop the app and reopen it to use.

## Android environment configuration

- WaveSDK Version: WaveSDK-2.0.37 
- SDK API Level: 25 
- JDK Version: JDK 1.8 
- Unity Version: Unity 2017.2.3f1

## Notes

- Ensure that your Focus and computer are on the same network.
- You can use adb to print log information from Android device: `adb logcat`.
- Instead of `Build and Run`, you can also `Build` and use `adb install -r apk_name.apk` to install manually.
- Currently this only supports one-hand tracking (model used is right hand), but it should not be too hard to support both left/right hands

