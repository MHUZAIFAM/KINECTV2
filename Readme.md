# Kinect V2 Color Feed Display using C++

This project demonstrates how to connect and retrieve a color feed from the Kinect V2 sensor using C++ in Visual Studio. The color feed is displayed using OpenCV.

## Prerequisites

### Hardware
- Microsoft Kinect V2 sensor
- USB 3.0 port

### Software
- Visual Studio (tested on Visual Studio 2019 or newer)
- Kinect for Windows SDK v2.0
- OpenCV (tested with OpenCV 4.1.0)

## Setup

1. **Clone the Repository**
   ```bash
   git clone https://github.com/your-username/kinect-v2-color-feed.git
   ```

2. **Open the Project in Visual Studio**
   - Open Visual Studio and create a new C++ project or open the provided solution file.

4. **Configure Kinect SDK Directories**
   - Go to `Project Properties` > `Configuration Properties` > `VC++ Directories`.
   - Set up the following directories:

   **Include Directories:**
   ```bash
   C:\Program Files\Microsoft SDKs\Kinect\v2.0_1409\inc
   ```
   **Library Directories:**
   ```bash
   C:\Program Files\Microsoft SDKs\Kinect\v2.0_1409\inc
   ```
5. **Link Kinect Libraries**
   - Go to `Configuration Properties` > `Linker` > `Input` > `Additional Dependencies`.
   - Add `Kinect20.lib`.

6. **Configure OpenCV Directories**
   - Under VC++ Directories, add the following paths:
   **Include Directories:**
     ```bash
     C:\opencv\build\include
     ```
   **Library Directories:**
     ```bash
     C:\opencv\build\x64\vc16\lib
     ```
7. **Link OpenCV Library**
   - Go to `Linker` > `Input` > `Additional Dependencies`.
   - Add `opencv_world4100d.lib`.
     


   
