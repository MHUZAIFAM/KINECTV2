#include <Kinect.h>

#include <opencv2/opencv.hpp>
#include <iostream>

int main() {
    IKinectSensor* sensor = nullptr;
    IColorFrameReader* colorFrameReader = nullptr;

    // Initialize Kinect Sensor
    if (FAILED(GetDefaultKinectSensor(&sensor)) || !sensor) {
        std::cerr << "Kinect sensor not found!" << std::endl;
        return -1;
    }
    sensor->Open();

    // Get color frame source and reader
    IColorFrameSource* colorSource = nullptr;
    sensor->get_ColorFrameSource(&colorSource);
    colorSource->OpenReader(&colorFrameReader);

    // Create an OpenCV window
    cv::namedWindow("Kinect Feed", cv::WINDOW_AUTOSIZE);

    // Frame loop
    while (true) {
        IColorFrame* frame = nullptr;
        HRESULT hr = colorFrameReader->AcquireLatestFrame(&frame);
        if (SUCCEEDED(hr)) {
            // Get frame description
            IFrameDescription* frameDescription = nullptr;
            frame->get_FrameDescription(&frameDescription);

            // Get frame dimensions
            int width, height;
            frameDescription->get_Width(&width);
            frameDescription->get_Height(&height);

            // Allocate buffer for the color data
            UINT bufferSize = width * height * 4; // 4 bytes per pixel (BGRA)
            BYTE* colorBuffer = new BYTE[bufferSize];

            // Copy the color data to the buffer
            hr = frame->CopyConvertedFrameDataToArray(bufferSize, colorBuffer, ColorImageFormat_Bgra);
            if (SUCCEEDED(hr)) {
                // Create an OpenCV Mat from the buffer
                cv::Mat colorMat(height, width, CV_8UC4, colorBuffer);

                // Convert BGRA to BGR for OpenCV
                cv::Mat bgrMat;
                cv::cvtColor(colorMat, bgrMat, cv::COLOR_BGRA2BGR);

                // Display the image in the OpenCV window
                cv::imshow("Kinect Feed", bgrMat);
            }

            // Clean up
            delete[] colorBuffer;
            frame->Release();
            frameDescription->Release(); // Release the frame description
        }
        else {
            std::cout << "Failed to capture frame." << std::endl;
        }

        // Break the loop on ESC key press
        if (cv::waitKey(30) == 27) break;
    }

    // Cleanup
    colorFrameReader->Release();
    colorSource->Release();
    sensor->Close();
    sensor->Release();

    return 0;
}
