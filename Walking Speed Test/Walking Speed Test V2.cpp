#include <iostream>
#include <Kinect.h>
#include <opencv2/opencv.hpp>
#include <deque>
#include <numeric>
#include <chrono>
#include <iomanip>
#include<iostream>
using namespace std;

#pragma comment(lib, "kinect20.lib")

template<class Interface>
inline void SafeRelease(Interface*& interfaceToRelease) {
    if (interfaceToRelease) {
        interfaceToRelease->Release();
        interfaceToRelease = nullptr;
    }
}

// Moving average filter
float getSmoothedDepth(std::deque<float>& depthQueue, float newDepth, size_t windowSize) {
    depthQueue.push_back(newDepth);
    if (depthQueue.size() > windowSize) {
        depthQueue.pop_front();
    }
    float sum = std::accumulate(depthQueue.begin(), depthQueue.end(), 0.0f);
    return sum / depthQueue.size();
}

// Timer variables
bool isTiming = false;
std::chrono::steady_clock::time_point startTime;
std::chrono::steady_clock::time_point endTime;

// Helper function to calculate the moving average of a deque
float calculateMovingAverage(const std::deque<float>& values) {
    if (values.empty()) return 0.0f;
    float sum = std::accumulate(values.begin(), values.end(), 0.0f);
    return sum / values.size();
}

// Timer logic for walking test
// Variables for displaying timer information
std::string timerMessage = "";
float timerStartDepth = 0.0f;
float timerStopDepth = 0.0f;

void processWalkingTest(float depth, std::string& timerMessage) {
    // Check for start condition (depth between 5.98m and 6.0m)
    if (!isTiming && depth >= 5.99f && depth <= 6.0f) {
        isTiming = true;
        startTime = std::chrono::steady_clock::now();
        timerMessage = "Timer Started! Depth: " + std::to_string(depth).substr(0, 4) + " m";
        std::cout << "Timer Started! Depth: " << depth << endl;

    }

    // Check for stop condition (depth between 0.98m and 1.0m)
    if (isTiming && depth >= 1.0f && depth <= 1.1f) {
        endTime = std::chrono::steady_clock::now();     
        isTiming = false;

        // Calculate elapsed time
        auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime).count();
        float elapsedSeconds = elapsedTime / 1000.0f;

        timerMessage = "Timer stopped! Depth: " + std::to_string(depth).substr(0, 4) + " m " + 
            "Time Taken: " + std::to_string(elapsedSeconds).substr(0, 5) + " s";
        cout << "Timer Stopped! Depth "<< depth << "\nTime: " << elapsedSeconds << " s" << endl;
    }
}

int main() {
    // Initialize Kinect sensor
    IKinectSensor* kinectSensor = nullptr;
    HRESULT hr = GetDefaultKinectSensor(&kinectSensor);

    if (FAILED(hr) || !kinectSensor) {
        std::cerr << "Failed to initialize Kinect sensor!" << std::endl;
        return -1;
    }

    hr = kinectSensor->Open();
    if (FAILED(hr)) {
        std::cerr << "Failed to open Kinect sensor!" << std::endl;
        return -1;
    }

    // Depth frame reader
    IDepthFrameReader* depthFrameReader = nullptr;
    IDepthFrameSource* depthFrameSource = nullptr;

    hr = kinectSensor->get_DepthFrameSource(&depthFrameSource);
    if (FAILED(hr) || !depthFrameSource) {
        std::cerr << "Failed to get Depth Frame Source!" << std::endl;
        return -1;
    }

    hr = depthFrameSource->OpenReader(&depthFrameReader);
    if (FAILED(hr) || !depthFrameReader) {
        std::cerr << "Failed to open Depth Frame Reader!" << std::endl;
        return -1;
    }

    // Color frame reader
    IColorFrameReader* colorFrameReader = nullptr;
    IColorFrameSource* colorFrameSource = nullptr;

    hr = kinectSensor->get_ColorFrameSource(&colorFrameSource);
    if (FAILED(hr) || !colorFrameSource) {
        std::cerr << "Failed to get Color Frame Source!" << std::endl;
        return -1;
    }

    hr = colorFrameSource->OpenReader(&colorFrameReader);
    if (FAILED(hr) || !colorFrameReader) {
        std::cerr << "Failed to open Color Frame Reader!" << std::endl;
        return -1;
    }

    // Depth frame properties
    int depthWidth = 0, depthHeight = 0;
    IFrameDescription* depthFrameDescription = nullptr;
    depthFrameSource->get_FrameDescription(&depthFrameDescription);
    depthFrameDescription->get_Width(&depthWidth);
    depthFrameDescription->get_Height(&depthHeight);
    SafeRelease(depthFrameDescription);

    // Depth buffer and smoothing
    std::vector<UINT16> depthBuffer(depthWidth * depthHeight);
    std::deque<float> depthQueue; // To store depth values for smoothing
    const size_t smoothingWindowSize = 10; // Adjust smoothing window size as needed

    // Main loop
    // Main loop
while (true) {
    // Get Depth Frame
    IDepthFrame* depthFrame = nullptr;
    hr = depthFrameReader->AcquireLatestFrame(&depthFrame);

    if (SUCCEEDED(hr)) {
        hr = depthFrame->CopyFrameDataToArray(static_cast<UINT>(depthBuffer.size()), &depthBuffer[0]);

        if (SUCCEEDED(hr)) {
            // Find the closest depth value in the center of the frame
            int centerX = depthWidth / 2;
            int centerY = depthHeight / 2;
            int index = centerY * depthWidth + centerX;
            UINT16 depthValue = depthBuffer[index];

            // Convert depth to meters and smooth it
            float depthInMeters = depthValue * 0.001f;
            float smoothedDepth = getSmoothedDepth(depthQueue, depthInMeters, smoothingWindowSize);

            // Process the walking test timer
            processWalkingTest(smoothedDepth, timerMessage);

            // Get color frame for live feed
            IColorFrame* colorFrame = nullptr;
            hr = colorFrameReader->AcquireLatestFrame(&colorFrame);

            if (SUCCEEDED(hr)) {
                int colorWidth = 0, colorHeight = 0;
                IFrameDescription* colorFrameDescription = nullptr;
                colorFrame->get_FrameDescription(&colorFrameDescription);
                colorFrameDescription->get_Width(&colorWidth);
                colorFrameDescription->get_Height(&colorHeight);
                SafeRelease(colorFrameDescription);

                // Prepare frame buffer
                std::vector<BYTE> colorBuffer(colorWidth * colorHeight * 4); // BGRA
                hr = colorFrame->CopyConvertedFrameDataToArray(static_cast<UINT>(colorBuffer.size()), colorBuffer.data(), ColorImageFormat_Bgra);

                if (SUCCEEDED(hr)) {
                    // Create OpenCV Mat and display it
                    cv::Mat colorMat(colorHeight, colorWidth, CV_8UC4, colorBuffer.data());

                    // Display depth and timer information
                    cv::putText(colorMat, "Depth: " + std::to_string(smoothedDepth).substr(0, 4) + " m",
                                cv::Point(50, 50), cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 255, 0), 2);
                    cv::putText(colorMat, timerMessage, cv::Point(50, 100),
                                cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 255, 255), 2);

                    cv::imshow("Kinect Live Feed", colorMat);
                    if (cv::waitKey(30) == 27) break; // Exit on ESC key
                }
            }

            SafeRelease(colorFrame);
        }
    }

    SafeRelease(depthFrame);
}

    // Clean up
    SafeRelease(depthFrameReader);
    SafeRelease(depthFrameSource);
    SafeRelease(colorFrameReader);
    SafeRelease(colorFrameSource);
    if (kinectSensor) kinectSensor->Close();
    SafeRelease(kinectSensor);

    return 0;
}
