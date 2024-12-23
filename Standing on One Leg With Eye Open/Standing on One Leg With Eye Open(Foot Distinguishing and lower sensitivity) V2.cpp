#include <Kinect.h>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <chrono>
#include <cmath>

using namespace std;
using namespace std::chrono;

// Threshold for foot raise detection
const float FOOT_RAISE_THRESHOLD_Z = 0.1f; // Depth difference
const float FOOT_RAISE_THRESHOLD_Y = 0.05f; // Height difference

// Timer variables
steady_clock::time_point footRaiseStart;
bool timerActive = false;
float elapsedTime = 0.0f;

void drawText(cv::Mat& frame, const string& text, cv::Point position, cv::Scalar color, double scale = 1.0) {
    cv::putText(frame, text, position, cv::FONT_HERSHEY_SIMPLEX, scale, color, 2);
}

int main() {
    // Initialize Kinect
    IKinectSensor* sensor = nullptr;
    IBodyFrameReader* bodyFrameReader = nullptr;
    ICoordinateMapper* coordinateMapper = nullptr;

    HRESULT hr = GetDefaultKinectSensor(&sensor);
    if (FAILED(hr) || !sensor) {
        cerr << "Error: Kinect sensor not found. HRESULT: " << hr << endl;
        return -1;
    }

    hr = sensor->Open();
    if (FAILED(hr)) {
        cerr << "Error: Could not open Kinect sensor. HRESULT: " << hr << endl;
        return -1;
    }

    hr = sensor->get_CoordinateMapper(&coordinateMapper);
    if (FAILED(hr)) {
        cerr << "Error: Could not get coordinate mapper. HRESULT: " << hr << endl;
        sensor->Close();
        sensor->Release();
        return -1;
    }

    IBodyFrameSource* bodySource = nullptr;
    sensor->get_BodyFrameSource(&bodySource);
    if (!bodySource) {
        cerr << "Error: Body frame source not found." << endl;
        sensor->Close();
        sensor->Release();
        return -1;
    }

    hr = bodySource->OpenReader(&bodyFrameReader);
    if (FAILED(hr)) {
        cerr << "Error: Could not open body frame reader. HRESULT: " << hr << endl;
        bodySource->Release();
        sensor->Close();
        sensor->Release();
        return -1;
    }

    bodySource->Release();

    // OpenCV window
    cv::namedWindow("Kinect Feed", cv::WINDOW_AUTOSIZE);

    while (true) {
        IBodyFrame* bodyFrame = nullptr;
        HRESULT hrBody = bodyFrameReader->AcquireLatestFrame(&bodyFrame);

        if (SUCCEEDED(hrBody)) {
            IBody* bodies[BODY_COUNT] = { 0 };
            bodyFrame->GetAndRefreshBodyData(_countof(bodies), bodies);

            cv::Mat frame(480, 640, CV_8UC3, cv::Scalar(0, 0, 0)); // Black background for feed

            for (int i = 0; i < BODY_COUNT; ++i) {
                IBody* body = bodies[i];
                if (body) {
                    BOOLEAN isTracked = false;
                    body->get_IsTracked(&isTracked);

                    if (isTracked) {
                        Joint joints[JointType_Count];
                        body->GetJoints(_countof(joints), joints);

                        Joint leftFoot = joints[JointType_FootLeft];
                        Joint rightFoot = joints[JointType_FootRight];

                        // Ensure joints are tracked
                        if (leftFoot.TrackingState == TrackingState_Tracked &&
                            rightFoot.TrackingState == TrackingState_Tracked) {

                            float leftZ = leftFoot.Position.Z;
                            float rightZ = rightFoot.Position.Z;
                            float leftY = leftFoot.Position.Y;
                            float rightY = rightFoot.Position.Y;

                            // Determine which foot is raised
                            string status = "Standing";
                            if (fabs(leftZ - rightZ) > FOOT_RAISE_THRESHOLD_Z ||
                                fabs(leftY - rightY) > FOOT_RAISE_THRESHOLD_Y) {

                                if (leftY > rightY) {
                                    status = "Left Foot Raised";
                                    if (!timerActive) {
                                        footRaiseStart = steady_clock::now();
                                        timerActive = true;
                                    }
                                } else if (rightY > leftY) {
                                    status = "Right Foot Raised";
                                    if (!timerActive) {
                                        footRaiseStart = steady_clock::now();
                                        timerActive = true;
                                    }
                                }
                            } else {
                                if (timerActive) {
                                    auto footRaiseEnd = steady_clock::now();
                                    elapsedTime = duration_cast<seconds>(footRaiseEnd - footRaiseStart).count();
                                    cout << "Foot raised for " << elapsedTime << " seconds." << endl;
                                    timerActive = false;
                                }
                            }

                            // Display status on feed
                            drawText(frame, "Status: " + status, cv::Point(50, 50), cv::Scalar(0, 255, 0));

                            // Display live timer
                            if (timerActive) {
                                auto currentTime = steady_clock::now();
                                float liveElapsedTime = duration_cast<seconds>(currentTime - footRaiseStart).count();
                                drawText(frame, "Timer: " + to_string(liveElapsedTime) + "s", cv::Point(50, 100), cv::Scalar(0, 255, 255));
                            }
                        }
                    }
                }
            }

            bodyFrame->Release();

            // Show frame
            cv::imshow("Kinect Feed", frame);
        }

        if (cv::waitKey(30) == 13) break; // Press Enter to exit
    }

    // Cleanup
    bodyFrameReader->Release();
    coordinateMapper->Release();
    sensor->Close();
    sensor->Release();

    return 0;
}
