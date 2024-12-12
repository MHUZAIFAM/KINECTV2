//sensitive for right foot, best for left foot, cant differentiate between feet always prints right foot, and ultimately cant differentiate between multiple people who is the real patient and so on and so forth
#include <Kinect.h>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include <deque>
#include <cmath>
#include <chrono>

using namespace std;
using namespace std::chrono;

// Define pairs of joints to draw skeleton
const std::vector<std::pair<JointType, JointType>> bones = {
    { JointType_Head, JointType_Neck },
    { JointType_Neck, JointType_SpineShoulder },
    { JointType_SpineShoulder, JointType_SpineMid },
    { JointType_SpineMid, JointType_SpineBase },
    { JointType_SpineShoulder, JointType_ShoulderLeft },
    { JointType_SpineShoulder, JointType_ShoulderRight },
    { JointType_SpineBase, JointType_HipLeft },
    { JointType_SpineBase, JointType_HipRight },
    { JointType_ShoulderLeft, JointType_ElbowLeft },
    { JointType_ElbowLeft, JointType_WristLeft },
    { JointType_WristLeft, JointType_HandLeft },
    { JointType_ShoulderRight, JointType_ElbowRight },
    { JointType_ElbowRight, JointType_WristRight },
    { JointType_WristRight, JointType_HandRight },
    { JointType_HipLeft, JointType_KneeLeft },
    { JointType_KneeLeft, JointType_AnkleLeft },
    { JointType_AnkleLeft, JointType_FootLeft },
    { JointType_HipRight, JointType_KneeRight },
    { JointType_KneeRight, JointType_AnkleRight },
    { JointType_AnkleRight, JointType_FootRight }
};

// Joint tracking and foot position tracking logic
struct JointTracker {
    float prevZ = 0.0f;
    bool isRaised = false;

    bool checkMovement(float currZ) {
        // Check if the foot is raised or moved forward/backward
        float threshold = 0.1f; // Movement threshold (adjustable)
        bool movementDetected = fabs(currZ - prevZ) > threshold;
        prevZ = currZ;
        return movementDetected;
    }

    bool isStableForDuration(float currZ, float threshold, float timeLimit, system_clock::time_point& startTime) {
        // Check if the foot has been stable in its position for a certain time
        if (fabs(currZ - prevZ) < threshold) {
            if (duration_cast<seconds>(system_clock::now() - startTime).count() >= timeLimit) {
                return true;
            }
        }
        else {
            startTime = system_clock::now(); // Reset the timer
        }
        prevZ = currZ;
        return false;
    }
};

int main() {
    // Initialize Kinect Sensor, readers, and coordinate mapper
    IKinectSensor* sensor = nullptr;
    IColorFrameReader* colorFrameReader = nullptr;
    IBodyFrameReader* bodyFrameReader = nullptr;
    ICoordinateMapper* coordinateMapper = nullptr;

    // Initialize Kinect
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

    IColorFrameSource* colorSource = nullptr;
    sensor->get_ColorFrameSource(&colorSource);
    if (!colorSource) {
        cerr << "Error: Color frame source not found." << endl;
        sensor->Close();
        sensor->Release();
        return -1;
    }

    hr = colorSource->OpenReader(&colorFrameReader);
    if (FAILED(hr)) {
        cerr << "Error: Could not open color frame reader. HRESULT: " << hr << endl;
        sensor->Close();
        sensor->Release();
        return -1;
    }

    IBodyFrameSource* bodySource = nullptr;
    sensor->get_BodyFrameSource(&bodySource);
    if (!bodySource) {
        cerr << "Error: Body frame source not found." << endl;
        colorSource->Release();
        sensor->Close();
        sensor->Release();
        return -1;
    }

    hr = bodySource->OpenReader(&bodyFrameReader);
    if (FAILED(hr)) {
        cerr << "Error: Could not open body frame reader. HRESULT: " << hr << endl;
        colorSource->Release();
        sensor->Close();
        sensor->Release();
        return -1;
    }

    colorSource->Release();
    bodySource->Release();

    // OpenCV window
    cv::namedWindow("Kinect Camera Feed", cv::WINDOW_AUTOSIZE);

    // Initialize joint trackers for both feet
    JointTracker leftFootTracker, rightFootTracker;
    system_clock::time_point leftFootStartTime, rightFootStartTime;
    bool rightFootRaised = false;
    bool balanceChecked = false;

    while (true) {
        // Process color frame
        IColorFrame* colorFrame = nullptr;
        HRESULT hrColor = colorFrameReader->AcquireLatestFrame(&colorFrame);

        if (SUCCEEDED(hrColor)) {
            IFrameDescription* frameDescription = nullptr;
            colorFrame->get_FrameDescription(&frameDescription);

            int width, height;
            frameDescription->get_Width(&width);
            frameDescription->get_Height(&height);

            UINT bufferSize = width * height * 4;
            BYTE* colorBuffer = new BYTE[bufferSize];
            hrColor = colorFrame->CopyConvertedFrameDataToArray(bufferSize, colorBuffer, ColorImageFormat_Bgra);

            if (SUCCEEDED(hrColor)) {
                cv::Mat colorMat(height, width, CV_8UC4, colorBuffer);
                cv::Mat bgrMat;
                cv::cvtColor(colorMat, bgrMat, cv::COLOR_BGRA2BGR);

                // Process body frame
                IBodyFrame* bodyFrame = nullptr;
                HRESULT hrBody = bodyFrameReader->AcquireLatestFrame(&bodyFrame);

                if (SUCCEEDED(hrBody)) {
                    IBody* bodies[BODY_COUNT] = { 0 };
                    bodyFrame->GetAndRefreshBodyData(_countof(bodies), bodies);

                    for (int i = 0; i < BODY_COUNT; ++i) {
                        IBody* body = bodies[i];
                        if (body) {
                            BOOLEAN isTracked = false;
                            body->get_IsTracked(&isTracked);

                            if (isTracked) {
                                Joint joints[JointType_Count];
                                body->GetJoints(_countof(joints), joints);

                                // Track left and right foot
                                Joint leftAnkle = joints[JointType_AnkleLeft];
                                Joint leftToes = joints[JointType_FootLeft];
                                Joint rightAnkle = joints[JointType_AnkleRight];
                                Joint rightToes = joints[JointType_FootRight];

                                // Check if the person is standing still for balance check
                                if (!balanceChecked) {
                                    if (leftFootTracker.isStableForDuration(leftAnkle.Position.Z, 0.05f, 3, leftFootStartTime) &&
                                        rightFootTracker.isStableForDuration(rightAnkle.Position.Z, 0.05f, 3, rightFootStartTime)) {
                                        cout << "Initial foot coordinates: Left Foot (X: " << leftAnkle.Position.X << ", Y: " << leftAnkle.Position.Y << ", Z: " << leftAnkle.Position.Z << ") "
                                            << "Right Foot (X: " << rightAnkle.Position.X << ", Y: " << rightAnkle.Position.Y << ", Z: " << rightAnkle.Position.Z << ")" << endl;
                                        balanceChecked = true;
                                    }
                                }

                                // Show message asking to raise right foot
                                if (!rightFootRaised) {
                                    cv::putText(bgrMat, "Raise your right foot!", cv::Point(50, 50), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 0), 2);
                                    if (rightAnkle.TrackingState == TrackingState_Tracked && fabs(rightAnkle.Position.Z - leftAnkle.Position.Z) > 0.1f) {
                                        rightFootRaised = true;
                                        cout << "Right foot raised!" << endl;
                                        rightFootStartTime = system_clock::now(); // Start timer when foot is raised
                                    }
                                }

                                // Check if the raised foot touches the other leg or ground
                                if (rightFootRaised && (fabs(rightAnkle.Position.Z - leftAnkle.Position.Z) < 0.1f || fabs(rightAnkle.Position.Y) < 0.1f)) {
                                    auto elapsed = duration_cast<seconds>(system_clock::now() - rightFootStartTime).count();
                                    cout << "Time the foot was raised: " << elapsed << " seconds" << endl;
                                    rightFootRaised = false; // Reset after foot touches other leg/ground
                                }
                            }
                        }
                    }

                    bodyFrame->Release();
                }

                cv::imshow("Kinect Camera Feed", bgrMat);
            }

            delete[] colorBuffer;
            frameDescription->Release();
            colorFrame->Release();
        }

        if (cv::waitKey(30) == 13) break;
    }

    // Cleanup
    colorFrameReader->Release();
    bodyFrameReader->Release();
    coordinateMapper->Release();
    sensor->Close();
    sensor->Release();

    return 0;
}
