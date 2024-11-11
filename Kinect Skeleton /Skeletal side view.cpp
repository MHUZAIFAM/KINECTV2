#include <Kinect.h>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>

using namespace std;

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

int main() {
    // Initialize Kinect Sensor, readers, and coordinate mapper
    IKinectSensor* sensor = nullptr;
    IColorFrameReader* colorFrameReader = nullptr;
    IBodyFrameReader* bodyFrameReader = nullptr;
    ICoordinateMapper* coordinateMapper = nullptr;

    if (FAILED(GetDefaultKinectSensor(&sensor)) || !sensor) {
        std::cerr << "Kinect sensor not found!" << std::endl;
        return -1;
    }

    sensor->Open();
    sensor->get_CoordinateMapper(&coordinateMapper);

    IColorFrameSource* colorSource = nullptr;
    sensor->get_ColorFrameSource(&colorSource);
    colorSource->OpenReader(&colorFrameReader);

    IBodyFrameSource* bodySource = nullptr;
    sensor->get_BodyFrameSource(&bodySource);
    bodySource->OpenReader(&bodyFrameReader);

    cv::namedWindow("Kinect Skeleton", cv::WINDOW_AUTOSIZE);

    // Frame loop
    while (true) {
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

                IBodyFrame* bodyFrame = nullptr;
                HRESULT hrBody = bodyFrameReader->AcquireLatestFrame(&bodyFrame);

                if (SUCCEEDED(hrBody)) {
                    IBody* bodies[BODY_COUNT] = { 0 };
                    hrBody = bodyFrame->GetAndRefreshBodyData(_countof(bodies), bodies);

                    for (int i = 0; i < BODY_COUNT; ++i) {
                        IBody* body = bodies[i];
                        if (body) {
                            BOOLEAN isTracked = false;
                            body->get_IsTracked(&isTracked);

                            if (isTracked) {
                                // Declare the joints array inside the loop
                                Joint joints[JointType_Count];
                                body->GetJoints(_countof(joints), joints);

                                // Draw bones (lines connecting joints)
                                for (const auto& bone : bones) {
                                    Joint joint1 = joints[bone.first];
                                    Joint joint2 = joints[bone.second];

                                    if (joint1.TrackingState == TrackingState_Tracked && joint2.TrackingState == TrackingState_Tracked) {
                                        ColorSpacePoint colorPoint1, colorPoint2;
                                        coordinateMapper->MapCameraPointToColorSpace(joint1.Position, &colorPoint1);
                                        coordinateMapper->MapCameraPointToColorSpace(joint2.Position, &colorPoint2);

                                        int x1 = static_cast<int>(colorPoint1.X);
                                        int y1 = static_cast<int>(colorPoint1.Y);
                                        int x2 = static_cast<int>(colorPoint2.X);
                                        int y2 = static_cast<int>(colorPoint2.Y);

                                        // Check if the coordinates are within bounds
                                        if (x1 >= 0 && x1 < width && y1 >= 0 && y1 < height &&
                                            x2 >= 0 && x2 < width && y2 >= 0 && y2 < height) {
                                            cv::line(bgrMat, cv::Point(x1, y1), cv::Point(x2, y2), cv::Scalar(0, 255, 0), 3);
                                        }
                                    }
                                }

                                // Draw circles at each of the 25 joints
                                for (int j = 0; j < JointType_Count; j++) {
                                    if (joints[j].TrackingState == TrackingState_Tracked) {
                                        ColorSpacePoint colorPoint;
                                        coordinateMapper->MapCameraPointToColorSpace(joints[j].Position, &colorPoint);

                                        int x = static_cast<int>(colorPoint.X);
                                        int y = static_cast<int>(colorPoint.Y);

                                        // Check if the joint position is within bounds before drawing
                                        if (x >= 0 && x < width && y >= 0 && y < height) {
                                            cv::circle(bgrMat, cv::Point(x, y), 10, cv::Scalar(0, 0, 255), -1); // Draw a circle with radius 10
                                        }
                                    }
                                }
                            }
                        }
                    }

                    bodyFrame->Release();
                }

                cv::imshow("Kinect Skeleton", bgrMat);
            }

            delete[] colorBuffer;
            colorFrame->Release();
            frameDescription->Release();
        }

        // Break on Enter key press
        if (cv::waitKey(30) == 13) break;
    }

    // Release Kinect resources
    colorFrameReader->Release();
    bodyFrameReader->Release();
    coordinateMapper->Release();
    colorSource->Release();
    sensor->Close();
    sensor->Release();

    return 0;
}
