#include <Kinect.h>
#include <opencv2/opencv.hpp>
#include <iostream>

using namespace cv;

void processDepthFrame(UINT16* depthData, int width, int height) {
    Mat depthMat(height, width, CV_16UC1, depthData);
    Mat displayMat;

    // Convert depth to a displayable format
    depthMat.convertTo(displayMat, CV_8U, 255.0 / 8000.0);

    // Threshold to create a binary image
    Mat binaryMat;
    threshold(displayMat, binaryMat, 127, 255, THRESH_BINARY);

    // Find contours
    std::vector<std::vector<Point>> contours;
    findContours(binaryMat, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

    for (const auto& contour : contours) {
        // Approximate contours to polygons
        std::vector<Point> poly;
        approxPolyDP(contour, poly, 3, true);

        // Check if the shape is an X (specific logic)
        if (poly.size() == 4) { // Basic check for quadrilaterals
            // Determine if the angles suggest an X shape
            double angle1 = fabs(atan2(poly[1].y - poly[0].y, poly[1].x - poly[0].x) -
                atan2(poly[2].y - poly[3].y, poly[2].x - poly[3].x)) * 180.0 / CV_PI;

            double angle2 = fabs(atan2(poly[3].y - poly[0].y, poly[3].x - poly[0].x) -
                atan2(poly[2].y - poly[1].y, poly[2].x - poly[1].x)) * 180.0 / CV_PI;

            if (angle1 < 10.0 || angle2 < 10.0) { // Allow some tolerance
                // Ensure that poly has valid points before calling boundingRect
                if (!poly.empty()) {
                    // Draw rectangle around the detected X shape
                    Rect boundingRect = cv::boundingRect(poly); // Use cv:: to avoid ambiguity
                    rectangle(displayMat, boundingRect, Scalar(0, 255, 0), 2);

                    // Calculate and mark the center
                    Point center = (boundingRect.tl() + boundingRect.br()) / 2;
                    circle(displayMat, center, 5, Scalar(0, 0, 255), -1);

                    // Show result and exit after detecting one X
                    imshow("Depth Feed", displayMat);
                    return; // Exit after detecting the first X
                }
            }
        }
    }

    // Show result if no X detected
    imshow("Depth Feed", displayMat);
}

int main() {
    // Initialize Kinect
    IKinectSensor* sensor = nullptr;
    HRESULT hr = GetDefaultKinectSensor(&sensor);
    if (SUCCEEDED(hr)) {
        sensor->Open();

        IDepthFrameReader* depthFrameReader = nullptr;
        IDepthFrameSource* depthFrameSource = nullptr;
        sensor->get_DepthFrameSource(&depthFrameSource);
        depthFrameSource->OpenReader(&depthFrameReader);

        while (true) {
            IDepthFrame* depthFrame = nullptr;
            if (SUCCEEDED(depthFrameReader->AcquireLatestFrame(&depthFrame))) {
                UINT16* depthData = nullptr;
                int nWidth = 0, nHeight = 0; // Change to int

                // Get frame description
                IFrameDescription* frameDescription = nullptr;
                depthFrame->get_FrameDescription(&frameDescription);
                frameDescription->get_Width(&nWidth);  // Use int*
                frameDescription->get_Height(&nHeight); // Use int*
                frameDescription->Release();

                // Access underlying buffer
                UINT capacity;
                depthFrame->AccessUnderlyingBuffer(&capacity, &depthData);

                // Process the depth frame
                processDepthFrame(depthData, nWidth, nHeight);
                depthFrame->Release();
            }

            // Exit on ESC key
            if (waitKey(30) == 27) break;
        }

        depthFrameReader->Release();
        depthFrameSource->Release();
        sensor->Close();
    }
    else {
        std::cerr << "Failed to initialize Kinect sensor." << std::endl;
    }

    return 0;
}
