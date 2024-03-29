// Copyright 2022 BrOleg5

#include <cmath>

#include <opencv2/core/utility.hpp>
#include <opencv2/videoio.hpp>

#include "arucolocalization.hpp"
#include "read_save_camera_parameters.hpp"

cv::Point2f calibrate(const std::vector<std::vector<cv::Point2f>>& markerCorners,
                      float markerSize, cv::Point2f& std);

const char about[] = "Calibrate camera using Aruco marker.";
const char keys[]  =
        "{h help ? usage |       | Print help message}"
        "{d              |       | dictionary: DICT_4X4_50=0, DICT_4X4_100=1, DICT_4X4_250=2,"
        "DICT_4X4_1000=3, DICT_5X5_50=4, DICT_5X5_100=5, DICT_5X5_250=6, DICT_5X5_1000=7, "
        "DICT_6X6_50=8, DICT_6X6_100=9, DICT_6X6_250=10, DICT_6X6_1000=11, DICT_7X7_50=12,"
        "DICT_7X7_100=13, DICT_7X7_250=14, DICT_7X7_1000=15, DICT_ARUCO_ORIGINAL = 16,"
        "DICT_APRILTAG_16h5=17, DICT_APRILTAG_25h9=18, DICT_APRILTAG_36h10=19,"
        "DICT_APRILTAG_36h11=20}"
        "{id             |       | Marker id}"
        "{ci             |       | Camera id, if ommited, input comes from video file }"
        "{v              |       | Input from video file if input doesnt come from camera (--ci) }"
        "{n              | 30    | Number of frames to calibrate }"
        "{ms             |       | Size of marker in mm}"
        "{o              |       | Output file}";

int main( int argc, char **argv ) {
    cv::CommandLineParser parser(argc, argv, keys);
    parser.about(about);

    if(parser.has("h") || parser.has("help") || parser.has("?") || parser.has("usage")) {
        parser.printMessage();
        return 0;
    }

    cv::aruco::PREDEFINED_DICTIONARY_NAME dictionary_name;
    if (parser.has("d")) {
        int dictionary_id = parser.get<int>("d");
        dictionary_name = cv::aruco::PREDEFINED_DICTIONARY_NAME(dictionary_id);
    } else {
        std::cout << "Dictionary is not specified.\n";
        return -1;
    }

    cv::VideoCapture videoCapture;
    if(parser.has("ci")) {
        int cam_id = parser.get<int>("ci");
        #ifdef WIN32
            videoCapture.open(cam_id, cv::CAP_DSHOW);
        #else
            videoCapture.open(cam_id);
        #endif
        videoCapture.set(cv::CAP_PROP_FRAME_WIDTH, 1920);
        videoCapture.set(cv::CAP_PROP_FRAME_HEIGHT, 1080);
        videoCapture.set(cv::CAP_PROP_FOCUS, 0);  // min: 0, max: 255, increment:5
        videoCapture.set(cv::CAP_PROP_AUTO_EXPOSURE, 1);
        videoCapture.set(cv::CAP_PROP_BUFFERSIZE, 1);
        // link: https://stackoverflow.com/a/70074022
        #ifdef WIN32
            videoCapture.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
        #endif

        // Checking for the camera to be connected
        if (videoCapture.isOpened()) {
            std::cout << "Camera is connected.\n";
        } else {
            std::cout << "Camera is not connected.\n";
            return -1;
        }
    } else if(parser.has("v")) {
        std::string videoFile = parser.get<std::string>("v");
        videoCapture.open(videoFile);
        // Checking for the video file to be opened
        if (videoCapture.isOpened()) {
            std::cout << "Video file is opened.\n";
        } else {
            std::cout << "Video file is not opened.\n";
            return -1;
        }
    } else {
        std::cout << "Camera of video file is not specified.\n";
        return -1;
    }

    int markerID = 0;
    if(parser.has("id")) {
        markerID = parser.get<int>("id");
    } else {
        std::cout << "Aruco marker is not specified.\n";
        return -1;
    }

    int calibFrameNumber = 30;
    if(parser.has("n")) {
        calibFrameNumber = parser.get<int>("n");
    }

    float markerSize = 0;
    if(parser.has("ms")) {
        markerSize = parser.get<float>("ms");
    } else {
        std::cout << "Size of aruco marker is not specified.\n";
        return -1;
    }

    std::string outputFile;
    if(parser.has("o")){
        outputFile = parser.get<std::string>("o");
    }

    if(!parser.check()) {
        parser.printErrors();
        return -1;
    }

    ArucoLocalization cv_system(dictionary_name);
    cv::Mat frame;
    std::vector<std::vector<cv::Point2f>> calibMarkerCorners;
    for (int i = 0; i < calibFrameNumber; i++) {
        videoCapture >> frame;
        if(cv_system.detectMarkers(frame)) {
            int markerIndex = cv_system.filterMarkers(markerID);
            if(markerIndex < 0) {
                std::cout << "Marker with ID=" << markerID << " not searched.\n";
                break;
            }
            std::vector<std::vector<cv::Point2f>> markers_corners;
            cv_system.getMarkersCorners(markers_corners);
            calibMarkerCorners.push_back(markers_corners[markerIndex]);
        } else {
            std::cout << "Marker localization failed.\n";
            break;
        }
    }
    videoCapture.release();
    cv::Point2f std = {0.0f, 0.0f};
    cv::Point2f pixelResolution = calibrate(calibMarkerCorners, markerSize, std);
    if (!saveCameraParams(outputFile, pixelResolution, markerSize, std)) {
        return -1;
    }
    return 0;
}

cv::Point2f calibrate(const std::vector<std::vector<cv::Point2f>>& markerCorners,
                      float markerSize, cv::Point2f& std) {
    float xPixNum = 0;
    float yPixNum = 0;
    size_t frameNumber = markerCorners.size();
    std::vector<float> xRes;
    std::vector<float> yRes;
    for (size_t i = 0; i < frameNumber; i++) {
        float pixNumber1 = std::abs(markerCorners[i][0].x - markerCorners[i][1].x);
        float pixNumber2 = std::abs(markerCorners[i][2].x - markerCorners[i][3].x);
        xRes.push_back(markerSize / pixNumber1);
        xRes.push_back(markerSize / pixNumber2);
        xPixNum += (pixNumber1 + pixNumber2);

        pixNumber1 = std::abs(markerCorners[i][0].y - markerCorners[i][3].y);
        pixNumber2 = std::abs(markerCorners[i][1].y - markerCorners[i][2].y);
        yRes.push_back(markerSize / pixNumber1);
        yRes.push_back(markerSize / pixNumber2);
        yPixNum += (pixNumber1 + pixNumber2);
    }
    xPixNum /= 2.0f * frameNumber;
    yPixNum /= 2.0f * frameNumber;
    cv::Point2f pixelResolution = {markerSize / xPixNum, markerSize / yPixNum};

    // Estimate standard deviation
    float xSumOfSquares = 0;
    float ySumOfSquares = 0;
    for (size_t i = 0; i < (2 * frameNumber); i++) {
        xSumOfSquares += std::pow(xRes[i] - pixelResolution.x, 2);
        ySumOfSquares += std::pow(yRes[i] - pixelResolution.y, 2);
    }
    std.x = std::sqrt(1.0f / (static_cast<float>(2 * frameNumber) - 1.0f) * xSumOfSquares);
    std.y = std::sqrt(1.0f / (static_cast<float>(2 * frameNumber) - 1.0f) * ySumOfSquares);
    return pixelResolution;
}
