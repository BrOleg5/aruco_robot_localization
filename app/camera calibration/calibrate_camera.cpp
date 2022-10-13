#include "arucolocalization.hpp"
#include <opencv2/core/utility.hpp>
#include "read_save_camera_parameters.hpp"
#include <cmath>

cv::Point2f calibrate(const std::vector<std::vector<cv::Point2f>>& markerCorners, float markerSize, cv::Point2f& std);

const std::string about = "Calibrate camera using Aruco marker.";
const std::string keys  =
        "{h help ? usage |       | Print help message}"
        "{d              |       | dictionary: DICT_4X4_50=0, DICT_4X4_100=1, DICT_4X4_250=2,"
        "DICT_4X4_1000=3, DICT_5X5_50=4, DICT_5X5_100=5, DICT_5X5_250=6, DICT_5X5_1000=7, "
        "DICT_6X6_50=8, DICT_6X6_100=9, DICT_6X6_250=10, DICT_6X6_1000=11, DICT_7X7_50=12,"
        "DICT_7X7_100=13, DICT_7X7_250=14, DICT_7X7_1000=15, DICT_ARUCO_ORIGINAL = 16,"
        "DICT_APRILTAG_16h5=17, DICT_APRILTAG_25h9=18, DICT_APRILTAG_36h10=19, DICT_APRILTAG_36h11=20}"
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
    }
    else {
        std::cerr << "Dictionary not specified" << std::endl;
        return 1;
    }

    cv::VideoCapture video_capture;
    if(parser.has("ci")) {
        int cam_id = parser.get<int>("ci");
        #ifdef WIN32
            video_capture.open(cam_id, cv::CAP_DSHOW);
        #else
            video_capture.open(cam_id);
        #endif
        video_capture.set(cv::CAP_PROP_FRAME_WIDTH, 1920);
        video_capture.set(cv::CAP_PROP_FRAME_HEIGHT, 1080);
        video_capture.set(cv::CAP_PROP_FOCUS, 0); // min: 0, max: 255, increment:5
        video_capture.set(cv::CAP_PROP_AUTO_EXPOSURE, 1);
        video_capture.set(cv::CAP_PROP_BUFFERSIZE, 1);
        // link: https://stackoverflow.com/a/70074022
        #ifdef WIN32
            video_capture.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
        #endif

        //Checking for the camera to be connected 
        if (video_capture.isOpened()) {
            std::cout << "Camera connected." << std::endl;
        }
        else {
            std::cerr << "Camera not connected." << std::endl;
            return 2;
        }
    }
    else if(parser.has("v")){
        std::string videoFile = parser.get<std::string>("v");
        video_capture.open(videoFile);
        //Checking for the video file to be opened 
        if (video_capture.isOpened()) {
            std::cout << "Video file opened." << std::endl;
        }
        else {
            std::cerr << "Video file not opened." << std::endl;
            return 2;
        }
    }
    else {
        std::cerr << "Camera of video file not specified" << std::endl;
        return 3;
    }

    int markerID = 0;
    if(parser.has("id")) {
        markerID = parser.get<int>("id");
    }
    else {
        markerID = -1;
    }

    int calibFrameNumber = 30;
    if(parser.has("n")) {
        calibFrameNumber = parser.get<int>("n");
    }

    float markerSize = 0;
    if(parser.has("ms")) {
        markerSize = parser.get<float>("ms");
    }

    std::string outputFile;
    if(parser.has("o")){
        outputFile = parser.get<std::string>("o");
    }

    if(!parser.check()) {
        parser.printErrors();
        return 4;
    }

	ArucoLocalization cv_system(video_capture, dictionary_name);
    std::vector<std::vector<cv::Point2f>> calibMarkerCorners;
    for (int i = 0; i < calibFrameNumber; i++) {
        int status = cv_system.detectMarkers();
        if (status == 0) {
            std::vector<int> marker_ids;
            cv_system.getMarkersIndexes(marker_ids);
            int markerIndex = 0;
            if(markerID >= 0) {
                std::vector<int>::iterator markerIterator = std::find(marker_ids.begin(), marker_ids.end(), markerID);
                if(markerIterator != marker_ids.end()) {
                    markerIndex = static_cast<int>(markerIterator - marker_ids.begin());
                }
                else {
                    std::cerr << "Marker with ID=" << markerID << " not searched.\n";
                    return 1;
                }
            }
            std::vector<std::vector<cv::Point2f>> markers_corners;
            cv_system.getMarkersCorners(markers_corners);
            calibMarkerCorners.push_back(markers_corners[markerIndex]);
        }
        else if(status == 1) {
            std::cout << "Marker localization failed." << std::endl;
            video_capture.release();
            return 2;
        }
        else if(status == 2) {
            break;
        }
    }
    video_capture.release();
    cv::Point2f std = {0.0f, 0.0f};
    cv::Point2f pixelResolution = calibrate(calibMarkerCorners, markerSize, std);
    if (!saveCameraParams(outputFile, pixelResolution, markerSize, std))
    {
        return 3;
    }
	return 0;
}

cv::Point2f calibrate(const std::vector<std::vector<cv::Point2f>>& markerCorners, float markerSize, cv::Point2f& std) {
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