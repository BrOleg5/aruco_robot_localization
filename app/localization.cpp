#include "arucolocalization.hpp"
#include <opencv2/core/utility.hpp>
#include "sharedmemory.hpp"
#include <chrono>
#include <iomanip>
#include "read_save_camera_parameters.hpp"

using namespace std::chrono;

//link: https://learn.microsoft.com/en-us/windows/console/registering-a-control-handler-function?source=recommendations
#if  defined(WIN32) || defined(_WIN32) || defined(__WIN32__)
#   include <windows.h>
#   include <stdio.h>
BOOL isExit = false;
BOOL WINAPI CtrlHandler(DWORD fdwCtrlType);
#endif

const std::string about = "Localization of Aruco marker.";
const std::string keys  =
        "{h help ? usage |        | Print help message}"
        "{d              |        | dictionary: DICT_4X4_50=0, DICT_4X4_100=1, DICT_4X4_250=2,"
        "DICT_4X4_1000=3, DICT_5X5_50=4, DICT_5X5_100=5, DICT_5X5_250=6, DICT_5X5_1000=7, "
        "DICT_6X6_50=8, DICT_6X6_100=9, DICT_6X6_250=10, DICT_6X6_1000=11, DICT_7X7_50=12,"
        "DICT_7X7_100=13, DICT_7X7_250=14, DICT_7X7_1000=15, DICT_ARUCO_ORIGINAL = 16,"
        "DICT_APRILTAG_16h5=17, DICT_APRILTAG_25h9=18, DICT_APRILTAG_36h10=19, DICT_APRILTAG_36h11=20}"
        "{id             |        | Marker id, if ommited, detect all markers from dictionaty }"
        "{ci             |        | Camera id, if ommited, input comes from video file }"
        "{v              |        | Input from video file if input doesnt come from camera (--ci) }"
        "{t              | 0      | Program execution time in ms. If it equals 0, application run until user stop.}"
        "{cp             |        | JSON file with camera parameters }"
        "{shm            |        | Use shared memory to transmit data to other programs}"
        "{ov             | <none> | Output video }"
        "{ce             | 0      | Camera exposure }";

int main( int argc, char **argv ) {
    cv::CommandLineParser parser(argc, argv, keys);
    parser.about(about);

    if(parser.has("h") || parser.has("help") || parser.has("?") || parser.has("usage")) {
        parser.printMessage();
        return 0;
    }

    // Test duration
    double test_duration = parser.get<double>("t");
    // Shared memory flag
    bool shm_flag = parser.has("shm");

    shm::Transmitter<float> transmitter;
    if(shm_flag) {
        transmitter.create("CameraData", 1000);
        //Initialization shared memory vector
        for (size_t i = 0; i < 7; i++) {
            transmitter.data->push_back(0.0f);
        }
    }

    cv::aruco::PREDEFINED_DICTIONARY_NAME dictionary_name;
    if (parser.has("d")) {
        int dictionary_id = parser.get<int>("d");
        dictionary_name = cv::aruco::PREDEFINED_DICTIONARY_NAME(dictionary_id);
    }
    else {
        std::cout << "Dictionary not specified" << std::endl;
        return -1;
    }

    int frame_width = 1920;
    int frame_height = 1080;
    cv::VideoCapture videoCapture;
    if(parser.has("ci")) {
        int cam_id = parser.get<int>("ci");
        #ifdef WIN32
            videoCapture.open(cam_id, cv::CAP_DSHOW);
        #else
            videoCapture.open(cam_id);
        #endif
        videoCapture.set(cv::CAP_PROP_FRAME_WIDTH, frame_width);
        videoCapture.set(cv::CAP_PROP_FRAME_HEIGHT, frame_height);
        videoCapture.set(cv::CAP_PROP_FOCUS, 0); // min: 0, max: 255, increment:5
        videoCapture.set(cv::CAP_PROP_BUFFERSIZE, 1);
        // link: https://stackoverflow.com/a/70074022
        #ifdef WIN32
            videoCapture.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
        #endif
        if(parser.has("ce")) {
            videoCapture.set(cv::CAP_PROP_AUTO_EXPOSURE, 0);
            videoCapture.set(cv::CAP_PROP_EXPOSURE, parser.get<double>("ce"));
        }
        else {
            videoCapture.set(cv::CAP_PROP_AUTO_EXPOSURE, 1);
        }

        //Checking for the camera to be connected 
        if (videoCapture.isOpened()) {
            std::cout << "Camera connected." << std::endl;
        }
        else {
            std::cout << "Camera not connected." << std::endl;
            return -1;
        }
    }
    else if(parser.has("v")) {
        std::string videoFile = parser.get<std::string>("v");
        videoCapture.open(videoFile);
        //Checking for the video file to be opened 
        if (videoCapture.isOpened()) {
            std::cout << "Video file opened." << std::endl;
        }
        else {
            std::cout << "Video file not opened." << std::endl;
            return -1;
        }
    }
    else {
        std::cout << "Camera of video file not specified" << std::endl;
        return -1;
    }

    bool has_marker_id = parser.has("id");
    int markerID = 0;
    if(has_marker_id) {
        markerID = parser.get<int>("id");
    }

    std::string camParamFile;
    if(parser.has("cp")){
        camParamFile = parser.get<std::string>("cp");
    }
    cv::Point2f pixelResolution;
    if (!readCameraParameters(camParamFile, pixelResolution)) {
        std::cout << "Read camera parameters error.\n";
        return -1;
    }
    std::cout << "Camera parameters:\n\tpixel resolution x: " << pixelResolution.x 
            << "\n\tpixel resolution y: " << pixelResolution.y << '\n';
    // validate data
    if((pixelResolution.x == 0) || (pixelResolution.y == 0)) {
        std::cout << "Get invalid camera parameters.\n";
        return -1;
    }
    
    bool writeVideo = parser.has("ov");
    std::string outputFile;
    cv::VideoWriter videoWriter;
    if(writeVideo) {
        outputFile = parser.get<std::string>("ov");
        std::cout << "Write video file.\n";
        bool status = videoWriter.open(outputFile, cv::VideoWriter::fourcc('M', 'P', '4', 'V'), 30, cv::Size(1920, 1080), true);
        if(!status) {
            std::cout << "Video writer not initialized.\n";
            return -1;
        }
    }

    if(!parser.check()) {
        parser.printErrors();
        return -1;
    }

    #if defined(WIN32) || defined(_WIN32) || defined(__WIN32__)
    if(!SetConsoleCtrlHandler(CtrlHandler, TRUE)) {
        std::cout << "Could not set control handler\n";
        return -1;
    }
    #endif

    cv::Mat frame;
	td::TransferData transfer(pixelResolution);
	ArucoLocalization cv_system(dictionary_name);
    cv_system.setFrameSize(frame_width, frame_height);
	steady_clock::time_point start_time = steady_clock::now();
    steady_clock::time_point current_time = steady_clock::now();
    long long time = duration_cast<milliseconds>(current_time - start_time).count();
    long long prev_time = time;
    while ((time <= test_duration) || (test_duration == 0)) {
        videoCapture >> frame;
        if(frame.empty()) {
            std::cout << "End of video file.\n";
            break;
        }
        current_time = steady_clock::now();
        time = duration_cast<milliseconds>(current_time - start_time).count();
        if(cv_system.detectMarkers(frame)) {
            if(has_marker_id) {
                if(!cv_system.estimatePosition(&transfer, markerID)) {
                    std::cout << "Marker with ID " << markerID << "not found.\n";
                    break;
                }
            }
            else {
                cv_system.estimatePosition(&transfer);
            }
            if (shm_flag) {
                transmitter.data->at(0) = static_cast<float>(time);
                transmitter.data->at(1) = transfer.currGlobalCartesian.x;
                transmitter.data->at(2) = transfer.currGlobalCartesian.y;
                transmitter.data->at(3) = transfer.currAngle;
                transmitter.data->at(4) = transfer.deltaEigenCartesian.x;
                transmitter.data->at(5) = transfer.deltaEigenCartesian.y;
                transmitter.data->at(6) = transfer.deltaAngle;
            }
            else {
                float dt = static_cast<float>(time - prev_time) / 1000;
                std::cout << " | " << std::setw(15) << time;
                std::cout << " | " << std::setw(15) << transfer.currGlobalCartesian.x;
                std::cout << " | " << std::setw(15) << transfer.currGlobalCartesian.y;
                std::cout << " | " << std::setw(15) << transfer.currAngle;
                std::cout << " | " << std::setw(15) << transfer.deltaEigenCartesian.x / dt;
                std::cout << " | " << std::setw(15) << transfer.deltaEigenCartesian.y / dt;
                std::cout << " | " << std::setw(15) << transfer.deltaAngle / dt << " |\n";
            }
            if(writeVideo) {
                videoWriter.write(frame);
            }
        }
        else {
            std::cout << "Robot localization failed.\n";
            cv::imshow("Camera frame", frame);
            cv::waitKey();
            break;
        }
        prev_time = time;
        #if defined(WIN32) || defined(_WIN32) || defined(__WIN32__)
        if(isExit) {
            break;
        }
        #endif
    }
    videoCapture.release();
    if(writeVideo) {
        videoWriter.release();
    }
	return 0;
}

#if defined(WIN32) || defined(_WIN32) || defined(__WIN32__)
BOOL WINAPI CtrlHandler(DWORD fdwCtrlType) {
    switch (fdwCtrlType) {
    case CTRL_C_EVENT:
    case CTRL_CLOSE_EVENT:
    case CTRL_LOGOFF_EVENT:
    case CTRL_SHUTDOWN_EVENT:
        isExit = TRUE;
        return TRUE;
    default:
        return FALSE;
    }
}
#endif