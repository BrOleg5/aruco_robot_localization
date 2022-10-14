#include "arucolocalization.hpp"
#include <opencv2/core/utility.hpp>
#include "sharedmemory.hpp"
#include <chrono>
#include <iomanip>
#include "read_save_camera_parameters.hpp"

const std::string about = "Localization of Aruco marker.";
const std::string keys  =
        "{h help ? usage |       | Print help message}"
        "{d              |       | dictionary: DICT_4X4_50=0, DICT_4X4_100=1, DICT_4X4_250=2,"
        "DICT_4X4_1000=3, DICT_5X5_50=4, DICT_5X5_100=5, DICT_5X5_250=6, DICT_5X5_1000=7, "
        "DICT_6X6_50=8, DICT_6X6_100=9, DICT_6X6_250=10, DICT_6X6_1000=11, DICT_7X7_50=12,"
        "DICT_7X7_100=13, DICT_7X7_250=14, DICT_7X7_1000=15, DICT_ARUCO_ORIGINAL = 16,"
        "DICT_APRILTAG_16h5=17, DICT_APRILTAG_25h9=18, DICT_APRILTAG_36h10=19, DICT_APRILTAG_36h11=20}"
        "{id             |       | Marker id, if ommited, detect all markers from dictionaty }"
        "{ci             |       | Camera id, if ommited, input comes from video file }"
        "{v              |       | Input from video file if input doesnt come from camera (--ci) }"
        "{t              | 0     | Program execution time in ms. If it equals 0, application run until user stop.}"
        "{cp             |       | JSON file with camera parameters }"
        "{shm            |       | Use shared memory to transmit data to other programs}";

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
        for (size_t i = 0; i < 7; i++)
        {
            transmitter.data->push_back(0.0f);
        }
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
    if (!readCameraParameters(camParamFile, pixelResolution))
    {
        std::cerr << "Read camera parameters error.\n";
        return 7;
    }
    std::cout << "Camera parameters:\n\tpixel resolution x: " << pixelResolution.x 
            << "\n\tpixel resolution y: " << pixelResolution.y << '\n';
    // validate data
    if((pixelResolution.x == 0) || (pixelResolution.y == 0)) {
        std::cerr << "Get invalid camera parameters.\n";
        return 8;
    }
    

    if(!parser.check()) {
        parser.printErrors();
        return 4;
    }

	td::TransferData transfer(pixelResolution);
	ArucoLocalization cv_system(video_capture, dictionary_name);
	std::chrono::steady_clock::time_point start_time = std::chrono::steady_clock::now();
    std::chrono::steady_clock::time_point current_time = std::chrono::steady_clock::now();
    long long time = std::chrono::duration_cast<std::chrono::milliseconds>(current_time - start_time).count();
    long long prev_time = time;
    while ((time <= test_duration) || (test_duration == 0)) {
        current_time = std::chrono::steady_clock::now();
        time = std::chrono::duration_cast<std::chrono::milliseconds>(current_time - start_time).count();
        int status = cv_system.detectMarkers();
        if (status == 0) {
            if(has_marker_id){
                if(!cv_system.estimatePosition(&transfer, markerID)) {
                    return 5;
                }
            }
            else {
                cv_system.estimatePosition(&transfer);
            }
            if (shm_flag){
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
        }
        else if(status == 1) {
            std::cout << "Robot localization failed." << std::endl;
            video_capture.release();
            return 6;
        }
        else if(status == 2) {
            break;
        }
        prev_time = time;
    }
    video_capture.release();
	return 0;
}