#include "arucolocalization.hpp"
#include <opencv2/core/utility.hpp>
#include <iomanip>
#include <chrono>

const std::string about = "Localization of Aruco marker.";
const std::string keys  =
        "{h help ? usage |       | Print help message}"
        "{d              |       | dictionary: DICT_4X4_50=0, DICT_4X4_100=1, DICT_4X4_250=2,"
        "DICT_4X4_1000=3, DICT_5X5_50=4, DICT_5X5_100=5, DICT_5X5_250=6, DICT_5X5_1000=7, "
        "DICT_6X6_50=8, DICT_6X6_100=9, DICT_6X6_250=10, DICT_6X6_1000=11, DICT_7X7_50=12,"
        "DICT_7X7_100=13, DICT_7X7_250=14, DICT_7X7_1000=15, DICT_ARUCO_ORIGINAL = 16,"
        "DICT_APRILTAG_16h5=17, DICT_APRILTAG_25h9=18, DICT_APRILTAG_36h10=19, DICT_APRILTAG_36h11=20}"
        "{ci       | 0     | Camera id if input doesnt come from video (-v) }";

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
        return 0;
    }

    cv::VideoCapture video_capture;
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
    video_capture.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));

    //Checking for the video_capture to be connected 
    if (video_capture.isOpened()) {
        std::cout << "video_capture connected." << std::endl;
    }
    else {
        std::cout << "video_capture not connected." << std::endl;
        exit(1);
    }

    if(!parser.check()) {
        parser.printErrors();
        return 0;
    }

	ArucoLocalization cv_system(video_capture, dictionary_name);
    std::chrono::steady_clock::time_point start_time = std::chrono::steady_clock::now();
    std::chrono::steady_clock::time_point current_time = std::chrono::steady_clock::now();
    long long time = std::chrono::duration_cast<std::chrono::milliseconds>(current_time - start_time).count();
    while(true) {
        start_time = std::chrono::steady_clock::now();
        bool status = cv_system.localizate();
        current_time = std::chrono::steady_clock::now();
        time = std::chrono::duration_cast<std::chrono::milliseconds>(current_time - start_time).count();
        std::cout << "Time process one frame: " << time << "ms\n";
        if(status) {
            cv_system.show_markers();
            if(cv::pollKey() == 'q') {
                break;
            }
        }
        else {
            cv_system.show_frame();
            cv::waitKey();
            std::cout << "Robot localization failed." << std::endl;
            return 1;
        }
    }
    video_capture.release();
	return 0;
}