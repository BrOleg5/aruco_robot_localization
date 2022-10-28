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
        "{id             |       | Marker id, if ommited, detect all markers from dictionaty }"
        "{ci             |       | Camera id, if ommited, input comes from video file }"
        "{iv             |       | Input from video file if input doesnt come from camera (--ci) }"
        "{ov             |       | Input from video file if input doesnt come from camera (--ci) }"
        "{ce             | 0     | Camera exposure }";

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
        if(parser.has("ce")) {
            video_capture.set(cv::CAP_PROP_AUTO_EXPOSURE, 0);
            video_capture.set(cv::CAP_PROP_EXPOSURE, parser.get<double>("ce"));
        }
        else {
            video_capture.set(cv::CAP_PROP_AUTO_EXPOSURE, 1);
        }

        //Checking for the camera to be connected 
        if (video_capture.isOpened()) {
            std::cout << "Camera connected." << std::endl;
        }
        else {
            std::cerr << "Camera not connected." << std::endl;
            return 2;
        }
    }
    else if(parser.has("iv")){
        std::string videoFile = parser.get<std::string>("iv");
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

    bool write_video = parser.has("ov");
    cv::VideoWriter video_writer;
    if(write_video) {
        std::string output_file = parser.get<std::string>("ov");
        video_writer = cv::VideoWriter(output_file, -1, 30, cv::Size(1920, 1080));
    }

    if(!parser.check()) {
        parser.printErrors();
        return 4;
    }

	ArucoLocalization cv_system(video_capture, dictionary_name);
    cv::Mat frame;
    std::chrono::steady_clock::time_point start_time = std::chrono::steady_clock::now();
    std::chrono::steady_clock::time_point current_time = std::chrono::steady_clock::now();
    long long time = std::chrono::duration_cast<std::chrono::milliseconds>(current_time - start_time).count();
    while(true) {
        start_time = std::chrono::steady_clock::now();
        int status = cv_system.detectMarkers();
        current_time = std::chrono::steady_clock::now();
        time = std::chrono::duration_cast<std::chrono::milliseconds>(current_time - start_time).count();
        std::cout << "Time process one frame: " << time << " ms\n";
        if(status == ArucoLocalization::Status::OK) {
            if(has_marker_id) {
                frame = cv_system.draw_marker(markerID);
                if(write_video) {
                    video_writer.write(frame);
                }
                cv::imshow("Found aruco marker", frame);
            }
            else {
                cv_system.show_markers();
            }
            if(cv::pollKey() == 'q') {
                break;
            }
        }
        else if(status == ArucoLocalization::Status::MARKER_NOT_DETECTED){
            frame = cv_system.get_frame();
            cv::imshow("Found aruco marker", frame);
            cv::waitKey();
            std::cout << "Cam frame" << std::endl;
            video_capture.release();
            return 5;
        }
        else if(status == ArucoLocalization::Status::END_OF_VIDEO_FILE){
            break;
        }
    }
    video_capture.release();
	return 0;
}