#include "arucolocalization.hpp"
#include <opencv2/core/utility.hpp>
#include <iomanip>
#include <chrono>

using namespace std::chrono;

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
        std::cout << "Dictionary not specified" << std::endl;
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
        videoCapture.set(cv::CAP_PROP_FOCUS, 0); // min: 0, max: 255, increment:5
        videoCapture.set(cv::CAP_PROP_AUTO_EXPOSURE, 1);
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
    else if(parser.has("iv")){
        std::string videoFile = parser.get<std::string>("iv");
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

    int markerID = -1;
    if(parser.has("id")) {
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
        return -1;
    }

	ArucoLocalization cv_system(dictionary_name);
    cv::Mat frame;
    steady_clock::time_point start_time = steady_clock::now();
    steady_clock::time_point current_time = steady_clock::now();
    long long time = duration_cast<milliseconds>(current_time - start_time).count();
    while(true) {
        videoCapture >> frame;
        if(frame.empty()) {
            std::cout << "End of video file.\n";
            break;
        }
        start_time = steady_clock::now();
        bool status = cv_system.detectMarkers(frame);
        current_time = steady_clock::now();
        time = duration_cast<milliseconds>(current_time - start_time).count();
        std::cout << "Time process one frame: " << time << " ms\r";
        if(status) {
            if(!cv_system.draw_marker(frame, markerID)) {
                std::cout << "Marker with ID " << markerID << "not found.\n";
                break;
            }
            if(write_video) {
                video_writer.write(frame);
            }
            cv::imshow("Found aruco markers", frame);
            if(cv::pollKey() == 'q') {
                break;
            }
        }
        else {
            std::cout << "Aruco markers not found.\n";
            cv::imshow("Camera frame", frame);
            cv::waitKey();
            break;
        }
    }
    videoCapture.release();
    if(write_video) {
        video_writer.release();
    }
	return 0;
}