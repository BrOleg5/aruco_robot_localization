#include "arucolocalization.hpp"
#include "read_save_camera_parameters.hpp"
#include <cmath>
#include <opencv2/core/types.hpp>
#include <opencv2/core/persistence.hpp>
#include <opencv2/core/utility.hpp>

bool readCoordinates(std::string filename, cv::Point3f* frameCoordinate, int frame_num);

const std::string about = "Global coordinate test.";
const std::string keys  =
        "{h help ? usage |       | Print help message}"
        "{d              |       | dictionary: DICT_4X4_50=0, DICT_4X4_100=1, DICT_4X4_250=2,"
        "DICT_4X4_1000=3, DICT_5X5_50=4, DICT_5X5_100=5, DICT_5X5_250=6, DICT_5X5_1000=7, "
        "DICT_6X6_50=8, DICT_6X6_100=9, DICT_6X6_250=10, DICT_6X6_1000=11, DICT_7X7_50=12,"
        "DICT_7X7_100=13, DICT_7X7_250=14, DICT_7X7_1000=15, DICT_ARUCO_ORIGINAL = 16,"
        "DICT_APRILTAG_16h5=17, DICT_APRILTAG_25h9=18, DICT_APRILTAG_36h10=19, DICT_APRILTAG_36h11=20}"
        "{id             |       | Marker id, if ommited, detect all markers from dictionaty }"
        "{img            |       | Path to test images }"
        "{cp             |       | JSON file with camera parameters }";

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
        return 1;
    }

    cv::VideoCapture video_capture;
    std::string testImagesPath;
    if(parser.has("img")){
        testImagesPath = parser.get<std::string>("img");
        std::string images = testImagesPath + std::string("/test_frame_%1d.jpg");
        video_capture.open(images, cv::CAP_IMAGES);
    }
    else {
        std::cout << "Test images path not specified.\n";
        return 3;
    }

    int markerID = 0;
    if(parser.has("id")) {
        markerID = parser.get<int>("id");
    }
    else {
        std::cout << "Marker ID not specified.\n";
        return 4;
    }

    std::string camParamFile;
    if(parser.has("cp")){
        camParamFile = parser.get<std::string>("cp");
    }
    else {
        std::cout << "File of camera parameters not specified.\n";
        return 5;
    }
    cv::Point2f pixelResolution;
    if (!readCameraParameters(camParamFile, pixelResolution)) {
        std::cout << "Read camera parameters error.\n";
        return 6;
    }
    // validate data
    if((pixelResolution.x == 0) || (pixelResolution.y == 0)) {
        std::cout << "Get invalid camera parameters.\n";
        return 7;
    }

    const int frame_num = 5;
    cv::Point3f frameCoordinate[frame_num];
    std::string frameCoordinateFilePath = testImagesPath + "/frame_coordinate.json";
    if(!readCoordinates(frameCoordinateFilePath, frameCoordinate, frame_num)) {
        std::cout << "Read coordinate error.\n";
        return 8;
    }

    float coord_treshold = 1;
    float angle_treshold = 1;

	td::TransferData transfer(pixelResolution);
	ArucoLocalization cv_system(video_capture, dictionary_name);
    for (int i = 0; i < frame_num; i++) {
        int status = cv_system.detectMarkers();
        if (status == 0) {
            if(!cv_system.estimatePosition(&transfer, markerID)) {
                return 9;
            }
            if ((std::abs(transfer.currGlobalCartesian.x - frameCoordinate[i].x) > coord_treshold) ||
                (std::abs(transfer.currGlobalCartesian.y - frameCoordinate[i].y) > coord_treshold) ||
                (std::abs(transfer.currAngle - frameCoordinate[i].z) > angle_treshold)) {
                std::cout << "Test failed.\n";
                return 10;
            }
        }
        else if(status == 1) {
            std::cout << "Robot localization failed." << std::endl;
            video_capture.release();
            return 11;
        }
        else if(status == 2) {
            break;
        }
    }
    video_capture.release();
    std::cout << "Test success.\n";
	return 0;
}

bool readCoordinates(std::string filename, cv::Point3f* frameCoordinate, int frame_num) {
    cv::FileStorage fs(filename, cv::FileStorage::READ);
    if (!fs.isOpened()) {
        return false;
    }
    for (int i = 0; i < frame_num; i++) {
        std::string key = std::string("test_frame_") + std::to_string(i) + std::string(".jpg");
        fs[key]["x"] >> frameCoordinate[i].x;
        fs[key]["y"] >> frameCoordinate[i].y;
        fs[key]["ang"] >> frameCoordinate[i].z;
    }
    fs.release();
    return true;
}
