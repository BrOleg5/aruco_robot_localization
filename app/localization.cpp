#include "arucolocalization.hpp"
#include <opencv2/core/utility.hpp>
#include "sharedmemory.hpp"
#include <chrono>
#include <iomanip>

const std::string about = "Localization of Aruco marker.";
const std::string keys  =
        "{h help ? usage |       | Print help message}"
        "{d              |       | dictionary: DICT_4X4_50=0, DICT_4X4_100=1, DICT_4X4_250=2,"
        "DICT_4X4_1000=3, DICT_5X5_50=4, DICT_5X5_100=5, DICT_5X5_250=6, DICT_5X5_1000=7, "
        "DICT_6X6_50=8, DICT_6X6_100=9, DICT_6X6_250=10, DICT_6X6_1000=11, DICT_7X7_50=12,"
        "DICT_7X7_100=13, DICT_7X7_250=14, DICT_7X7_1000=15, DICT_ARUCO_ORIGINAL = 16,"
        "DICT_APRILTAG_16h5=17, DICT_APRILTAG_25h9=18, DICT_APRILTAG_36h10=19, DICT_APRILTAG_36h11=20}"
        "{ci             | 0     | Camera id}"
        "{t              | 10000 | Program execution time in ms}"
        "{shm            |       | Use shared memory to transmit data to other programs}";

int main( int argc, char **argv ) {
    cv::CommandLineParser parser(argc, argv, keys);
    parser.about(about);

    if(parser.has("h") || parser.has("help") || parser.has("?") || parser.has("usage")) {
        parser.printMessage();
        return 0;
    }

    // Index webcam
    int cam_idx = parser.get<int>("ci");
    // Test duration
    double test_duration = parser.get<double>("t");
    // Shared memory flag
    bool shm_flag = parser.has("shm");

    shm::Transmitter<double> transmitter;
    if(shm_flag) {
        transmitter.create("CameraData", 1000);
        //Initialization shared memory vector
        for (size_t i = 0; i < 7; i++)
        {
            transmitter.data->push_back(0);
        }    
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

    if(!parser.check()) {
        parser.printErrors();
        return 0;
    }

	td::TransferData transfer;
	ArucoLocalization cv_system(cam_idx, dictionary_name);
	std::chrono::steady_clock::time_point start_time = std::chrono::steady_clock::now();
    std::chrono::steady_clock::time_point current_time = std::chrono::steady_clock::now();
    long long time = std::chrono::duration_cast<std::chrono::milliseconds>(current_time - start_time).count();
    while (time <= test_duration) {
        current_time = std::chrono::steady_clock::now();
        time = std::chrono::duration_cast<std::chrono::milliseconds>(current_time - start_time).count();
        bool status = cv_system.localizate(&transfer);
        if (status) {
            if (shm_flag){
                transmitter.data->at(0) = (double) time;
                transmitter.data->at(1) = transfer.currGlobalCartesian.x;
                transmitter.data->at(2) = transfer.currGlobalCartesian.y;
                transmitter.data->at(3) = transfer.currAngle;
                transmitter.data->at(4) = transfer.deltaEigenCartesian.x;
                transmitter.data->at(5) = transfer.deltaEigenCartesian.y;
                transmitter.data->at(6) = transfer.deltaAngle;
            }
            else {
                std::cout << "|" << std::setw(15) << time;
                std::cout << "|" << std::setw(15) << transfer.currGlobalCartesian.x;
                std::cout << "|" << std::setw(15) << transfer.currGlobalCartesian.y;
                std::cout << "|" << std::setw(15) << transfer.currAngle;
                std::cout << "|" << std::setw(15) << transfer.deltaEigenCartesian.x;
                std::cout << "|" << std::setw(15) << transfer.deltaEigenCartesian.y;
                std::cout << "|" << std::setw(15) << transfer.deltaAngle << "|\n";
            }
        }
        else {
            break;
        }
    }
	return 0;
}