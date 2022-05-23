#include "arucolocalization.hpp"
#include "cmdoptionparser.hpp"
#include "sharedmemory.hpp"
#include <chrono>
#include <iomanip>

int main( int argc, char **argv ) {

    // Index webcam
    int cam_idx = 0;
    // Test duration
    double test_duration = 60000;
    // Shared memory flag
    bool shm_flag = false;
    // Test flag
    bool test_flag = false;

    // Process command line options.
    if (argc > 1){
        // Help option
        if (cmdOptionExists(argv, argv+argc, "--help")) {
            std::cout << "usage: localization [options]\n\nOptions:\n\n"
                      << std::left << "  " << std::setw(15) << "--help" << "Display this information.\n"
                      << "  " << std::setw(15) << "-cam <index>" << "Use webcamera with <index> in system.\n"
                      << "  " << std::setw(15) << "-t <duration>" << "Set <duration> of program execution in ms.\n"
                      << "  " << std::setw(15) << "-shared-memory" << "Use shared memory from boost to transfer measurements.\n"
                      << "  " << std::setw(15) << "-test" << "Test cam. Show frame with aruco marker detection.\n\n";
            return 0;
        }
        // Set webcam index option
        if (cmdOptionExists(argv, argv+argc, "-cam")){
            cam_idx = atoi(getCmdOption(argv, argv+argc, "-cam"));
        }
        // Set test duration option
        if (cmdOptionExists(argv, argv+argc, "-t")){
            test_duration = atof(getCmdOption(argv, argv+argc, "-t"));
        }
        // Shared memory flag
        shm_flag = cmdOptionExists(argv, argv+argc, "-shared-memory");
        // Test flag
        test_flag = cmdOptionExists(argv, argv+argc, "-test");
    }

    shm::Transmitter<double> transmitter;
    if(shm_flag) {
        transmitter.create("CameraData", 1000);
        //Initialization shared memory vector
        for (size_t i = 0; i < 7; i++)
        {
            transmitter.data->push_back(0);
        }    
    }

	td::TransferData transfer;
	ArucoLocalization cv_system(cam_idx, cv::aruco::DICT_4X4_50);
    if(test_flag) {
        bool status = cv_system.localizate(&transfer);
        if(status) {
            cv_system.show_markers();
            cv::waitKey();
        }
        else {
            cv_system.show_frame();
	        cv::waitKey();
            std::cout << "Robot localization failed." << std::endl;
        }
        return 0;
    }
	std::chrono::steady_clock::time_point start_time = std::chrono::steady_clock::now();
    std::chrono::steady_clock::time_point current_time = std::chrono::steady_clock::now();
    long long time = std::chrono::duration_cast<std::chrono::milliseconds>(current_time - start_time).count();
    while (time <= test_duration) {
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
            std::cout << "|" << std::setw(15) << time;
            std::cout << "|" << std::setw(15) << transfer.currGlobalCartesian.x;
            std::cout << "|" << std::setw(15) << transfer.currGlobalCartesian.y;
            std::cout << "|" << std::setw(15) << transfer.currAngle;
            std::cout << "|" << std::setw(15) << transfer.deltaEigenCartesian.x;
            std::cout << "|" << std::setw(15) << transfer.deltaEigenCartesian.y;
            std::cout << "|" << std::setw(15) << transfer.deltaAngle << "|\n";
        }
        else {
            break;
        }
        current_time = std::chrono::steady_clock::now();
        time = std::chrono::duration_cast<std::chrono::milliseconds>(current_time - start_time).count();
    }
	return 0;
}