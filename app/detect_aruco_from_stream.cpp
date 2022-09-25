#include "arucolocalization.hpp"
#include "cmdoptionparser.hpp"
#include <iomanip>
#include <chrono>

int main( int argc, char **argv ) {

    // Index webcam
    int cam_idx = 1;

    // Process command line options.
    if (argc > 1){
        // Help option
        if (cmdOptionExists(argv, argv+argc, "--help")) {
            std::cout << "usage: detect_aruco_from_stream [options]\n\nOptions:\n\n"
                      << std::left << "  " << std::setw(15) << "--help" << "Display this information.\n"
                      << "  " << std::setw(15) << "-cam <index>" << "Use web camera with <index> in system.\n\n";
            return 0;
        }
        // Set webcam index option
        if (cmdOptionExists(argv, argv+argc, "-cam")){
            cam_idx = atoi(getCmdOption(argv, argv+argc, "-cam"));
        }
    }

	ArucoLocalization cv_system(cam_idx, cv::aruco::DICT_4X4_50);
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
	return 0;
}