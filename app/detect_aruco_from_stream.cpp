#include "arucolocalization.hpp"
#include "cmdoptionparser.hpp"
#include <iomanip>

int main( int argc, char **argv ) {

    // Index webcam
    int cam_idx = 0;

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

	ArucoLocalization cv_system(cam_idx, cv::aruco::DICT_6X6_50);
    while(true) {
        bool status = cv_system.localizate();
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