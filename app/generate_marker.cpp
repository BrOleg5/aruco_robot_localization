#include "opencv2/aruco.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include <iostream>
#include <iomanip>
#include "cmdoptionparser.hpp"

int main( int argc, char **argv ) {
    int marker_id = 0;
    int img_size = 100;
    int border_size = 1;
    // Process command line options.
    if (argc > 1){
        // Help option
        if (cmdOptionExists(argv, argv+argc, "--help")) {
            std::cout << "usage: generate_marker [options]\n\nOptions:\n\n"
                      << std::left << "  " << std::setw(15) << "--help" << "Display this information.\n"
                      << "  " << std::setw(15) << "-id <marker_id>" << "Marker id from DICT_4X4_50.\n"
                      << "  " << std::setw(15) << "-size <pix>" << "Size of the image in pixels.\n"
                      << "  " << std::setw(15) << "-border-bits <bord_bits>" << "Width of the marker border.\n\n";
            return 0;
        }
        // Set marker id
        if (cmdOptionExists(argv, argv+argc, "-id")){
            marker_id = atoi(getCmdOption(argv, argv+argc, "-id"));
        }
        // Set test duration option
        if (cmdOptionExists(argv, argv+argc, "-size")){
            img_size = atoi(getCmdOption(argv, argv+argc, "-size"));
        }
        // 
        if (cmdOptionExists(argv, argv+argc, "-border-bits")){
            border_size = atoi(getCmdOption(argv, argv+argc, "-border-bits"));
        }
    }

    cv::Mat markerImage;
    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
    cv::aruco::drawMarker(dictionary, marker_id, img_size, markerImage, border_size);
    cv::imwrite("marker.png", markerImage);
    return 0;
}