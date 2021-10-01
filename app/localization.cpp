#include "arucolocalization.hpp"
#include "cmdoptionparser.hpp"
#include "boost/interprocess/shared_memory_object.hpp"
#include "boost/interprocess/mapped_region.hpp"
#include <chrono>
#include <time.h>
#include <iomanip>
#include <cstring>
#include <cstdlib>
#include <string>

int main( int argc, char **argv ) {

    //Index webcam
    int cam_idx = 2;
    float test_duration = 1;

    // Process command line options.
    if (argc > 1){
        // Help option
        if (cmdOptionExists(argv, argv+argc, "--help")) {
            std::cout << "usage: localization [options]\n\n Options:\n\n"
                      << std::left << "  " << std::setw(15) << "--help" << "Display this information.\n"
                      << "  " << std::setw(15) << "-cam <index>" << "Use webcamera with <index> in system.\n"
                      << "  " << std::setw(15) << "-t <duration>" << "Set <duration> of program execution in ms.\n\n";
            return 0;
        }
        // Set webcam index option
        if (cmdOptionExists(argv, argv+argc, "-cam")){
            cam_idx = atoi(getCmdOption(argv, argv+argc, "-cam"));
        }
        // Set test duration option
        if (cmdOptionExists(argv, argv+argc, "-t")){
            test_duration = atof(getCmdOption(argv, argv+argc, "-t")) / 1000;
        }
    }

    //Remove shared memory on construction and destruction
    struct shm_remove
    {
        shm_remove() { boost::interprocess::shared_memory_object::remove("Robotino4OpenCV"); }
        ~shm_remove(){ boost::interprocess::shared_memory_object::remove("Robotino4OpenCV"); }
    } remover;

    //Create a shared memory object.
    boost::interprocess::shared_memory_object shm (boost::interprocess::create_only, "Robotino4OpenCV", boost::interprocess::read_write);

    //Set size
    shm.truncate(1000);

    //Map the whole shared memory in this process
    boost::interprocess::mapped_region region(shm, boost::interprocess::read_write);

	td::TransferData transfer;
	ArucoLocalization cv_system(cam_idx, cv::aruco::DICT_4X4_50);
    unsigned int count = 0;
	auto timePoint1 = std::chrono::steady_clock::now();
    while (getTime(timePoint1) <= test_duration) {
        bool status = cv_system.localizate(&transfer);
        if (status) {
            unsigned long shift = 0;
            uint8_t* start_point = (uint8_t*) region.get_address();
            std::memcpy(start_point, &count, sizeof(count));
            shift += sizeof(count);
            std::memcpy(start_point + shift, &transfer.currGlobalCartesian.x, sizeof(transfer.currGlobalCartesian.x));
            shift += sizeof(transfer.currGlobalCartesian.x);
            std::memcpy(start_point + shift, &transfer.currGlobalCartesian.y, sizeof(transfer.currGlobalCartesian.y));
            shift += sizeof(transfer.currGlobalCartesian.y);
            std::memcpy(start_point + shift, &transfer.currAngle, sizeof(transfer.currAngle));
            shift += sizeof(transfer.currAngle);
            std::memcpy(start_point + shift, &transfer.deltaEigenCartesian.x, sizeof(transfer.deltaEigenCartesian.x));
            shift += sizeof(transfer.deltaEigenCartesian.x);
            std::memcpy(start_point + shift, &transfer.deltaEigenCartesian.y, sizeof(transfer.deltaEigenCartesian.y));
            shift += sizeof(transfer.deltaEigenCartesian.y);
            std::memcpy(start_point + shift, &transfer.deltaAngle, sizeof(transfer.deltaAngle));
            shift += sizeof(transfer.deltaAngle);
            std::cout << "|" << std::setw(15) << getTime(timePoint1);
            std::cout << "|" << std::setw(15) << transfer.currGlobalCartesian.x;
            std::cout << "|" << std::setw(15) << transfer.currGlobalCartesian.y;
            std::cout << "|" << std::setw(15) << transfer.currAngle;
            std::cout << "|" << std::setw(15) << transfer.deltaEigenCartesian.x;
            std::cout << "|" << std::setw(15) << transfer.deltaEigenCartesian.y;
            std::cout << "|" << std::setw(15) << transfer.deltaAngle << "|" << std::endl;
            count++;
        }
    }
	return 0;
}