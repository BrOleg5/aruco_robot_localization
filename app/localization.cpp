#include "arucolocalization.hpp"
#include "boost/interprocess/shared_memory_object.hpp"
#include "boost/interprocess/mapped_region.hpp"
#include <chrono>
#include <time.h>
#include <iomanip>
#include <cstring>
#include <cstdlib>
#include <string>

int main() {
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

    //Write all the memory to 1
    std::memset(region.get_address(), 1, region.get_size());

	td::TransferData transfer;
	ArucoLocalization cv_system(2, cv::aruco::DICT_4X4_50);
    unsigned int count = 0;
	auto timePoint1 = std::chrono::steady_clock::now();
	double measure_time = 0;
	while (true) {
		std::cout << "Enter measurement time in sec or enter 0 to exit." << std::endl;
		std::cin >> measure_time;
		if (measure_time == 0) {
			break;
		}
		double start_time = getTime(timePoint1);
		while ((getTime(timePoint1) - start_time) <= measure_time) {
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
				std::cout << "|" << std::setw(15) << getTime(timePoint1) - start_time;
				std::cout << "|" << std::setw(15) << transfer.currGlobalCartesian.x;
				std::cout << "|" << std::setw(15) << transfer.currGlobalCartesian.y;
				std::cout << "|" << std::setw(15) << transfer.currAngle;
				std::cout << "|" << std::setw(15) << transfer.deltaEigenCartesian.x;
				std::cout << "|" << std::setw(15) << transfer.deltaEigenCartesian.y;
				std::cout << "|" << std::setw(15) << transfer.deltaAngle << "|" << std::endl;
                count++;
			}
		}
	}
	return 0;
}