#include <chrono>
#include <time.h>
#include <iomanip>
#include "arucolocalization.h"

int main() {
	td::TransferData transfer;
	ArucoLocalization cv_system(2, cv::aruco::DICT_4X4_50);
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
			//cv_system.show_markers();
			if (status) {
				std::cout << "|" << std::setw(15) << getTime(timePoint1) - start_time;
				std::cout << "|" << std::setw(15) << transfer.currGlobalCartesian.x;
				std::cout << "|" << std::setw(15) << transfer.currGlobalCartesian.y;
				std::cout << "|" << std::setw(15) << transfer.currAngle;
				std::cout << "|" << std::setw(15) << transfer.deltaEigenCartesian.x;
				std::cout << "|" << std::setw(15) << transfer.deltaEigenCartesian.y;
				std::cout << "|" << std::setw(15) << transfer.deltaAngle << "|" << std::endl;
			}
		}
	}
	return 0;
}