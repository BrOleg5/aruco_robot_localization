#include <chrono>
#include <iomanip>
#include "arucolocalization.hpp"

int main() {
	td::TransferData transfer;
	ArucoLocalization cv_system(0, cv::aruco::DICT_4X4_50);
	double measure_time = 0;
	while (true) {
		std::cout << "Enter measurement time in sec or enter 0 to exit." << std::endl;
		std::cin >> measure_time;
		if (measure_time == 0) {
			break;
		}
		std::chrono::steady_clock::time_point start_time = std::chrono::steady_clock::now();
		std::chrono::steady_clock::time_point current_time = std::chrono::steady_clock::now();
		while (std::chrono::duration_cast<std::chrono::seconds>(current_time - start_time).count() <= measure_time) {
			bool status = cv_system.localizate(&transfer);
			//cv_system.show_markers();
			if (status) {
				std::cout << "|" << std::setw(15) << std::chrono::duration_cast<std::chrono::milliseconds>(current_time - start_time).count();
				std::cout << "|" << std::setw(15) << transfer.currGlobalCartesian.x;
				std::cout << "|" << std::setw(15) << transfer.currGlobalCartesian.y;
				std::cout << "|" << std::setw(15) << transfer.currAngle;
				std::cout << "|" << std::setw(15) << transfer.deltaEigenCartesian.x;
				std::cout << "|" << std::setw(15) << transfer.deltaEigenCartesian.y;
				std::cout << "|" << std::setw(15) << transfer.deltaAngle << "|" << std::endl;
			}
			current_time = std::chrono::steady_clock::now();
		}
	}
	return 0;
}