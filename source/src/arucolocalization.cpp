#include "arucolocalization.hpp"

void td::TransferData::Angle(cv::Point2d* arucoCorner) {
	if (arucoCorner[1].x >= arucoCorner[2].x) {
		if (arucoCorner[1].y >= arucoCorner[2].y) {
			// if in [0 ; 90] degrees, then               
			currAngle = (double)270 - std::acos(abs(arucoCorner[1].x - arucoCorner[2].x) / sqrt(pow(arucoCorner[1].x - arucoCorner[2].x, 2) + pow(arucoCorner[1].y - arucoCorner[2].y, 2))) * (double)180 / (double)PI;
		}
		else {
			// if in [270 ; 360] degrees, then            
			currAngle = -90 + acos(abs(arucoCorner[1].x - arucoCorner[2].x) / sqrt(pow(arucoCorner[1].x - arucoCorner[2].x, 2) + pow(arucoCorner[1].y - arucoCorner[2].y, 2))) * (double)180 / (double)PI;
		}
	}
	else {
		if (arucoCorner[1].y >= arucoCorner[2].y) {
			// if in [90 ; 180] degrees, then             
			currAngle = (double)90 + acos(abs(arucoCorner[1].x - arucoCorner[2].x) / sqrt(pow(arucoCorner[1].x - arucoCorner[2].x, 2) + pow(arucoCorner[1].y - arucoCorner[2].y, 2))) * (double)180 / (double)PI;
		}
		else {
			// if in [180 ; 270] degrees, then            
			currAngle = (double)90 - acos(abs(arucoCorner[1].x - arucoCorner[2].x) / sqrt(pow(arucoCorner[1].x - arucoCorner[2].x, 2) + pow(arucoCorner[1].y - arucoCorner[2].y, 2))) * (double)180 / (double)PI;
		}
	}
	if (currAngle < 0) {
		currAngle = 360 + currAngle;
	}
	if (abs(currAngle - prevAngle) > 160) {
		if (currAngle > prevAngle) {
			deltaAngle = -1 * (360 - currAngle + prevAngle);
		}
		else {
			deltaAngle = 360 - prevAngle + currAngle;
		}
	}
	else {
		deltaAngle = currAngle - prevAngle;
	}
}

void td::TransferData::DeltaEigen() {
	cv::Point2d delta;
	delta.x = (currGlobalCartesian.x - prevGlobalCartesian.x) * MMperPIX_X;
	delta.y = (currGlobalCartesian.y - prevGlobalCartesian.y) * MMperPIX_Y;
	deltaEigenCartesian.x = delta.x * sin(currAngle / 180 * PI) * (-1) - delta.y * cos(currAngle / 180 * PI);
	deltaEigenCartesian.y = delta.x * cos(currAngle / 180 * PI) * (-1) + delta.y * sin(currAngle / 180 * PI);
}

ArucoLocalization::ArucoLocalization(int cam_index, cv::aruco::PREDEFINED_DICTIONARY_NAME name) {
	for (int i = 0; i < 3; i++) {
		arucoCorner[i].x = 0;
		arucoCorner[i].y = 0;
	}
	webcam.open(cam_index);
	webcam.set(cv::CAP_PROP_FRAME_WIDTH, 1920);
	webcam.set(cv::CAP_PROP_FRAME_HEIGHT, 1080);
	webcam.set(cv::CAP_PROP_AUTOFOCUS, 0);
	webcam.set(cv::CAP_PROP_AUTO_EXPOSURE, 1);
	detector_parameters = cv::aruco::DetectorParameters::create();
	dictionary = cv::aruco::getPredefinedDictionary(name);

	//Checking for the webcam to be connected 
	if (webcam.isOpened()) {
		std::cout << "Webcam connected." << std::endl;
	}
	else {
		std::cout << "Webcam not connected." << std::endl;
		exit(1);
	}
}

ArucoLocalization::~ArucoLocalization() {
	webcam.~VideoCapture();
}

bool ArucoLocalization::localizate(td::TransferData* data) {
	if (!webcam.read(currentVideoFrame)) {
		std::cout << "Could not read current video frame." << std::endl;
		return false;
	}
	currentVideoFrame = currentVideoFrame(cv::Rect(340, 0, 1150, 1080));
	//Clearing the markers' indexes vector                   
	markerIds.clear();
	//Detecting the aruco markers                        
	cv::aruco::detectMarkers(currentVideoFrame, dictionary, markerCorners, markerIds, detector_parameters, rejectedCandidates);
	if (markerCorners.empty() == 0) {
		//Checking if there are any markers                  
		if (!markerIds.empty()) {
			//If yes, setting the corners' cartesian         
			for (int i = 0; i < 3; i++) {
				arucoCorner[i].x = markerCorners[0][i].x - 0.105 * (markerCorners[0][i].x - 620);
				arucoCorner[i].y = markerCorners[0][i].y - 0.105 * (markerCorners[0][i].y - 540);
			}
		}
		else {             
			std::cout << "Not aruco markers." << std::endl;
			return false;
		}
		//Calculating current cartesian position in pixels 
		data->prevGlobalCartesian.x = data->currGlobalCartesian.x;
		data->prevGlobalCartesian.y = data->currGlobalCartesian.y;
		data->currGlobalCartesian.x = (arucoCorner[0].x + arucoCorner[2].x) / 2;
		data->currGlobalCartesian.y = (arucoCorner[0].y + arucoCorner[2].y) / 2;
		//Finding the angle of the aruco vector              
		data->prevAngle = data->currAngle;
		data->Angle(arucoCorner);
		data->DeltaEigen();
		return true;
	}
	else {
		std::cout << "Error detecting aruco, try replacing Robotino." << std::endl;
		return false;
	}
}

//Call after localization (function localize).
void ArucoLocalization::show_markers() {
	//Resulting image to be shown                          
	cv::Mat outputImage;
	//Drawing the detection square in the image
	currentVideoFrame.copyTo(outputImage);
	cv::aruco::drawDetectedMarkers(outputImage, markerCorners, markerIds);
	cv::imshow("Found aruco markers.", outputImage);
	cv::waitKey();
}

double getTime(std::chrono::time_point<std::chrono::steady_clock> timePoint1) {
	auto timePoint2 = std::chrono::steady_clock::now();
	std::chrono::duration<float> curTime = timePoint2 - timePoint1;
	auto timePassed = curTime.count();
	return timePassed;
}