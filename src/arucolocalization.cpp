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

ArucoLocalization::ArucoLocalization(const cv::VideoCapture& video_capture, cv::aruco::PREDEFINED_DICTIONARY_NAME dict_name) {
	for (int i = 0; i < 3; i++) {
		arucoCorner[i].x = 0;
		arucoCorner[i].y = 0;
	}
	videoCapture = video_capture;
	detectorParameters = cv::aruco::DetectorParameters::create();
	dictionary = cv::aruco::getPredefinedDictionary(dict_name);
}

bool ArucoLocalization::detectMarkers() {
	videoCapture >> currentVideoFrame;
	if(currentVideoFrame.empty()) {
		std::cout << "End of video file.\n";
		return false;
	}
	markerIds.clear();
	cv::aruco::detectMarkers(currentVideoFrame, dictionary, markerCorners, markerIds, detectorParameters, rejectedCandidates);
	if (!markerCorners.empty() && !markerIds.empty()) {
		return true;
	}
	else {
		std::cerr << "Error detecting aruco marker" << std::endl;
		return false;
	}
}

bool ArucoLocalization::estimatePosition(td::TransferData* data) {
		// Calculating corners position of detecting Aruco marker
		for (int i = 0; i < 3; i++) {
			arucoCorner[i].x = markerCorners[0][i].x - 0.105 * (markerCorners[0][i].x - 620);
			arucoCorner[i].y = markerCorners[0][i].y - 0.105 * (markerCorners[0][i].y - 540);
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

bool ArucoLocalization::estimatePosition(td::TransferData* data, int markerID) {
	std::vector<int>::iterator markerIterator = std::find(markerIds.begin(), markerIds.end(), markerID);
	if(markerIterator != markerIds.end()) {
		int markerIndex = static_cast<int>(markerIterator - markerIds.begin());
		// Calculating corners position of detecting Aruco marker
		for (int i = 0; i < 3; i++) {
			arucoCorner[i].x = markerCorners[markerIndex][i].x - 0.105 * (markerCorners[markerIndex][i].x - 620);
			arucoCorner[i].y = markerCorners[markerIndex][i].y - 0.105 * (markerCorners[markerIndex][i].y - 540);
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
		std::cerr << "Marker with ID=" << markerID << " not searched.\n";
		return false;
	}
}

void ArucoLocalization::show_markers() {    
	cv::Mat outputImage;
	currentVideoFrame.copyTo(outputImage);
	cv::aruco::drawDetectedMarkers(outputImage, markerCorners, markerIds);
	cv::imshow("Found aruco markers", outputImage);
}

void ArucoLocalization::show_marker(int markerID) {
	std::vector<int>::iterator markerIterator = std::find(markerIds.begin(), markerIds.end(), markerID);
	if(markerIterator != markerIds.end()) {
		int markerIndex = static_cast<int>(markerIterator - markerIds.begin());
		cv::Mat outputImage;
		currentVideoFrame.copyTo(outputImage);
		std::vector<std::vector<cv::Point2f>> oneMarkerCorner = {markerCorners[markerIndex]};
		std::vector<int> oneMarkerIds = {markerIds[markerIndex]};
		cv::aruco::drawDetectedMarkers(outputImage, oneMarkerCorner, oneMarkerIds);
		cv::imshow("Found aruco marker", outputImage);
	}
	else {
		std::cerr << "Marker with ID=" << markerID << " not searched.\n";
	}
}

void ArucoLocalization::show_frame() {
	cv::imshow("Cam frame", currentVideoFrame);
}