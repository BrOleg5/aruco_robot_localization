#include "arucolocalization.hpp"

void td::TransferData::Angle(cv::Point2f* arucoCorner) {
	if (arucoCorner[1].x >= arucoCorner[2].x) {
		if (arucoCorner[1].y >= arucoCorner[2].y) {
			// if in [0 ; 90] degrees, then               
			currAngle = 270.0f - std::acos(abs(arucoCorner[1].x - arucoCorner[2].x) / sqrt(pow(arucoCorner[1].x - arucoCorner[2].x, 2) + pow(arucoCorner[1].y - arucoCorner[2].y, 2))) * 180.0f / PI;
		}
		else {
			// if in [270 ; 360] degrees, then            
			currAngle = -90.0f + acos(abs(arucoCorner[1].x - arucoCorner[2].x) / sqrt(pow(arucoCorner[1].x - arucoCorner[2].x, 2) + pow(arucoCorner[1].y - arucoCorner[2].y, 2))) * 180.0f / PI;
		}
	}
	else {
		if (arucoCorner[1].y >= arucoCorner[2].y) {
			// if in [90 ; 180] degrees, then             
			currAngle = 90.0f + acos(abs(arucoCorner[1].x - arucoCorner[2].x) / sqrt(pow(arucoCorner[1].x - arucoCorner[2].x, 2) + pow(arucoCorner[1].y - arucoCorner[2].y, 2))) * 180.0f / PI;
		}
		else {
			// if in [180 ; 270] degrees, then            
			currAngle = 90.0f - acos(abs(arucoCorner[1].x - arucoCorner[2].x) / sqrt(pow(arucoCorner[1].x - arucoCorner[2].x, 2) + pow(arucoCorner[1].y - arucoCorner[2].y, 2))) * 180.0f / PI;
		}
	}
	if (currAngle < 0) {
		currAngle = 360.0f + currAngle;
	}
	if (abs(currAngle - prevAngle) > 160) {
		if (currAngle > prevAngle) {
			deltaAngle = -1.0f * (360.0f - currAngle + prevAngle);
		}
		else {
			deltaAngle = 360.0f - prevAngle + currAngle;
		}
	}
	else {
		deltaAngle = currAngle - prevAngle;
	}
}

void td::TransferData::DeltaEigen() {
	cv::Point2f delta;
	delta.x = (currGlobalCartesian.x - prevGlobalCartesian.x) * MMperPIX_X;
	delta.y = (currGlobalCartesian.y - prevGlobalCartesian.y) * MMperPIX_Y;
	deltaEigenCartesian.x = delta.x * sin(currAngle / 180.0f * PI) * (-1.0f) - delta.y * cos(currAngle / 180.0f * PI);
	deltaEigenCartesian.y = delta.x * cos(currAngle / 180.0f * PI) * (-1.0f) + delta.y * sin(currAngle / 180.0f * PI);
}

ArucoLocalization::ArucoLocalization(const cv::VideoCapture& video_capture, cv::aruco::PREDEFINED_DICTIONARY_NAME dict_name) {
	for (int i = 0; i < 3; i++) {
		arucoCorner[i].x = 0;
		arucoCorner[i].y = 0;
	}
	videoCapture = video_capture;
	frame_height = static_cast<int>(videoCapture.get(cv::CAP_PROP_FRAME_HEIGHT));
	frame_width = static_cast<int>(videoCapture.get(cv::CAP_PROP_FRAME_WIDTH));
	detectorParameters = cv::aruco::DetectorParameters::create();
	dictionary = cv::aruco::getPredefinedDictionary(dict_name);
}

int ArucoLocalization::detectMarkers() {
	videoCapture >> currentVideoFrame;
	if(currentVideoFrame.empty()) {
		std::cout << "End of video file.\n";
		return 2;
	}
	markerIds.clear();
	cv::aruco::detectMarkers(currentVideoFrame, dictionary, markerCorners, markerIds, detectorParameters, rejectedCandidates);
	if (!markerCorners.empty() && !markerIds.empty()) {
		return 0;
	}
	else {
		std::cerr << "Error detecting aruco marker" << std::endl;
		return 1;
	}
}

bool ArucoLocalization::estimatePosition(td::TransferData* data, int markerID) {
	int markerIndex = 0;
	if(markerID >= 0) {
		std::vector<int>::iterator markerIterator = std::find(markerIds.begin(), markerIds.end(), markerID);
		if(markerIterator != markerIds.end()) {
			markerIndex = static_cast<int>(markerIterator - markerIds.begin());
		}
		else {
			std::cerr << "Marker with ID=" << markerID << " not searched.\n";
			return false;
		}
	}
	// Calculating corners position of detecting Aruco marker
	for (int i = 0; i < 3; i++) {
		arucoCorner[i].x = markerCorners[markerIndex][i].x - 0.105f * (markerCorners[markerIndex][i].x - static_cast<float>(frame_width/2));
		arucoCorner[i].y = markerCorners[markerIndex][i].y - 0.105f * (markerCorners[markerIndex][i].y - static_cast<float>(frame_height/2));
	}
	//Calculating current cartesian position in pixels
	data->prevGlobalCartesian.x = data->currGlobalCartesian.x;
	data->prevGlobalCartesian.y = data->currGlobalCartesian.y;
	data->currGlobalCartesian.x = (arucoCorner[0].x + arucoCorner[1].x + arucoCorner[2].x + arucoCorner[3].x) / 4.0f;
	data->currGlobalCartesian.y = (arucoCorner[0].y + arucoCorner[1].y + arucoCorner[2].y + arucoCorner[3].y) / 4.0f;
	//Finding the angle of the aruco vector
	data->prevAngle = data->currAngle;
	data->Angle(arucoCorner);
	data->DeltaEigen();
	return true;
}

void ArucoLocalization::show_markers() {    
	cv::Mat outputImage;
	currentVideoFrame.copyTo(outputImage);
	cv::aruco::drawDetectedMarkers(outputImage, markerCorners, markerIds);
	cv::imshow("Found aruco markers", outputImage);
}

cv::Mat ArucoLocalization::draw_marker(int markerID) {
	cv::Mat outputImage;
	currentVideoFrame.copyTo(outputImage);
	std::vector<int>::iterator markerIterator = std::find(markerIds.begin(), markerIds.end(), markerID);
	if(markerIterator != markerIds.end()) {
		int markerIndex = static_cast<int>(markerIterator - markerIds.begin());
		std::vector<std::vector<cv::Point2f>> oneMarkerCorner = {markerCorners[markerIndex]};
		std::vector<int> oneMarkerIds = {markerIds[markerIndex]};
		cv::aruco::drawDetectedMarkers(outputImage, oneMarkerCorner, oneMarkerIds);
	}
	else {
		std::cerr << "Marker with ID=" << markerID << " not searched.\n";
	}
	return outputImage;
}

void ArucoLocalization::show_frame() {
	cv::imshow("Cam frame", currentVideoFrame);
}

cv::Mat ArucoLocalization::get_frame() {
	return currentVideoFrame;
}