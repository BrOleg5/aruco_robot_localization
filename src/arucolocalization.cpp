#include "arucolocalization.hpp"

void td::TransferData::Angle(cv::Point2f* arucoCorner) {
	float angle[4] = { 0.0f };
	angle[0] = calcAngleParallelSide(arucoCorner[1], arucoCorner[2]);
	angle[1] = calcAngleParallelSide(arucoCorner[0], arucoCorner[3]);
	angle[2] = calcAnglePerpendicularSide(arucoCorner[0], arucoCorner[1]);
	angle[3] = calcAnglePerpendicularSide(arucoCorner[3], arucoCorner[2]);
	bool isCloseZero = true;
	for (int i = 0; i < 4; i++)	{
		if((angle[i] > 20) && (angle[i] < 340)) {
			isCloseZero = false;
		}
	}
	if(isCloseZero) {
		for (int i = 0; i < 4; i++) {
			angle[i] = normAnglePI(angle[i], false);
		}
	}
	currAngle = 0;
	for (int i = 0; i < 4; i++)	{
		currAngle += angle[i];
	}
	currAngle /= 4.0f;
	if(isCloseZero) {
		currAngle = normAngle2PI(currAngle, false);
	}
}

float td::calcAngleParallelSide(cv::Point2f& p1, cv::Point2f& p2) {
	float angle = std::atan2f(p2.x - p1.x, p2.y - p1.y);
	angle = normAngle2PI(angle);
	return rad2deg(angle);
}

float td::calcAnglePerpendicularSide(cv::Point2f& p1, cv::Point2f& p2) {
	float angle = std::atan2f(p1.y - p2.y, p2.x - p1.x);
	angle = normAngle2PI(angle);
	return rad2deg(angle);
}

float td::normAngle2PI(float angle, bool isRadian) {
	if(angle < 0.0f) {
		if(isRadian){
			angle += 2*PI;
		}
		else {
			angle += 360;
		}
	}
	return angle;
}

float td::normAnglePI(float angle, bool isRadian) {
	if(isRadian) {
		if(angle > PI) {
			angle -= 2*PI;
		}
	}
	else {
		if(angle > 180.0f) {
			angle -= 360;
		}
	}
	return angle;
}

void td::TransferData::DeltaAngle() {
	if(std::abs(currAngle -prevAngle) > 160) {
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
	delta.x = (currGlobalCartesian.x - prevGlobalCartesian.x) * pixelResolution.x;
	delta.y = (currGlobalCartesian.y - prevGlobalCartesian.y) * pixelResolution.y;
	float radAngle = deg2rad(currAngle);
	deltaEigenCartesian.x = delta.x * std::sin(radAngle) * (-1.0f) - delta.y * std::cos(radAngle);
	deltaEigenCartesian.y = delta.x * std::cos(radAngle) * (-1.0f) + delta.y * std::sin(radAngle);
}

float td::deg2rad(float deg) {
	return (deg / 180.0f * PI);
}

float td::rad2deg(float rad) {
	return (rad / PI * 180.0f);
}

ArucoLocalization::ArucoLocalization() {
	for (int i = 0; i < 3; i++) {
		arucoCorner[i].x = 0;
		arucoCorner[i].y = 0;
	}
}

ArucoLocalization::ArucoLocalization(const cv::VideoCapture& video_capture, 
									 cv::aruco::PREDEFINED_DICTIONARY_NAME dict_name) {
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
		return Status::END_OF_VIDEO_FILE;
	}
	markerIds.clear();
	cv::aruco::detectMarkers(currentVideoFrame, dictionary, markerCorners, markerIds, detectorParameters, rejectedCandidates);
	if (!markerCorners.empty() && !markerIds.empty()) {
		return Status::OK;
	}
	else {
		std::cerr << "Markers are not detected.\n";
		return Status::MARKER_NOT_DETECTED;
	}
}

int ArucoLocalization::filterMarkers(int markerID) {
	std::vector<int>::iterator markerIterator = std::find(markerIds.begin(), markerIds.end(), markerID);
	if(markerIterator != markerIds.end()) {
		return *markerIterator;
	}
	else {
		return Status::NOT_MARKER_INDEX;
	}
}

bool ArucoLocalization::estimatePosition(td::TransferData* data, int markerID) {
	int markerIndex = 0;
	if(markerID >= 0) {
		markerIndex = filterMarkers(markerID);
		if(markerIndex < 0) {
			return false;
		}
	}
	std::vector<cv::Point2f> currMarkerCorners = markerCorners[markerIndex];

	// Calculating corners position of detecting Aruco marker
	for (int i = 0; i < 4; i++) {
		arucoCorner[i].x = currMarkerCorners[i].x - 0.105f * (currMarkerCorners[i].x - static_cast<float>(frame_width/2));
		arucoCorner[i].y = currMarkerCorners[i].y - 0.105f * (currMarkerCorners[i].y - static_cast<float>(frame_height/2));
	}
	//Calculating current cartesian position in pixels
	data->prevGlobalCartesian.x = data->currGlobalCartesian.x;
	data->prevGlobalCartesian.y = data->currGlobalCartesian.y;
	data->currGlobalCartesian.x = (arucoCorner[0].x + arucoCorner[1].x + arucoCorner[2].x + arucoCorner[3].x) / 4.0f;
	data->currGlobalCartesian.y = (arucoCorner[0].y + arucoCorner[1].y + arucoCorner[2].y + arucoCorner[3].y) / 4.0f;
	//Finding the angle of the aruco vector
	data->prevAngle = data->currAngle;
	data->Angle(arucoCorner);
	data->DeltaAngle();
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
	if(markerID >= 0) {
		int markerIndex = filterMarkers(markerID);
		if(markerIndex >= 0) {
			std::vector<std::vector<cv::Point2f>> oneMarkerCorner = {markerCorners[markerIndex]};
			std::vector<int> oneMarkerIds = {markerIds[markerIndex]};
			cv::aruco::drawDetectedMarkers(outputImage, oneMarkerCorner, oneMarkerIds);
		}
		else {
			std::cerr << "Marker with ID = " << markerID << " not searched.\n";
		}
	}
	else {
		std::cerr << "Invalid marker ID = " << markerID << '\n';
	}
	return outputImage;
}

void ArucoLocalization::show_frame() {
	cv::imshow("Cam frame", currentVideoFrame);
}

cv::Mat ArucoLocalization::get_frame() {
	return currentVideoFrame;
}

void ArucoLocalization::getMarkersCorners(std::vector<std::vector<cv::Point2f>>& marker_corners) {
	marker_corners = markerCorners;
}

void ArucoLocalization::getMarkersIndexes(std::vector<int>& marker_ids) {
	marker_ids = markerIds;
}