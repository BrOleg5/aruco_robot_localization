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
	//Clearing the markers' indexes vector
	markerIds.clear();
	//Detecting the aruco markers
	cv::aruco::detectMarkers(currentVideoFrame, dictionary, markerCorners, markerIds, detectorParameters, rejectedCandidates);
	if (!markerCorners.empty() && !markerIds.empty()) {
		for (int i = 0; i < 3; i++) {
					arucoCorner[i].x = markerCorners[0][i].x - 0.105 * (markerCorners[0][i].x - 620);
					arucoCorner[i].y = markerCorners[0][i].y - 0.105 * (markerCorners[0][i].y - 540);
		}
		return true;
	}
	else {
		std::cerr << "Error detecting aruco marker" << std::endl;
		return false;
	}
}

void ArucoLocalization::estimatePosition(td::TransferData* data) {
	//Calculating current cartesian position in pixels 
	data->prevGlobalCartesian.x = data->currGlobalCartesian.x;
	data->prevGlobalCartesian.y = data->currGlobalCartesian.y;
	data->currGlobalCartesian.x = (arucoCorner[0].x + arucoCorner[2].x) / 2;
	data->currGlobalCartesian.y = (arucoCorner[0].y + arucoCorner[2].y) / 2;
	//Finding the angle of the aruco vector              
	data->prevAngle = data->currAngle;
	data->Angle(arucoCorner);
	data->DeltaEigen();
}

void ArucoLocalization::show_markers() {
	//Resulting image to be shown                          
	cv::Mat outputImage;
	//Drawing the detection square in the image
	currentVideoFrame.copyTo(outputImage);
	cv::aruco::drawDetectedMarkers(outputImage, markerCorners, markerIds);
	cv::imshow("Found aruco markers", outputImage);
}

void ArucoLocalization::show_frame() {
	cv::imshow("Cam frame", currentVideoFrame);
}