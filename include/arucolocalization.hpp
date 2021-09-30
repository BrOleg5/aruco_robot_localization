#ifndef ARUCOLOCALIZATION_LIB
#define ARUCOLOCALIZATION_LIB

#define PI 3.14159265
#define MMperPIX_X 2.1659
#define MMperPIX_Y 2.1131

#include <iostream>
#include "opencv2/aruco.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"

namespace td {
	class TransferData {
		public:
		cv::Point2d currGlobalCartesian;
		cv::Point2d prevGlobalCartesian;
		double currAngle;
		double prevAngle;
		double deltaAngle;
		cv::Point2d deltaEigenCartesian;

		TransferData() : currGlobalCartesian(0, 0), prevGlobalCartesian(0, 0), currAngle(0),
						 prevAngle(0), deltaAngle(0), deltaEigenCartesian(0, 0) {}
		~TransferData() {}

		void Angle(cv::Point2d* arucoCorner);
		void DeltaEigen();
	};
}

class ArucoLocalization {
	//Three first corners of aruco marker cartesian clockwise
	cv::Point2d arucoCorner[3];
	//Frame for webcam capture
	cv::Mat currentVideoFrame;
	//Webcam
	cv::VideoCapture webcam;
	//Vector for aruco markers' indexes
	std::vector<int> markerIds;
	//Matrix for aruco markers' actual and rejected corners
	std::vector<std::vector<cv::Point2f>> markerCorners, rejectedCandidates;
	//Creating the parameters for the aruco detection
	cv::Ptr<cv::aruco::DetectorParameters> detector_parameters;
	//Selecting the dictionary to use                        
	cv::Ptr<cv::aruco::Dictionary> dictionary;

	public:
	ArucoLocalization(int cam_index, cv::aruco::PREDEFINED_DICTIONARY_NAME name);
	~ArucoLocalization();

	bool localizate(td::TransferData* data);
	void show_markers();
};

double getTime(std::chrono::time_point<std::chrono::steady_clock> timePoint1);

#endif