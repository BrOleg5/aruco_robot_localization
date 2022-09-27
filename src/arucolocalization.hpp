#ifndef ARUCOLOCALIZATION_LIB
#define ARUCOLOCALIZATION_LIB

#ifndef PI
#	define PI 3.14159265358979323846
#endif

#define MMperPIX_X 2.1659
#define MMperPIX_Y 2.1131

#include <iostream>
#include "opencv2/aruco.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include <algorithm>

namespace td {

	/**
	 * @class TransferData
	 * @brief Store robot's global coordinates and local coordinate change.
	 */
	class TransferData {
		public:

		/**
		 * Current robot's global coordinate.
		 */
		cv::Point2d currGlobalCartesian;

		/**
		 * Previous robot's global coordinate.
		 */
		cv::Point2d prevGlobalCartesian;

		/**
		 * Current robot's rotation angle.
		 */
		double currAngle;

		/**
		 * Previous robot's rotation angle.
		 */
		double prevAngle;

		/**
		 * Robot's local coordinate change.
		 */
		cv::Point2d deltaEigenCartesian;

		/**
		 * Robot's angle change.
		 */
		double deltaAngle;

		/**
		 * Initializtion each coordinate as zero.
		 */
		TransferData() : currGlobalCartesian(0, 0), prevGlobalCartesian(0, 0), currAngle(0),
						 prevAngle(0), deltaAngle(0), deltaEigenCartesian(0, 0) {}
		
		~TransferData() {}

		/**
		 * Obtain robot's rotation angle from aruco corners.
		 * 
		 * @param arucoCorner array of aruco corners.
		 */
		void Angle(cv::Point2d* arucoCorner);

		/**
		 * Obtain robot's local coordinate change.
		 */
		void DeltaEigen();
	};
}

/**
 * @class ArucoLocalization
 * @brief Implementation robot localization by aruco marker.
 */
class ArucoLocalization {
	/**
	 * Three first corners of aruco marker cartesian clockwise.
	 */
	cv::Point2d arucoCorner[3];

	/**
	 * Frame for VideoCapture.
	 */
	cv::Mat currentVideoFrame;

	cv::VideoCapture videoCapture;

	/**
	 * Vector for aruco markers' indexes.
	 */
	std::vector<int> markerIds;

	/**
	 * Matrix for aruco markers' actual and rejected corners.
	 */
	std::vector<std::vector<cv::Point2f>> markerCorners, rejectedCandidates;

	/**
	 * Creating the parameters for the aruco detection.
	 */
	cv::Ptr<cv::aruco::DetectorParameters> detectorParameters;
	
	/**
	 * Selecting the dictionary to use.
	 */
	cv::Ptr<cv::aruco::Dictionary> dictionary;

	public:

	/**
	 * Constructor.
	 * 
	 * @param video_capture cv::VideoCapture object.
	 * @param dict_name name of aruco marker dictonary.
	 */
	ArucoLocalization(const cv::VideoCapture& video_capture, cv::aruco::PREDEFINED_DICTIONARY_NAME dict_name);
	
	~ArucoLocalization() {};

	/**
	 * Localize marker.
	 */
	bool detectMarkers();

	/**
	 * Calculate first detected marker positions on the plane.
	 * 
	 * @param data storage marker's global coordinates and local coordinate change.
	 */
	bool estimatePosition(td::TransferData* data);

	/**
	 * Calculate marker positions on the plane.
	 * 
	 * @param data storage marker's global coordinates and local coordinate change.
	 * @param markerID ID of marker that position calculating
	 */
	bool estimatePosition(td::TransferData* data, int markerID);

	/**
	 * Open window with frame and drew aruco markers.
	 * 
	 * @remark Call after localization (function localize).
	 */
	void show_markers();

	/**
	 * Open window with frame and drew aruco marker.
	 * 
	 * @param markerID ID of marker
	 * @remark Call after localization (function localize).
	 */
	void show_marker(int markerID);

	/**
	 * Open window with frame.
	 */
	void show_frame();
};
#endif