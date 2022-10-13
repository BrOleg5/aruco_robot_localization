#ifndef ARUCOLOCALIZATION_LIB
#define ARUCOLOCALIZATION_LIB

#ifndef PI
#	define PI 3.14159265358979323846f
#endif

#include <iostream>
#include "opencv2/aruco.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include <algorithm>
#include <cmath>

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
		cv::Point2f currGlobalCartesian;

		/**
		 * Previous robot's global coordinate.
		 */
		cv::Point2f prevGlobalCartesian;

		/**
		 * Current robot's rotation angle.
		 */
		float currAngle;

		/**
		 * Previous robot's rotation angle.
		 */
		float prevAngle;

		/**
		 * Robot's local coordinate change.
		 */
		cv::Point2f deltaEigenCartesian;

		/**
		 * Robot's angle change.
		 */
		float deltaAngle;

		cv::Point2f pixelResolution;

		/**
		 * Initializtion each coordinate as zero.
		 */
		TransferData(const cv::Point2f& pixel_resolution) : currGlobalCartesian(0.0f, 0.0f), prevGlobalCartesian(0.0f, 0.0f), currAngle(0.0f),
						 prevAngle(0.0f), deltaAngle(0.0f), deltaEigenCartesian(0.0f, 0.0f), pixelResolution(pixel_resolution) {}
		
		~TransferData() {}

		/**
		 * Obtain robot's rotation angle from aruco corners.
		 * 
		 * @param arucoCorner array of aruco corners.
		 */
		void Angle(cv::Point2f* arucoCorner);

		/**
		 * Obtain robot's local coordinate change.
		 */
		void DeltaEigen();
	};

	float deg2rad(float deg);
}

/**
 * @class ArucoLocalization
 * @brief Implementation robot localization by aruco marker.
 */
class ArucoLocalization {
	/**
	 * Corners of aruco marker cartesian clockwise.
	 */
	cv::Point2f arucoCorner[4];

	/**
	 * Frame size
	*/
	int frame_height;
	int frame_width;

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

	cv::Point2f pixelResolution;

	public:

	/**
	 * Constructor.
	 * 
	 * @param video_capture cv::VideoCapture object.
	 * @param dict_name name of aruco marker dictonary.
	 */
	ArucoLocalization(const cv::VideoCapture& video_capture, 
				 	  cv::aruco::PREDEFINED_DICTIONARY_NAME dict_name);
	
	~ArucoLocalization() {}

	/**
	 * Localize marker.
	 */
	int detectMarkers();

	/**
	 * Calculate marker positions on the plane.
	 * 
	 * @param data storage marker's global coordinates and local coordinate change.
	 * @param markerID ID of marker that position calculating
	 */
	bool estimatePosition(td::TransferData* data, int markerID = -1);

	/**
	 * Open window with frame and drew aruco markers.
	 * 
	 * @remark Call after localization (function localize).
	 */
	void show_markers();

	cv::Mat draw_marker(int markerID);

	cv::Mat get_frame();

	/**
	 * Open window with frame.
	 */
	void show_frame();

	void getMarkersCorners(std::vector<std::vector<cv::Point2f>>& marker_corners);
	void getMarkersIndexes(std::vector<int>& marker_ids);
};
#endif