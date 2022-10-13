#include <opencv2/core/types.hpp>
#include <opencv2/core/persistence.hpp>

inline static bool readCameraParameters(std::string filename, cv::Point2f& pixelResolution) {
    cv::FileStorage fs(filename, cv::FileStorage::READ);
    if (!fs.isOpened()) {
        return false;
    }
    fs["pixel_resolution_x"] >> pixelResolution.x;
    fs["pixel_resolution_y"] >> pixelResolution.y;
    fs.release();
    return true;
}

inline static bool saveCameraParams(const std::string &filename, const cv::Point2f& pixelResolution, 
                                    float markerSize,
                                    const cv::Point2f& std) {
    cv::FileStorage fs(filename, cv::FileStorage::WRITE || cv::FileStorage::FORMAT_JSON);
    if (!fs.isOpened()) {
        return false;
    }

    fs << "pixel_resolution_x" << pixelResolution.x;
    fs << "pixel_resolution_y" << pixelResolution.y;
    fs << "marker_size" << markerSize;
    fs << "standard_deviation_x" << std.x;
    fs << "standard_deviation_y" << std.y;
    fs.release();
    return true;
}