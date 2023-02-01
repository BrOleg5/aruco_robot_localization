// Copyright 2022 BrOleg5

#include "arucolocalization.hpp"

void td::TransferData::Angle(cv::Point2f* arucoCorner) {
    float angle[4] = { 0.0f };
    angle[0] = calcAngleParallelSide(arucoCorner[1], arucoCorner[2]);
    angle[1] = calcAngleParallelSide(arucoCorner[0], arucoCorner[3]);
    angle[2] = calcAnglePerpendicularSide(arucoCorner[0], arucoCorner[1]);
    angle[3] = calcAnglePerpendicularSide(arucoCorner[3], arucoCorner[2]);
    bool isCloseZero = true;
    for (int i = 0; i < 4; i++) {
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
    for (int i = 0; i < 4; i++) {
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
        } else {
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
    } else {
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
        } else {
            deltaAngle = 360.0f - prevAngle + currAngle;
        }
    } else {
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
    detectorParameters = cv::aruco::DetectorParameters::create();
}

ArucoLocalization::ArucoLocalization(cv::aruco::PREDEFINED_DICTIONARY_NAME dict_name):
    ArucoLocalization()
{
    dictionary = cv::aruco::getPredefinedDictionary(dict_name);
}

bool ArucoLocalization::detectMarkers(const cv::Mat& frame) {
    markerIds.clear();
    cv::aruco::detectMarkers(frame, dictionary, markerCorners, markerIds, detectorParameters);
    if (!markerCorners.empty() && !markerIds.empty()) {
        return true;
    } else {
        return false;
    }
}

int ArucoLocalization::filterMarkers(int markerID) {
    std::vector<int>::iterator markerIterator = std::find(markerIds.begin(), markerIds.end(),
                                                          markerID);
    if(markerIterator != markerIds.end()) {
        return *markerIterator;
    } else {
        return -1;
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
        arucoCorner[i].x = currMarkerCorners[i].x - 0.105f *
                           (currMarkerCorners[i].x - static_cast<float>(frame_width/2));
        arucoCorner[i].y = currMarkerCorners[i].y - 0.105f *
                           (currMarkerCorners[i].y - static_cast<float>(frame_height/2));
    }
    // Calculating current cartesian position in pixels
    data->prevGlobalCartesian.x = data->currGlobalCartesian.x;
    data->prevGlobalCartesian.y = data->currGlobalCartesian.y;
    data->currGlobalCartesian.x = (arucoCorner[0].x + arucoCorner[1].x +
                                  arucoCorner[2].x + arucoCorner[3].x) / 4.0f;
    data->currGlobalCartesian.y = (arucoCorner[0].y + arucoCorner[1].y +
                                  arucoCorner[2].y + arucoCorner[3].y) / 4.0f;
    // Finding the angle of the aruco vector
    data->prevAngle = data->currAngle;
    data->Angle(arucoCorner);
    data->DeltaAngle();
    data->DeltaEigen();
    return true;
}

bool ArucoLocalization::draw_marker(cv::InputOutputArray frame, int markerID) {
    if(markerID >= 0) {
        int markerIndex = filterMarkers(markerID);
        if(markerIndex >= 0) {
            std::vector<std::vector<cv::Point2f>> oneMarkerCorner = {markerCorners[markerIndex]};
            std::vector<int> oneMarkerIds = {markerIds[markerIndex]};
            cv::aruco::drawDetectedMarkers(frame, oneMarkerCorner, oneMarkerIds);
        } else {
            return false;
        }
    } else {
        cv::aruco::drawDetectedMarkers(frame, markerCorners, markerIds);
    }
    return true;
}

void ArucoLocalization::getMarkersCorners(std::vector<std::vector<cv::Point2f>>& marker_corners) {
    marker_corners = markerCorners;
}

void ArucoLocalization::getMarkersIndexes(std::vector<int>& marker_ids) {
    marker_ids = markerIds;
}

void ArucoLocalization::setMarkerDictonary(cv::aruco::PREDEFINED_DICTIONARY_NAME dict_name) {
    dictionary = cv::aruco::getPredefinedDictionary(dict_name);
}

void ArucoLocalization::setMarkerDictionary(int dict_id) {
    dictionary = cv::aruco::getPredefinedDictionary(dict_id);
}

void ArucoLocalization::setFrameSize(int frame_width, int frame_height) {
    this->frame_width = frame_width;
    this->frame_height = frame_height;
}

std::string getDictionaryName(int dict_id) {
    switch (dict_id) {
    case cv::aruco::DICT_4X4_50:
        return std::string("DICT_4X4_50");
    case cv::aruco::DICT_4X4_100:
        return std::string("DICT_4X4_100");
    case cv::aruco::DICT_4X4_250:
        return std::string("DICT_4X4_250");
    case cv::aruco::DICT_4X4_1000:
        return std::string("DICT_4X4_1000");
    case cv::aruco::DICT_5X5_50:
        return std::string("DICT_5X5_50");
    case cv::aruco::DICT_5X5_100:
        return std::string("DICT_5X5_100");
    case cv::aruco::DICT_5X5_250:
        return std::string("DICT_5X5_250");
    case cv::aruco::DICT_5X5_1000:
        return std::string("DICT_5X5_1000");
    case cv::aruco::DICT_6X6_50:
        return std::string("DICT_6X6_50");
    case cv::aruco::DICT_6X6_100:
        return std::string("DICT_6X6_100");
    case cv::aruco::DICT_6X6_250:
        return std::string("DICT_6X6_250");
    case cv::aruco::DICT_6X6_1000:
        return std::string("DICT_6X6_1000");
    case cv::aruco::DICT_7X7_50:
        return std::string("DICT_7X7_50");
    case cv::aruco::DICT_7X7_100:
        return std::string("DICT_7X7_100");
    case cv::aruco::DICT_7X7_250:
        return std::string("DICT_7X7_250");
    case cv::aruco::DICT_7X7_1000:
        return std::string("DICT_7X7_1000");
    case cv::aruco::DICT_ARUCO_ORIGINAL:
        return std::string("DICT_ARUCO_ORIGINAL");
    case cv::aruco::DICT_APRILTAG_16h5:
        return std::string("DICT_APRILTAG_16h5");
    case cv::aruco::DICT_APRILTAG_25h9:
        return std::string("DICT_APRILTAG_25h9");
    case cv::aruco::DICT_APRILTAG_36h10:
        return std::string("DICT_APRILTAG_36h10");
    case cv::aruco::DICT_APRILTAG_36h11:
        return std::string("DICT_APRILTAG_36h11");

    default:
        return std::string("Undefined dictionary");
    }
}
