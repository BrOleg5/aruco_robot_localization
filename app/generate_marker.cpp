// Copyright 2022 BrOleg5

#include <iostream>

#include <opencv2/core/utility.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/highgui.hpp>

enum ErrorCode : int {
    OK,
    COMMAND_LINE_PARSER_ERROR,
    NOT_ENOUGH_ARGUMENTS
};

const char about[] = "Aruco marker generator.";
const char keys[]  =
        "{h help ? usage |        | Print help message}"
        "{@outfile       | <none> | Output image }"
        "{d              |        | dictionary: DICT_4X4_50=0, DICT_4X4_100=1, DICT_4X4_250=2,"
        "DICT_4X4_1000=3, DICT_5X5_50=4, DICT_5X5_100=5, DICT_5X5_250=6, DICT_5X5_1000=7, "
        "DICT_6X6_50=8, DICT_6X6_100=9, DICT_6X6_250=10, DICT_6X6_1000=11, DICT_7X7_50=12,"
        "DICT_7X7_100=13, DICT_7X7_250=14, DICT_7X7_1000=15, DICT_ARUCO_ORIGINAL = 16}"
        "{id             |        | Marker id in the dictionary }"
        "{ms             |        | Marker size in pixels }"
        "{bb             | 1      | Number of bits in marker borders }"
        "{si             |        | show generated image }";

int main( int argc, char **argv ) {
    cv::CommandLineParser parser(argc, argv, keys);
    parser.about(about);

    if(parser.has("h") || parser.has("help") || parser.has("?") || parser.has("usage")) {
        parser.printMessage();
        return ErrorCode::OK;
    }

    if(!parser.check()) {
        parser.printErrors();
        return ErrorCode::COMMAND_LINE_PARSER_ERROR;
    }

    cv::Ptr<cv::aruco::Dictionary> dictionary;
    if (parser.has("d")) {
        int dictionaryId = parser.get<int>("d");
        dictionary = cv::aruco::getPredefinedDictionary(
            cv::aruco::PREDEFINED_DICTIONARY_NAME(dictionaryId)
        );
    } else {
        std::cout << "Dictionary not specified.\n";
        return ErrorCode::NOT_ENOUGH_ARGUMENTS;
    }

    int marker_id = 0;
    if (parser.has("id")) {
        marker_id = parser.get<int>("id");
    } else {
        std::cout << "Marker ID not specified.\n";
        return ErrorCode::NOT_ENOUGH_ARGUMENTS;
    }

    int img_size = 0;
    if (parser.has("ms")) {
        img_size = parser.get<int>("ms");
    } else {
        std::cout << "Marker size not specified.\n";
        return ErrorCode::NOT_ENOUGH_ARGUMENTS;
    }

    int border_size = parser.get<int>("bb");

    std::string out = parser.get<std::string>(0);

    cv::Mat markerImage;
    cv::aruco::drawMarker(dictionary, marker_id, img_size, markerImage, border_size);

    if(parser.has("si")) {
        cv::imshow("marker", markerImage);
        cv::waitKey(0);
    }

    cv::imwrite(out, markerImage);
    return 0;
}
