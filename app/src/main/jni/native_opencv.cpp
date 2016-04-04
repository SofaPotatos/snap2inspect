#include "native_opencv.h"

using namespace std;
using namespace cv;

extern "C" {
JNIEXPORT void JNICALL Java_i2r_snap2inspect_Tutorial3Activity_salt(JNIEnv *env, jobject instance,
                                                                                         jlong matAddrGray,
                                                                                         jint nbrElem) {

    Mat &mGr = *(Mat *) matAddrGray;
    Mat Img;
    Size size_ds(640, 360);
    Size size_or(1280,720);
    resize(mGr,Img,size_ds);
    vector<cv::Point2f> chessImgPts;

    bool bPrintedPatternFound = findChessboardCorners(Img, Size(6, 9), chessImgPts,
                                                      CV_CALIB_CB_ADAPTIVE_THRESH |
                                                      CV_CALIB_CB_FAST_CHECK |
                                                      CV_CALIB_CB_NORMALIZE_IMAGE);


    vector<cv::Point2f> circlesImgPts;
    cv::SimpleBlobDetector::Params params;
    params.filterByArea = true;
    params.filterByCircularity = true;
    params.filterByConvexity = true;
    params.filterByInertia = true;
    params.minThreshold = 10;
    params.minArea = 5;
    params.maxArea = 100000;
    params.minCircularity = 0.5;
    params.minConvexity = 0.75;
    params.minInertiaRatio = 0.001;
    params.blobColor = 255;
    Ptr< FeatureDetector > blobDetector = SimpleBlobDetector::create(params);
    bool bProjectedPatternFound = cv::findCirclesGrid(Img, Size(6, 4), circlesImgPts,
                                                      cv::CALIB_CB_SYMMETRIC_GRID, blobDetector);
        //cvtColor(mGr,mGr , CV_GRAY2RGB);
        //cv::threshold(mGr, mGr, 200, 255, cv::THRESH_BINARY_INV);

    if(bPrintedPatternFound)
    {
        for (size_t i = 0; i < chessImgPts.size(); i++) {
            cv::circle(Img, chessImgPts[i], 5, 255, 2, 8, 0);
        }
    }

    if (bProjectedPatternFound) {
        for (size_t i = 0; i < circlesImgPts.size(); i++) {
            cv::circle(Img, circlesImgPts[i], 5, 0, 2, 8, 0);
        }
    }

    if(bPrintedPatternFound&&bProjectedPatternFound)
    {
        cv::threshold(Img, Img, 200, 255, cv::THRESH_BINARY_INV);
    }
    resize(Img,mGr,size_or);
//        int i = rand() % mGr.cols;
//        int j = rand() % mGr.rows;
//        mGr.at<uchar>(j, i) = 255;
//    }
}
}
