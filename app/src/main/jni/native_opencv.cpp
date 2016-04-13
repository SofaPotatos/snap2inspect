#include "native_opencv.h"
using namespace std;
using namespace cv;

extern "C" {

// project the 2D point set from image to 3D plane of chess board
//bool backProject(Mat cameraMatrix, const Mat& boardRot64,
//                                    const Mat& boardTrans64,
//                                    const vector<Point2f>& imgPt,
//                                    vector<Point3f>& worldPt) {
//    if( imgPt.size() == 0 ) {
//        return false;
//    }
//    else
//    {
//        Mat imgPt_h = Mat::zeros(3, imgPt.size(), CV_32F);
//        for( int h=0; h<imgPt.size(); ++h ) {
//            imgPt_h.at<float>(0,h) = imgPt[h].x;
//            imgPt_h.at<float>(1,h) = imgPt[h].y;
//            imgPt_h.at<float>(2,h) = 1.0f;
//        }
//        Mat Kinv64 = cameraMatrix.inv();
//        Mat Kinv,boardRot,boardTrans;
//        Kinv64.convertTo(Kinv, CV_32F);
//        boardRot64.convertTo(boardRot, CV_32F);
//        boardTrans64.convertTo(boardTrans, CV_32F);
//
//        // Transform all image points to world points in camera reference frame
//        // and then into the plane reference frame
//        Mat worldImgPt = Mat::zeros( 3, imgPt.size(), CV_32F );
//        Mat rot3x3;
//        Rodrigues(boardRot, rot3x3);
//
//        Mat transPlaneToCam = rot3x3.inv()*boardTrans;
//
//        for( int i=0; i<imgPt.size(); ++i ) {
//            Mat col = imgPt_h.col(i);
//            Mat worldPtcam = Kinv*col;
//            Mat worldPtPlane = rot3x3.inv()*(worldPtcam);
//
//            float scale = transPlaneToCam.at<float>(2)/worldPtPlane.at<float>(2);
//            Mat worldPtPlaneReproject = scale*worldPtPlane-transPlaneToCam;
//
//            Point3f pt;
//            pt.x = worldPtPlaneReproject.at<float>(0);
//            pt.y = worldPtPlaneReproject.at<float>(1);
//            pt.z = 0;
//            worldPt.push_back(pt);
//        }
//    }
//    return true;
//}


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
        cv::Mat boardRot;
        cv::Mat boardTrans;
        //cv::solvePnP(candidateObjectPts, imgPts, cameraMatrix, distCoeffs, boardRot, boardTrans);
        //backProject(boardRot, boardTrans, circlesImgPts, circlesObjectPts);

        //imagePoints.push_back(calibrationProjector.candidateImagePoints);
        //objectPoints.push_back(circlesObjectPts);
       // camPoints.push_back(circlesImgPts);

    }
    resize(Img,mGr,size_or);
//        int i = rand() % mGr.cols;
//        int j = rand() % mGr.rows;
//        mGr.at<uchar>(j, i) = 255;
//    }
}
}
