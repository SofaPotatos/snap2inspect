package i2r.snap2inspect;

import android.util.Log;
import org.opencv.calib3d.Calib3d;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfDouble;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.MatOfPoint3f;
import org.opencv.core.Point;
import org.opencv.core.Point3;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import java.util.ArrayList;
import java.util.List;

public class CamProjCalib {
    private static final String TAG = "CamProjCalibrator";

    //Camera
    private final Size mPatternSize = new Size(4, 11);
    private final int mCornersSize = (int)(mPatternSize.width * mPatternSize.height);
    private boolean mPatternWasFound = false;
    private MatOfPoint2f mCorners;
    private List<Mat> mCornersBuffer = new ArrayList<Mat>();
    private boolean mIsCalibrated = false;
//
    private int mFlags;
    private double mSquareSize = 18.1;
    private Size mImageSize;
    private MatOfPoint3f mBoardPoints;
//
    //Projector
    private final Size pPatternSize = new Size(6, 8);
    private final int pCornersSize = (int)(pPatternSize.width * pPatternSize.height);
    private boolean pPatternWasFound = false;
    private MatOfPoint2f pCorners;
    private List<Mat> pCornersBuffer = new ArrayList<Mat>();
    private boolean pIsCalibrated = false;
    private int pFlags;
    private double pSquareSize = 80;
    private Point pCenter = new Point(300, 250);
    private Size pImageSize;
    public Mat pProjImage;
    private MatOfPoint2f pPatternPoints;
//
//    //Global
    private List<Mat> oCornersBuffer = new ArrayList<Mat>();
    public Mat R;
    public Mat T;
    public Mat E;
    public Mat F;
    public Mat Q;
    public Mat P1;
    public Mat P2;
    public Mat R1;
    public Mat R2;
    public Mat PM;
    public Mat PK;
    public Mat CM;
    public Mat CK;

    public CamProjCalib(){}

    public boolean setup(int mwidth, int mheight, int pwidth, int pheight) {
        mCorners = new MatOfPoint2f();
        mBoardPoints = new MatOfPoint3f();
        pCorners = new MatOfPoint2f();
        pPatternPoints = new MatOfPoint2f();
        pProjImage = new Mat();
        R=new Mat();
        T=new Mat();
        E=new Mat();
        F=new Mat();
        mImageSize = new Size(mwidth, mheight);
        pImageSize = new Size(pwidth, pheight);
        mFlags =Calib3d.CALIB_FIX_PRINCIPAL_POINT +
                Calib3d.CALIB_FIX_ASPECT_RATIO +
                Calib3d.CALIB_ZERO_TANGENT_DIST +
                Calib3d.CALIB_FIX_K1 +
                Calib3d.CALIB_FIX_K2 +
                Calib3d.CALIB_FIX_K3 +
                Calib3d.CALIB_FIX_K4 +
                Calib3d.CALIB_FIX_K5;
        CM = new Mat();
        Mat.eye(3, 3, CvType.CV_64FC1).copyTo(CM);
        PM = new Mat();
        Mat.eye(3, 3, CvType.CV_64FC1).copyTo(PM);
        CM.put(0,0,820.2072312616415);
        CM.put(1,1,820.2072312616415);
        CM.put(0,2,639.5);
        CM.put(10,2,359.5);
        PM.put(0,0,1758.104828);
        PM.put(1,1,1758.104828);
        PM.put(0,2,639.5);
        PM.put(10,2,359.5);
        CK= new Mat();
        Mat.zeros(5, 1, CvType.CV_64FC1).copyTo(CK);
        PK= new Mat();
        Mat.zeros(5, 1, CvType.CV_64FC1).copyTo(PK);

        Mat.zeros(mPatternSize, CvType.CV_64FC1).copyTo(mBoardPoints);
        calcBoardCornerPositions();
        calcPatternCornerPositions();
        Log.i(TAG, "Instantiated new " + this.getClass());
        return true;
    }

    public void processFrame(Mat grayFrame, Mat rgbaFrame) {
        findPattern(grayFrame);
        renderFrame(rgbaFrame);
    }

    public void doCalibrate() {

        for (int i = 0; i < mCornersBuffer.size(); i++) {
            pCornersBuffer.add(pPatternPoints.clone());
        }
        Calib3d.stereoCalibrate(oCornersBuffer, mCornersBuffer, pCornersBuffer, CM, CK, PM, PK, mImageSize, R, T, E, F, mFlags);
        mIsCalibrated = Core.checkRange(PM) && Core.checkRange(PK);
        Log.i(TAG, "CM: " + CM.dump());
        Log.i(TAG, "CK: " + CK.dump());
        Log.i(TAG, "PM: " + PM.dump());
        Log.i(TAG, "PK: " + PK.dump());
        Log.i(TAG, "R: " + R.dump());
        Log.i(TAG, "T: " + T.dump());
        Log.i(TAG, "F: " + F.dump());
    }

    public void clearCorners() {
        mCornersBuffer.clear();
        pCornersBuffer.clear();
    }

    private void calcBoardCornerPositions() {
        final int cn = 3;
        float positions[] = new float[mCornersSize * cn];
        for (int i = 0; i < mPatternSize.height; i++) {
            for (int j = 0;j < mPatternSize.width * cn; j += cn) {
                positions[(int) (i * mPatternSize.width * cn + j + 0)] =
                        (2 * (j / cn) + i % 2) * (float) mSquareSize;
                positions[(int) (i * mPatternSize.width * cn + j + 1)] =
                        i * (float) mSquareSize;
                positions[(int) (i * mPatternSize.width * cn + j + 2)] = 0;
            }
        }
        mBoardPoints.create(mCornersSize, 1, CvType.CV_32FC3);
        mBoardPoints.put(0, 0, positions);
    }

    private void calcPatternCornerPositions() {
        final int cn = 2;
        float positions[] = new float[pCornersSize * cn];
        Point vertex = new Point();
        int DotSize = 10;
        Mat.zeros(pImageSize, CvType.CV_8UC1).copyTo(pProjImage);

        for (int i = 0; i < pPatternSize.width; i++) {
            for (int j = 0; j < pPatternSize.height * cn; j += cn) {
                vertex.x=(float)pCenter.x+ j / cn * (float) pSquareSize;
                vertex.y=(float)pCenter.y+ i * (float) pSquareSize;

                positions[(int) (i * pPatternSize.height * cn + j + 0)] = (float) vertex.x;
                positions[(int) (i * pPatternSize.height * cn + j + 1)] = (float) vertex.y;

                Imgproc.circle(pProjImage, vertex, DotSize, new Scalar(255, 0, 0), -1);
            }
        }
        pPatternPoints.create(pCornersSize, 1, CvType.CV_32FC2);
        pPatternPoints.put(0, 0, positions);
    }

    private void findPattern(Mat grayFrame) {
        mPatternWasFound = Calib3d.findCirclesGrid(grayFrame, mPatternSize,
                mCorners, Calib3d.CALIB_CB_ASYMMETRIC_GRID);
        Core.absdiff(grayFrame, new Scalar(255), grayFrame);
        pPatternWasFound = Calib3d.findCirclesGrid(grayFrame, pPatternSize,
                pCorners, Calib3d.CALIB_CB_SYMMETRIC_GRID);
    }

    private boolean backProject(Mat boardRot64, Mat boardTrans64, MatOfPoint2f imgPt, MatOfPoint3f worldPt) {
        if( imgPt.empty()) {
            return false;
        }
        else
        {
            Log.i(TAG, "boardRot64: " + boardRot64.dump());
            Log.i(TAG, "boardTrans64,: " + boardTrans64.dump());
            Mat Kinv64 = new Mat();
            Mat Kinv = new Mat();
            Mat boardRot = new Mat();
            Mat boardTrans = new Mat();
            Kinv64=CM.inv();
            Kinv64.convertTo(Kinv, CvType.CV_32F);
            boardRot64.convertTo(boardRot, CvType.CV_32F);
            boardTrans64.convertTo(boardTrans, CvType.CV_32F);
            // Transform all image points to world points in camera reference frame
            // and then into the plane reference frame
            Mat rot3x3 = new Mat();
            Calib3d.Rodrigues(boardRot, rot3x3);

            Mat transPlaneToCam = new Mat();
            transPlaneToCam.create(3, 1, CvType.CV_32F);
            Mat worldPtcam = new Mat();
            worldPtcam.create(3,1,CvType.CV_32F);
            Mat worldPtPlane = new Mat();
            worldPtPlane.create(3, 1, CvType.CV_32F);
            Mat worldPtPlaneReproject = new Mat();
            Point3 fp = new Point3();
            List<Point3> fpa =new ArrayList<Point3>();
            worldPtPlaneReproject.create(3, 1, CvType.CV_32F);
            Log.i(TAG, "rot3x3.inv(): " + rot3x3.inv().dump());
            Log.i(TAG, "transPlaneToCam: " + transPlaneToCam.dump());
            Mat NullM = new Mat();
            Mat.zeros(3, 1, CvType.CV_32F).copyTo(NullM);
            Core.gemm(rot3x3.inv(), boardTrans,1, NullM, 0,transPlaneToCam,0);
            //multiply(rot3x3.inv(), boardTrans, transPlaneToCam);
            float positions[] = new float[3];
            float scale;
            float tmp1[]= new float[1];
            float tmp2[]= new float[1];
            Mat col = new Mat();
            col.create(3, 1, CvType.CV_32F);
            Log.i(TAG, "imgPt.rows() " + imgPt.rows());
            //Log.i(TAG, "imgPt " + imgPt);
            for( int i=0; i<imgPt.rows(); ++i ) {
                positions[0]=(float)imgPt.toArray()[i].x;
                positions[1]=(float)imgPt.toArray()[i].y;
                positions[2]= 1.0f;
                col.put(0, 0, positions);
                Core.gemm(Kinv, col, 1, NullM, 0, worldPtcam, 0);
                //Core.multiply(Kinv, col, worldPtcam);
                Core.gemm(rot3x3.inv(), worldPtcam, 1, NullM, 0, worldPtPlane, 0);
                //Core.multiply(rot3x3.inv(), worldPtcam, worldPtPlane);

                transPlaneToCam.get(2, 0, tmp1);
                worldPtPlane.get(2, 0, tmp2);
                scale=(float)(tmp1[0]/tmp2[0]);
                Core.multiply(worldPtPlane, new Scalar(scale), worldPtPlane);
                Core.subtract(worldPtPlane, transPlaneToCam, worldPtPlaneReproject);
                //worldPtPlaneReproject.put(2, 0, 0);
                fp=new Point3();
                worldPtPlaneReproject.get(0,0,tmp1);
                fp.x=tmp1[0];
                worldPtPlaneReproject.get(1,0,tmp1);
                fp.y=tmp1[0];
                fp.z=0;
                fpa.add(fp);
                //worldPt.push_back(worldPtPlaneReproject);
                Log.i(TAG, "worldPtPlaneReproject: " + worldPtPlaneReproject.dump());

            }
            worldPt.fromList(fpa);
            Log.i(TAG, "worldPt: " + worldPt.dump());
            return true;
        }


    }

    public boolean addCorners() {
        if (mPatternWasFound&&pPatternWasFound) {
            Mat rvec = new Mat();
            Mat tvec = new Mat();
            MatOfDouble CKT=new MatOfDouble(CK);

            //mCornersBuffer.add(mCorners.clone());// Wrong for Debug Only
            mCornersBuffer.add(pCorners.clone());
            Calib3d.solvePnP(mBoardPoints, mCorners, CM, CKT, rvec, tvec);
            MatOfPoint3f oCorners=new MatOfPoint3f();
            //Mat tmpOpoint= new Mat();
            //tmpOpoint.create(pCornersSize, 1, CvType.CV_32FC3);

            //backProject(rvec, tvec, mCorners, oCorners);// Wrong for Debug Only
            if(backProject(rvec, tvec, pCorners, oCorners))
            {
                oCornersBuffer.add(oCorners.clone());
                return true;
            }
        }
        return false;
    }

    private void drawPoints(Mat rgbaFrame) {
        Calib3d.drawChessboardCorners(rgbaFrame, mPatternSize, mCorners, mPatternWasFound);
        Calib3d.drawChessboardCorners(rgbaFrame, pPatternSize, pCorners, pPatternWasFound);
    }

    private void renderFrame(Mat rgbaFrame) {
        drawPoints(rgbaFrame);
        Imgproc.putText(rgbaFrame, "Captured: " + mCornersBuffer.size(), new Point(rgbaFrame.cols() / 3 * 2, rgbaFrame.rows() * 0.1),
                Core.FONT_HERSHEY_SIMPLEX, 1.0, new Scalar(255, 255, 0));
    }

    public int getCornersBufferSize() {
        return mCornersBuffer.size();
    }

    public boolean isCalibrated() {
        return mIsCalibrated;
    }

    public void setCalibrated() {
        mIsCalibrated = true;
    }
}



