package i2r.snap2inspect;

import java.io.File;
import java.io.FileOutputStream;
import java.text.DecimalFormat;
import java.util.ArrayList;
import java.util.List;

import org.opencv.android.JavaCameraView;
import org.opencv.calib3d.Calib3d;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.KeyPoint;
import org.opencv.core.Mat;
import org.opencv.core.MatOfByte;
import org.opencv.core.MatOfKeyPoint;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Point3;
import org.opencv.core.Scalar;
import org.opencv.features2d.FeatureDetector;
import org.opencv.features2d.Features2d;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;

import android.content.Context;
import android.hardware.Camera;
import android.hardware.Camera.PictureCallback;
import android.hardware.Camera.Size;
import android.os.Environment;
import android.util.AttributeSet;
import android.util.Log;

public class Tutorial3View extends JavaCameraView implements PictureCallback {

    private static final String TAG = "Sample::Tutorial3View";
    private String mPictureFileName;
    private FeatureDetector blobDetector;
    private MatOfKeyPoint keypoints;
    Mat Q;
    Mat P1;
    Mat P2;
    Mat R1;
    Mat R2;
    Mat PM;
    Mat PK;
    Mat CM;
    Mat CK;
    Mat DispImg;
    MatOfPoint2f ProjPoint;
    MatOfPoint2f RecProjPoint;
    ArrayList<Point3> DetectedPoints = new ArrayList<Point3>();
    Mat irgb;
    boolean p_status;

    float THRES_X=10;

    public Tutorial3View(Context context, AttributeSet attrs) {
        super(context, attrs);
    }

    public Mat getFrame(Mat rgbi) {
        return rgbi;
    }

    public MatOfPoint2f MatOfKeyPoint2MatOfPoint(MatOfKeyPoint inMOK)
    {
        KeyPoint[] kpa;
        kpa=inMOK.toArray();
        List<Point> pint=new ArrayList<Point>();
        for(int i=0;i<kpa.length;i++){
            pint.add(kpa[i].pt);
        }
        MatOfPoint2f outMOP = new MatOfPoint2f();
        outMOP.fromList(pint);
        return outMOP;

    }

    public void setup() {
        blobDetector = FeatureDetector.create(FeatureDetector.SIMPLEBLOB);
        String fileName = Environment.getExternalStorageDirectory().getPath() + "/blobparams2.yml";
        blobDetector.read(fileName);
        keypoints = new MatOfKeyPoint();

        i2r.snap2inspect.TaFileStorage taFileStorage= new i2r.snap2inspect.TaFileStorage();
        String root = Environment.getExternalStorageDirectory() +"";
        taFileStorage.open(root + "/PCC.xml");
        CM = taFileStorage.readMat("CM");
        CK = taFileStorage.readMat("CK");
        PM = taFileStorage.readMat("PM");
        PK = taFileStorage.readMat("PK");
        P1 = taFileStorage.readMat("P1");
        P2 = taFileStorage.readMat("P2");
        R1 = taFileStorage.readMat("R1");
        R2 = taFileStorage.readMat("R2");
        Q = taFileStorage.readMat("Q");
        taFileStorage.release();

        int DotSize = 5;
        org.opencv.core.Size pImageSize=new org.opencv.core.Size(1280,720);
        DispImg = new Mat();
        Mat.zeros(pImageSize, CvType.CV_8UC1).copyTo(DispImg);


        //origin
        //Point markp1 = new Point(640,240);
        //Point markp2 = new Point(540,480);
        //Point markp3 = new Point(740,480);
       // Point markp4 = new Point(640,400);
//        Point markp1 = new Point(640,240);
//        Point markp2 = new Point(440,580);
//        Point markp3 = new Point(840,580);
//        Point markp4 = new Point(640,450);
        Point markp1 = new Point(640,150);
        Point markp2 = new Point(280,600);
        Point markp3 = new Point(1000,600);
        Point markp4 = new Point(640,460);
        List<org.opencv.core.Point> ptmp=new ArrayList<Point>();
        ptmp.add(markp1);
        ptmp.add(markp2);
        ptmp.add(markp3);
        ptmp.add(markp4);

        ProjPoint = new MatOfPoint2f();
        ProjPoint.fromList(ptmp);
        RecProjPoint = new MatOfPoint2f();
        Imgproc.undistortPoints(ProjPoint, RecProjPoint, PM, PK, R2, P2);
        //origin

// testing

//        //        Calib3d.projectPoints();
//        Mat mapx=new Mat();
//        Mat mapy=new Mat();
//        //Calib3d.projectPoints();
//        Calib3d.initUndistortRectifyMap(PM, PK, R2, P2, pImageSize, CvType.CV_32FC1, mapx, mapy);
//
//        Point markc_p = new Point(640,450);
//        List<org.opencv.core.Point> ptmp=new ArrayList<Point>();
//        ptmp.add(markc_p);
//        ProjPoint = new MatOfPoint2f();
//        ProjPoint.fromList(ptmp);
//        RecProjPoint = new MatOfPoint2f();
//        Imgproc.undistortPoints(ProjPoint, RecProjPoint, PM, PK, R2, P2);
//        RecProjPoint.toArray()[0].x;


//testing end
        Imgproc.circle(DispImg, markp1, DotSize, new Scalar(255, 0, 0), -1);
        Imgproc.circle(DispImg, markp2, DotSize, new Scalar(255, 0, 0), -1);
        Imgproc.circle(DispImg, markp3, DotSize, new Scalar(255, 0, 0), -1);
        Imgproc.circle(DispImg, markp4, DotSize, new Scalar(255, 0, 0), -1);
     }

    public boolean isBusy() {return p_status;}



    public void measureFrame(Mat gi, Mat rgbi)
    {
        //Imgproc.cvtColor(rgbi, irgb, Imgproc.COLOR_BGR2GRAY);
        blobDetector.detect(gi, keypoints);
        if(!keypoints.empty()){
            Imgproc.cvtColor(rgbi,rgbi,Imgproc.COLOR_BGRA2BGR);
            Features2d.drawKeypoints(rgbi, keypoints, rgbi);
//        KeyPoint[] kpa;
//        kpa=keypoints.toArray();
//        List<Point> pint=new ArrayList<Point>();
//        for(int i=0;i<kpa.length;i++){
//            pint.add(kpa[i].pt);
//        }
//        MatOfPoint2f kpc_in = new MatOfPoint2f();
            MatOfPoint2f MatPsO=MatOfKeyPoint2MatOfPoint(keypoints);

            MatOfPoint2f MatPs=new MatOfPoint2f();
        //kpc_in.fromList(pint);
            Imgproc.undistortPoints(MatPsO, MatPs, CM, CK, R1, P1);

//        TaFileStorage taFileStorage=new TaFileStorage();
//        String root = Environment.getExternalStorageDirectory() +"";
//        taFileStorage.create(root + "/result_pts.xml");
//        Mat OutMat=new Mat();
//        MatPs.copyTo(OutMat);
//        OutMat.convertTo(OutMat, CvType.CV_32F);
//        Log.i(TAG, "ImgPt " + OutMat);
//        taFileStorage.writeMat("ImgPt", OutMat);
//        RecProjPoint.copyTo(OutMat);
//        OutMat.convertTo(OutMat, CvType.CV_32F);
//        taFileStorage.writeMat("ProjPt",OutMat);
//        taFileStorage.release();

            for(Point ptpan : RecProjPoint.toArray())
            {
                ptpan.x=ptpan.x;
                Imgproc.circle(rgbi, ptpan, 5, new Scalar(0, 0, 255), 2);
            }
            DecimalFormat twoPlaces = new DecimalFormat("0.0");
            Mat xyd1 = new Mat(4, 1, CvType.CV_32F);
            Mat NullM = new Mat();
            Mat.zeros(3, 1, CvType.CV_32F).copyTo(NullM);
            Point3 dpt = new Point3();
            float xyd1a[] = new float[4];
            int img_idx;
            for (Point ptpan : RecProjPoint.toArray()) {
                img_idx=0;
                for (Point ptimg : MatPs.toArray()) {
                    Imgproc.circle(rgbi, ptimg, 5, new Scalar(0, 255, 0), 2);
                    if (Math.abs(ptimg.x - ptpan.x) < THRES_X) {
                        xyd1a[0] = (float) ptimg.x;
                        xyd1a[1] = (float) ptimg.y;
                        xyd1a[2] = (float) (ptimg.y - ptpan.y);
                        xyd1a[3] = 1;
                        xyd1.put(0, 0, xyd1a);
                        Core.gemm(Q, xyd1, 1, NullM, 0, xyd1, 0);
                        xyd1.get(0, 0, xyd1a);

                        dpt.x = xyd1a[0] / xyd1a[3];
                        dpt.y = xyd1a[1] / xyd1a[3];
                        dpt.z = xyd1a[2] / xyd1a[3];
                        DetectedPoints.add(dpt.clone());
                        Imgproc.putText(rgbi, "(" + twoPlaces.format(dpt.x) + ", " + twoPlaces.format(dpt.y) + ", " + twoPlaces.format(dpt.z) + ")", MatPsO.toArray()[img_idx],
                                Core.FONT_HERSHEY_SIMPLEX, 1.0, new Scalar(255, 0, 0));
                        //break
                        Imgproc.line(rgbi, MatPsO.toArray()[img_idx], ptpan,new Scalar(255, 0, 0), 1);
                        Imgproc.circle(rgbi,  MatPsO.toArray()[img_idx], 5, new Scalar(255, 0, 0), 2);
                        break;
                    }
                    img_idx++;
                }

            }
            double V, A, D;
            double x1,x2,x3,x4,y1,y2,y3,y4,z1,z2,z3,z4,c1,c2,c3;
            if(DetectedPoints.size()==4) {
                x1=DetectedPoints.get(0).x;
                y1=DetectedPoints.get(0).y;
                z1=DetectedPoints.get(0).z;
                x2=DetectedPoints.get(1).x-x1;
                y2=DetectedPoints.get(1).y-y1;
                z2=DetectedPoints.get(1).z-z1;
                x3=DetectedPoints.get(2).x-x1;
                y3=DetectedPoints.get(2).y-y1;
                z3=DetectedPoints.get(2).z-z1;
                x4=DetectedPoints.get(3).x-x1;
                y4=DetectedPoints.get(3).y-y1;
                z4=DetectedPoints.get(3).z-z1;
                c1=y2*z3-z2*y3;
                c2=z2*x3-x2*z3;
                c3=x2*y3-y2*x3;
                V = Math.abs(c1*x4+c2*y4+c3*z4)/6;
                A = Math.abs(x2*x3+y2*y3+z2*z3)/2;
                D = 3*V/A;
                //Imgproc.putText(rgbi, twoPlaces.format(x2) + " " + twoPlaces.format(y2) + " " + twoPlaces.format(z2) + " " + twoPlaces.format(x3) + " " + twoPlaces.format(y3) + " " + twoPlaces.format(z3), new Point(rgbi.cols() / 5 * 1, rgbi.rows() * 0.3), Core.FONT_HERSHEY_SIMPLEX, 1.0, new Scalar(255, 0, 0));
                Imgproc.putText(rgbi, "Depth: "+ twoPlaces.format(D) + " Area: " + twoPlaces.format(A) + " Volume: " + twoPlaces.format(V), new Point(rgbi.cols() / 8 * 1, rgbi.rows() * 0.1), Core.FONT_HERSHEY_SIMPLEX, 1.0, new Scalar(255, 0, 0));
            }
            DetectedPoints.clear();


//            int img_idx=0;
//            for (Point ptimg : MatPs.toArray()) {
//                Imgproc.circle(rgbi, ptimg, 5, new Scalar(0, 255, 0), 2);
//                for (Point ptpan : RecProjPoint.toArray()) {
//                        if (Math.abs(ptimg.x - ptpan.x) < THRES_X) {
//                        xyd1a[0] = (float) ptimg.x;
//                        xyd1a[1] = (float) ptimg.y;
//                        xyd1a[2] = (float) (ptimg.y - ptpan.y);
//                        xyd1a[3] = 1;
//                        xyd1.put(0, 0, xyd1a);
//                        Core.gemm(Q, xyd1, 1, NullM, 0, xyd1, 0);
//                        xyd1.get(0, 0, xyd1a);
//                        dpt.x = xyd1a[0] / xyd1a[3];
//                        dpt.y = xyd1a[1] / xyd1a[3];
//                        dpt.z = xyd1a[2] / xyd1a[3];
//                        //DetectedPoints.add(dpt);
//                        Imgproc.putText(rgbi, "(" + twoPlaces.format(dpt.x) + ", " + twoPlaces.format(dpt.y) + ", " + twoPlaces.format(dpt.z) + ")", MatPsO.toArray()[img_idx],
//                                Core.FONT_HERSHEY_SIMPLEX, 1.0, new Scalar(255, 0, 0));
//                        //break
//                        Imgproc.circle(rgbi,  MatPsO.toArray()[img_idx], 5, new Scalar(255, 0, 0), 2);
//                    }
//
//                }
//                img_idx++;
//            }
        }


    }

    public void takePicture(final String fileName) {
        Log.i(TAG, "Taking picture");
        this.mPictureFileName = fileName;
        // Postview and jpeg are sent in the same buffers if the queue is not empty when performing a capture.
        // Clear up buffers to avoid mCamera.takePicture to be stuck because of a memory issue
        mCamera.setPreviewCallback(null);

        // PictureCallback is implemented by the current class
        p_status=true;
        mCamera.takePicture(null, null, this);
    }

    @Override
    public void onPictureTaken(byte[] data, Camera camera) {
        Log.i(TAG, "Saving a bitmap to file");
        // The camera preview was automatically stopped. Start it again.
        mCamera.startPreview();
        mCamera.setPreviewCallback(this);
        //loadCalibParam();
        //FeatureDetector blobDetector;
        //blobDetector = FeatureDetector.create(FeatureDetector.SIMPLEBLOB);
        //String fileName = Environment.getExternalStorageDirectory().getPath() + "/blobparams2.yml";
        //blobDetector.read(fileName);
        //MatOfKeyPoint keypoints = new MatOfKeyPoint();
//        Mat irgb = Imgcodecs.imdecode(new MatOfByte(data), Imgcodecs.CV_LOAD_IMAGE_UNCHANGED);
//        Imgproc.cvtColor(irgb, irgb, Imgproc.COLOR_BGR2GRAY);
//        blobDetector.detect(irgb, keypoints);
//        Features2d.drawKeypoints(irgb, keypoints, irgb);
//
//        //Imgcodecs.imwrite(mPictureFileName, irgb);
//        KeyPoint[] kpa;
//        kpa=keypoints.toArray();
//        List<org.opencv.core.Point> pint=new ArrayList<Point>();
//        for(int i=0;i<kpa.length;i++){
//           pint.add(kpa[i].pt);
//        }
//        MatOfPoint2f kp_in = new MatOfPoint2f();
//        MatOfPoint2f kp_out = new MatOfPoint2f();
//        kp_in.fromList(pint);
//        Imgproc.undistortPoints(kp_in, kp_out, CM, CK, R1, P1);
//        p_status=false;
        //Imgproc.undistortPoints(keypoints,keypoints, CM, CdistCoeffs, RM, PM);

        /*        irgb.release();
        keypoints.release();*/

          // Write the image in a file (in jpeg format)
//        try {
//            FileOutputStream fos = new FileOutputStream(mPictureFileName);
//
//            fos.write(data);
//            fos.close();
//
//        } catch (java.io.IOException e) {
//            Log.e("PictureDemo", "Exception in photoCallback", e);
//        }

    }
}
