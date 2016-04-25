package i2r.snap2inspect;

import java.io.File;
import java.io.FileOutputStream;
import java.text.DecimalFormat;
import java.util.ArrayList;
import java.util.List;

import org.opencv.android.JavaCameraView;
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
    private List<Point3> DetectedPoints = new ArrayList<Point3>();
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

        int DotSize = 10;
        org.opencv.core.Size pImageSize=new org.opencv.core.Size(1280,720);
        DispImg = new Mat();
        Mat.zeros(pImageSize, CvType.CV_8UC1).copyTo(DispImg);

        Point markp1 = new Point(640,240);
        Point markp2 = new Point(540,480);
        Point markp3 = new Point(740,480);

        List<org.opencv.core.Point> ptmp=new ArrayList<Point>();
        ptmp.add(markp1);
        ptmp.add(markp2);
        ptmp.add(markp3);

        ProjPoint = new MatOfPoint2f();
        ProjPoint.fromList(ptmp);
        RecProjPoint = new MatOfPoint2f();
        Imgproc.undistortPoints(ProjPoint, RecProjPoint, PM, PK, R2, P2);

        Imgproc.circle(DispImg, markp1, DotSize, new Scalar(255, 0, 0), -1);
        Imgproc.circle(DispImg, markp2, DotSize, new Scalar(255, 0, 0), -1);
        Imgproc.circle(DispImg, markp3, DotSize, new Scalar(255, 0, 0), -1);
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
            MatOfPoint2f MatPs=MatOfKeyPoint2MatOfPoint(keypoints);


        //kpc_in.fromList(pint);
            Imgproc.undistortPoints(MatPs, MatPs, CM, CK, R1, P1);

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
                Imgproc.circle(rgbi, ptpan, 5, new Scalar(255, 0, 0), 2);
            }
            DecimalFormat twoPlaces = new DecimalFormat("0.00");
            Mat xyd1 = new Mat(4, 1, CvType.CV_32F);
            Mat NullM = new Mat();
            Mat.zeros(3, 1, CvType.CV_32F).copyTo(NullM);
            Point3 dpt = new Point3();
            float xyd1a[] = new float[4];
            for (Point ptimg : MatPs.toArray()) {
                Imgproc.circle(rgbi, ptimg, 5, new Scalar(0, 255, 0), 2);
                for (Point ptpan : RecProjPoint.toArray()) {
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
                        //DetectedPoints.add(dpt);
                        Imgproc.putText(rgbi, "(" + twoPlaces.format(dpt.x) + ", " + twoPlaces.format(dpt.y) + ", " + twoPlaces.format(dpt.z) + ")", ptpan,
                                Core.FONT_HERSHEY_SIMPLEX, 1.0, new Scalar(255, 0, 0));
                        //break;
                    }

                }
            }
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
