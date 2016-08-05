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
import org.opencv.utils.Converters;

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
    //Mat PCam1;
    //Mat PCam2;
    Mat R1;
    Mat R2;
    Mat PM;
    Mat PK;
    Mat CM;
    Mat CK;
    Mat R;
    Mat T;
    Mat E;
    Mat DispImg;
    MatOfPoint2f ProjPoint;
    MatOfPoint2f RecProjPoint;
    ArrayList<Point3> DetectedPoints = new ArrayList<Point3>();
    boolean p_status;

    float THRES_X=20;


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
        R = taFileStorage.readMat("R");
        T = taFileStorage.readMat("T");
        E = taFileStorage.readMat("E");
        taFileStorage.release();

        int DotSize = 3;
        org.opencv.core.Size pImageSize=new org.opencv.core.Size(1280,720);
        DispImg = new Mat();
        Mat.zeros(pImageSize, CvType.CV_8UC1).copyTo(DispImg);


        //origin
        //Point markp1 = new Point(640,240);
        //Point markp2 = new Point(540,480);
        //Point markp3 = new Point(740,480);
       // Point markp4 = new Point(640,400);
        Point markp1 = new Point(640,240);
        Point markp2 = new Point(440,580);
        Point markp3 = new Point(840,580);
        Point markp4 = new Point(640,450);
//        Point markp1 = new Point(640,150);
//        Point markp2 = new Point(280,600);
//        Point markp3 = new Point(1000,600);
//        Point markp4 = new Point(640,460);
        List<org.opencv.core.Point> ptmp=new ArrayList<Point>();
        ptmp.add(markp1);
        ptmp.add(markp2);
        ptmp.add(markp3);
        ptmp.add(markp4);

        ProjPoint = new MatOfPoint2f();
        ProjPoint.fromList(ptmp);
        RecProjPoint = new MatOfPoint2f();
        Imgproc.undistortPoints(ProjPoint, RecProjPoint, PM, PK, R2, P2);

        Imgproc.circle(DispImg, markp1, DotSize, new Scalar(255, 0, 0), -1);
        Imgproc.circle(DispImg, markp2, DotSize, new Scalar(255, 0, 0), -1);
        Imgproc.circle(DispImg, markp3, DotSize, new Scalar(255, 0, 0), -1);
        Imgproc.circle(DispImg, markp4, DotSize, new Scalar(255, 0, 0), -1);
     }

    public boolean isBusy() {return p_status;}


    public void measureFrame(Mat gi, Mat rgbi)
    {
        Log.i(TAG, "start blobDetector");
        blobDetector.detect(gi, keypoints);
        if(!keypoints.empty()){
            Imgproc.cvtColor(rgbi,rgbi,Imgproc.COLOR_BGRA2BGR);
            Features2d.drawKeypoints(rgbi, keypoints, rgbi);
            MatOfPoint2f MatPsO=MatOfKeyPoint2MatOfPoint(keypoints);

            MatOfPoint2f MatPs=new MatOfPoint2f();
            Imgproc.undistortPoints(MatPsO, MatPs, CM, CK, R1, P1);

            for(Point ptpan : ProjPoint.toArray())
            {
                Imgproc.circle(rgbi, ptpan, 5, new Scalar(0, 0, 255), 2);
            }
            DecimalFormat twoPlaces = new DecimalFormat("0.0");
            Mat xyd1 = new Mat(4, 1, CvType.CV_32F);
            Mat NullM = new Mat();
            Mat.zeros(3, 1, CvType.CV_32F).copyTo(NullM);
            Point3 dpt = new Point3();
            float xyd1a[] = new float[4];
            int img_cor[] = new int[4];
            int[][] img_cor_tmp = new int[4][6];
            int img_idx;
            int ldm_idx;
            int buf_idx;
            ldm_idx=0;
            for(int i=0;i<4;i++) {
                img_cor[i]=-1;
            }
            Log.i(TAG, "blobDetector complete with point: Cols:"+ MatPsO.cols()+ " Rows: " + MatPsO.rows());
            Log.i(TAG, "detect corresponding points on recified space");
            for (Point ptpan : RecProjPoint.toArray()) {
                img_idx = 0;
                buf_idx = 0;
                //Log.i(TAG, "ldm_idx: "+ldm_idx);
                for (Point ptimg : MatPs.toArray()) {
                    //Log.i(TAG, "img_idx: "+img_idx);
                    if (Math.abs(ptimg.x - ptpan.x) < THRES_X) {
                         if(buf_idx<5) {
                             //Log.i(TAG, "buf_idx: "+buf_idx);
                             img_cor_tmp[ldm_idx][buf_idx+1]=img_idx;
                             //Log.i(TAG, "buf_idx: "+(buf_idx+1) + " assigned");
                             buf_idx++;
                             img_cor_tmp[ldm_idx][0]=buf_idx;
                        }
                        else
                        {
                           //Log.i(TAG, "buf_idx: "+buf_idx);
                            Log.i(TAG, "Break!!");
                            break;
                        }
                    }
                    img_idx++;
                }
                //Log.i(TAG, "ldm_idx: "+ldm_idx + "completed");
                ldm_idx++;
            }
            Log.i(TAG, "refine corresponding points with prior knowledge");
            double MINY=720;
            double MINX=1280;
            double MAXX=0;
            for (ldm_idx=0; ldm_idx<4;ldm_idx++) {
                for (int i=0; i<img_cor_tmp[ldm_idx][0]; i++) {
                    if (ldm_idx==0) {
                        if(MINY>MatPsO.toArray()[img_cor_tmp[ldm_idx][i+1]].y) {
                            MINY=MatPsO.toArray()[img_cor_tmp[ldm_idx][i+1]].y;
                            img_cor[ldm_idx]=img_cor_tmp[ldm_idx][i+1];
                        }
                    }

                    if (ldm_idx==1) {
                        if(MINX>MatPsO.toArray()[img_cor_tmp[ldm_idx][i+1]].x) {
                            MINX=MatPsO.toArray()[img_cor_tmp[ldm_idx][i+1]].x;
                            img_cor[ldm_idx]=img_cor_tmp[ldm_idx][i+1];
                        }
                    }

                    if (ldm_idx==2) {
                        if(MAXX<MatPsO.toArray()[img_cor_tmp[ldm_idx][i+1]].x) {
                            MAXX=MatPsO.toArray()[img_cor_tmp[ldm_idx][i+1]].x;
                            img_cor[ldm_idx]=img_cor_tmp[ldm_idx][i+1];
                        }
                    }

                    if (ldm_idx==3) {
                        if(img_cor_tmp[ldm_idx][i+1]!=img_cor[0]&&img_cor_tmp[ldm_idx][i+1]!=img_cor[1]&&img_cor_tmp[ldm_idx][i+1]!=img_cor[2]) {
                            img_cor[ldm_idx]=img_cor_tmp[ldm_idx][i+1];
                        }
                    }
                }
            }
            Log.i(TAG, "calculate the 3D struture based on corresponding points");
            float fs[] = new float[1];
            float sc;
            Mat pc1=new Mat(2,1,CvType.CV_32FC1);
            Mat pc2=new Mat(2,1,CvType.CV_32FC1);
            Mat p4=new Mat();
            ldm_idx=0;
            for(Point ptpan : RecProjPoint.toArray()) {
                if(img_cor[ldm_idx]!=-1) {
                    // triangulation in rectified space
                    xyd1a[0] = (float) MatPs.toArray()[img_cor[ldm_idx]].x;
                    xyd1a[1] = (float) MatPs.toArray()[img_cor[ldm_idx]].y;
                    xyd1a[2] = (float) (MatPs.toArray()[img_cor[ldm_idx]].y - ptpan.y);
                    xyd1a[3] = 1;
                    xyd1.put(0, 0, xyd1a);
                    Core.gemm(Q, xyd1, 1, NullM, 0, xyd1, 0);
                    xyd1.get(0, 0, xyd1a);
                    dpt.x = xyd1a[0] / xyd1a[3];
                    dpt.y = xyd1a[1] / xyd1a[3];
                    dpt.z = xyd1a[2] / xyd1a[3];

                    DetectedPoints.add(dpt.clone());
                    Imgproc.putText(rgbi, "(" + twoPlaces.format(dpt.x) + ", " + twoPlaces.format(dpt.y) + ", " + twoPlaces.format(dpt.z) + ")", ProjPoint.toArray()[ldm_idx],
                            Core.FONT_HERSHEY_SIMPLEX, 1.0, new Scalar(255, 0, 0));
                    Imgproc.line(rgbi, MatPsO.toArray()[img_cor[ldm_idx]], ProjPoint.toArray()[ldm_idx], new Scalar(0, 255, 0), 1);
                    Imgproc.circle(rgbi, MatPsO.toArray()[img_cor[ldm_idx]], 5, new Scalar(255, 0, 0), 2);
                }
                ldm_idx++;
            }

            Log.i(TAG, "calculate the depth");
            double A, D;
            double x1,x2,x3,x4,y1,y2,y3,y4,z1,z2,z3,z4,la,lb,lc,ld;
            if(DetectedPoints.size()==4) {
                x2=DetectedPoints.get(0).x;
                y2=DetectedPoints.get(0).y;
                z2=DetectedPoints.get(0).z;
                x3=DetectedPoints.get(1).x;
                y3=DetectedPoints.get(1).y;
                z3=DetectedPoints.get(1).z;
                x4=DetectedPoints.get(2).x;
                y4=DetectedPoints.get(2).y;
                z4=DetectedPoints.get(2).z;
                x1=DetectedPoints.get(3).x;
                y1=DetectedPoints.get(3).y;
                z1=DetectedPoints.get(3).z;
                la = y2 * (z3 - z4) + y3 * (z4 - z2) + y4 * (z2 - z3);
                lb = z2 * (x3 - x4) + z3 * (x4 - x2) + z4 * (x2 - x3);
                lc = x2 * (y3 - y4) + x3 * (y4 - y2) + x4 * (y2 - y3);
                ld = x2 * (y3 * z4 - y4 * z3) + x3 * (y4 * z2 - y2 * z4) + x4 * (y2 * z3 - y3 * z2);
                A=Math.sqrt(la*la+lb*lb+lc*lc);
                D=(la*x1+lb*y1+lc*z1-ld)/A;
                Imgproc.putText(rgbi, "Depth: "+ twoPlaces.format(D) + "mm", new Point(rgbi.cols() / 16 * 1, rgbi.rows() * 0.1), Core.FONT_ITALIC, 2.5, new Scalar(255, 255, 0));
            }
            DetectedPoints.clear();
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
