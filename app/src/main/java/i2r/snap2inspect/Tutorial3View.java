package i2r.snap2inspect;

import java.io.File;
import java.io.FileOutputStream;
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

    Mat irgb;
    boolean p_status;

    public Tutorial3View(Context context, AttributeSet attrs) {
        super(context, attrs);


    }

    public void loadCalibParam() {
        blobDetector = FeatureDetector.create(FeatureDetector.SIMPLEBLOB);
        String fileName = Environment.getExternalStorageDirectory().getPath() + "/blobparams.yml";
        blobDetector.read(fileName);
        keypoints = new MatOfKeyPoint();

        i2r.snap2inspect.TaFileStorage taFileStorage= new i2r.snap2inspect.TaFileStorage();
        String root = Environment.getExternalStorageDirectory() +"";
        taFileStorage.open(root + "/ProCamPara.xml");
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
    }

    public List<String> getEffectList() {
        return mCamera.getParameters().getSupportedColorEffects();
    }

    public boolean isEffectSupported() {
        return (mCamera.getParameters().getColorEffect() != null);
    }

    public String getEffect() {
        return mCamera.getParameters().getColorEffect();
    }

    public void setEffect(String effect) {
        Camera.Parameters params = mCamera.getParameters();
        params.setColorEffect(effect);
        mCamera.setParameters(params);
    }

    public List<Size> getResolutionList() {
        return mCamera.getParameters().getSupportedPreviewSizes();
    }

    public void setResolution(Size resolution) {
        disconnectCamera();
        mMaxHeight = resolution.height;
        mMaxWidth = resolution.width;
        connectCamera(getWidth(), getHeight());
    }

    public Size getResolution() {
        return mCamera.getParameters().getPreviewSize();
    }

    public boolean isBusy() {return p_status;}

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
        loadCalibParam();
        FeatureDetector blobDetector;
        blobDetector = FeatureDetector.create(FeatureDetector.SIMPLEBLOB);
        String fileName = Environment.getExternalStorageDirectory().getPath() + "/blobparams2.yml";
        blobDetector.read(fileName);
        MatOfKeyPoint keypoints = new MatOfKeyPoint();
        Mat irgb = Imgcodecs.imdecode(new MatOfByte(data), Imgcodecs.CV_LOAD_IMAGE_UNCHANGED);
        Imgproc.cvtColor(irgb, irgb, Imgproc.COLOR_BGR2GRAY);
        blobDetector.detect(irgb, keypoints);
        Features2d.drawKeypoints(irgb, keypoints, irgb);

        //Imgcodecs.imwrite(mPictureFileName, irgb);
        KeyPoint[] kpa;
        kpa=keypoints.toArray();
        List<org.opencv.core.Point> pint=new ArrayList<Point>();
        for(int i=0;i<kpa.length;i++){
           pint.add(kpa[i].pt);
        }
        MatOfPoint2f kp_in = new MatOfPoint2f();
        MatOfPoint2f kp_out = new MatOfPoint2f();
        kp_in.fromList(pint);
        Imgproc.undistortPoints(kp_in, kp_out, CM, CK, R1, P1);
        p_status=false;
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
