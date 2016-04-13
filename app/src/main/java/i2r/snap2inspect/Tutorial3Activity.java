package i2r.snap2inspect;

import java.text.SimpleDateFormat;
import java.util.Date;
import org.opencv.android.BaseLoaderCallback;
import org.opencv.android.CameraBridgeViewBase.CvCameraViewFrame;
import org.opencv.android.LoaderCallbackInterface;
import org.opencv.android.OpenCVLoader;
import org.opencv.calib3d.Calib3d;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.android.CameraBridgeViewBase.CvCameraViewListener2;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import android.annotation.SuppressLint;
import android.app.Activity;
import android.content.Context;
import android.media.MediaRouter;
import android.os.Bundle;
import android.os.Environment;
import android.util.Log;
import android.view.Display;
import android.view.MotionEvent;
import android.view.SurfaceView;
import android.view.View;
import android.view.View.OnTouchListener;
import android.view.WindowManager;
import android.widget.Button;
import android.widget.CompoundButton;
import android.widget.Switch;
import android.widget.Toast;


public class Tutorial3Activity extends Activity implements CvCameraViewListener2, OnTouchListener, CompoundButton.OnCheckedChangeListener {
    private static final String TAG = "Snap2inspect::Activity";

    private i2r.snap2inspect.Tutorial3View mOpenCvCameraView;
    private i2r.snap2inspect.CamProjCalib mCamProjCalib;
    private Button mButton;
    private Switch mSwitch;
    private MediaRouter mMediaRouter;
    private i2r.snap2inspect.SamplePresentation mPresentation;
    private Mat FrameG;
    private Mat FrameRGBA;
    private boolean init_stat=false;

    @Override
    public void onCheckedChanged(CompoundButton buttonView, boolean isChecked) {
        if (mPresentation != null) {
            Toast.makeText(this, "Monitored switch is " + (isChecked ? "on" : "off"),
                    Toast.LENGTH_SHORT).show();

            if (isChecked) {
                mPresentation.setImage2();
            } else {
                mPresentation.setImageDynamic(mCamProjCalib.pProjImage);
            }
        }
        else
        {
            Toast.makeText(this, "No projector is connected",
                    Toast.LENGTH_SHORT).show();
        }
    }

    private BaseLoaderCallback mLoaderCallback = new BaseLoaderCallback(this) {
        @Override
        public void onManagerConnected(int status) {
            switch (status) {
                case LoaderCallbackInterface.SUCCESS: {
                    Log.i(TAG, "OpenCV loaded successfully");
                    mOpenCvCameraView.enableView();
                    mOpenCvCameraView.setOnTouchListener(Tutorial3Activity.this);
                }
                break;
                default: {
                    super.onManagerConnected(status);
                }
                break;
            }
        }
    };

    public Tutorial3Activity() {
        Log.i(TAG, "Instantiated new " + this.getClass());
    }

    /**
     * Called when the activity is first created.
     */
    @Override
    public void onCreate(Bundle savedInstanceState) {
        Log.i(TAG, "called onCreate");
        super.onCreate(savedInstanceState);
        getWindow().addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);

        setContentView(R.layout.tutorial3_surface_view);
        mOpenCvCameraView = (i2r.snap2inspect.Tutorial3View) findViewById(R.id.tutorial3_activity_java_surface_view);
        mOpenCvCameraView.setVisibility(SurfaceView.VISIBLE);
        mOpenCvCameraView.setCvCameraViewListener(this);
        mCamProjCalib = new CamProjCalib();

        // BEGIN_INCLUDE(getMediaRouter)
        // Get the MediaRouter service
        mMediaRouter = (MediaRouter) getSystemService(Context.MEDIA_ROUTER_SERVICE);
        mButton = (Button) findViewById(R.id.button1);
        mButton.setOnClickListener(new View.OnClickListener() {

            @Override
            public void onClick(View v) {
                if (mOpenCvCameraView.isBusy()) {
                    mButton.setEnabled(false);
                } else {
                    takeMeasurement();
                }
            }
        });


        mSwitch = (Switch) findViewById(R.id.monitored_switch);
        if (mSwitch != null) {
            mSwitch.setOnCheckedChangeListener(this);
        }

        System.loadLibrary("native");
    }


    private final MediaRouter.SimpleCallback mMediaRouterCallback =
            new MediaRouter.SimpleCallback() {

                // BEGIN_INCLUDE(SimpleCallback)
                /**
                 * A new route has been selected as active. Disable the current
                 * route and enable the new one.
                 */
                @Override
                public void onRouteSelected(MediaRouter router, int type, MediaRouter.RouteInfo info) {
                    updatePresentation();
                }

                /**
                 * The route has been unselected.
                 */
                @Override
                public void onRouteUnselected(MediaRouter router, int type, MediaRouter.RouteInfo info) {
                    updatePresentation();

                }

                /**
                 * The route's presentation display has changed. This callback
                 * is called when the presentation has been activated, removed
                 * or its properties have changed.
                 */
                @Override
                public void onRoutePresentationDisplayChanged(MediaRouter router, MediaRouter.RouteInfo info) {
                    updatePresentation();
                }
                // END_INCLUDE(SimpleCallback)
            };

    /**
     * Updates the displayed presentation to enable a secondary screen if it has
     * been selected in the {@link android.media.MediaRouter} for the
     * {@link android.media.MediaRouter#ROUTE_TYPE_LIVE_VIDEO} type. If no screen has been
     * selected by the {@link android.media.MediaRouter}, the current screen is disabled.
     * Otherwise a new {@link SamplePresentation} is initialized and shown on
     * the secondary screen.
     */
    private void updatePresentation() {

        // BEGIN_INCLUDE(updatePresentationInit)
        // Get the selected route for live video
        MediaRouter.RouteInfo selectedRoute = mMediaRouter.getSelectedRoute(
                MediaRouter.ROUTE_TYPE_LIVE_VIDEO);

        // Get its Display if a valid route has been selected
        Display selectedDisplay = null;
        if (selectedRoute != null) {
            selectedDisplay = selectedRoute.getPresentationDisplay();
        }
        // END_INCLUDE(updatePresentationInit)

        // BEGIN_INCLUDE(updatePresentationDismiss)
        /*
         * Dismiss the current presentation if the display has changed or no new
         * route has been selected
         */
        if (mPresentation != null && mPresentation.getDisplay() != selectedDisplay) {
            mPresentation.dismiss();
            mPresentation = null;
        }
        // END_INCLUDE(updatePresentationDismiss)

        // BEGIN_INCLUDE(updatePresentationNew)
        /*
         * Show a new presentation if the previous one has been dismissed and a
         * route has been selected.
         */
        if (mPresentation == null && selectedDisplay != null) {

            // Initialise a new Presentation for the Display
            mPresentation = new i2r.snap2inspect.SamplePresentation(this, selectedDisplay);
            //mPresentation.setOnDismissListener(mOnDismissListener);

            // Try to show the presentation, this might fail if the display has
            // gone away in the mean time
            try {
                mPresentation.show();
            } catch (WindowManager.InvalidDisplayException ex) {
                // Couldn't show presentation - display was already removed
                mPresentation = null;
            }
        }
        // END_INCLUDE(updatePresentationNew)

    }


    @Override
    public void onPause() {
        super.onPause();
        if (mOpenCvCameraView != null)
            mOpenCvCameraView.disableView();

        // BEGIN_INCLUDE(onPause)
        // Stop listening for changes to media routes.
        mMediaRouter.removeCallback(mMediaRouterCallback);
        // END_INCLUDE(onPause)
    }

    @Override
    public void onResume() {
        super.onResume();
        if (!OpenCVLoader.initDebug()) {
            Log.d(TAG, "Internal OpenCV library not found. Using OpenCV Manager for initialization");
            OpenCVLoader.initAsync(OpenCVLoader.OPENCV_VERSION_3_0_0, this, mLoaderCallback);
            // BEGIN_INCLUDE(addCallback)
            // Register a callback for all events related to live video devices
            mMediaRouter.addCallback(MediaRouter.ROUTE_TYPE_LIVE_VIDEO, mMediaRouterCallback);
            // END_INCLUDE(addCallback)

            // Show the 'Not con nected' status message
            //mButton.setEnabled(false);
            //mTextStatus.setText(R.string.secondary_notconnected);

            // Update the displays based on the currently active routes
            updatePresentation();


        } else {
            Log.d(TAG, "OpenCV library found inside package. Using it!");
            mLoaderCallback.onManagerConnected(LoaderCallbackInterface.SUCCESS);
        }
    }

    public void onDestroy() {
        super.onDestroy();
        if (mOpenCvCameraView != null)
            mOpenCvCameraView.disableView();
    }

    public void onCameraViewStarted(int width, int height) {
    }

    public void onCameraViewStopped() {
    }

    public Mat onCameraFrame(CvCameraViewFrame inputFrame) {
        FrameRGBA = inputFrame.rgba();
        FrameG = inputFrame.gray();
        //Size imgSize = new Size(640, 360);
        //Size imgSize2 = new Size(1280, 720);
        boolean mPatternWasFound = false;
        boolean mBoardWasFound = false;
        if(!init_stat) {
            mCamProjCalib.setup(1280, 720, 1280, 720);
            init_stat=true;
        }

        mCamProjCalib.processFrame(FrameG,FrameRGBA);


       // Imgproc.resize(lastFrame, lastFrame,imgSize);

     //   Mat dispImg = new Mat ();
     //   lastFrame.copyTo(dispImg);

//        Size patternSize = new Size(9, 6);
    //    MatOfPoint2f corners = new MatOfPoint2f();
    //    Point vertex = new Point();
//        Calib3d.findChessboardCorners(lastFrame, patternSize, corners, Calib3d.CALIB_CB_ADAPTIVE_THRESH + Calib3d.CALIB_CB_NORMALIZE_IMAGE
//                + Calib3d.CALIB_CB_FAST_CHECK);
//
//        for (int i=0; i<corners.height();i++)
//        {
//            double[] m=corners.get(i, 0);
//            vertex.x=m[0];
//            vertex.y=m[1];
//            Imgproc.circle(lastFrame, vertex,2, new Scalar(255, 0, 0), 2);
//        }
//
//
//
//
//        MatOfPoint2f BoardCorners = new MatOfPoint2f();
//        mBoardWasFound = Calib3d.findCirclesGrid(lastFrame, new Size(4, 11),BoardCorners, Calib3d.CALIB_CB_ASYMMETRIC_GRID);
//        //mBoardWasFound=Calib3d.findChessboardCorners(lastFrame, new Size(9, 6), BoardCorners, Calib3d.CALIB_CB_ADAPTIVE_THRESH + Calib3d.CALIB_CB_NORMALIZE_IMAGE
//        //        + Calib3d.CALIB_CB_FAST_CHECK);
//        if(mBoardWasFound) {
//            Calib3d.drawChessboardCorners(dispFrame, new Size(4, 11), BoardCorners, mBoardWasFound);
//        }
//
//
//        MatOfPoint2f PatternCorners = new MatOfPoint2f();
//        Core.absdiff(lastFrame, new Scalar(255), lastFrame);
//        //Imgproc.threshold(lastFrame, lastFrame, 0, 255, Imgproc.THRESH_BINARY_INV);
//        mPatternWasFound=Calib3d.findCirclesGrid(lastFrame, new Size(6, 8), PatternCorners, Calib3d.CALIB_CB_SYMMETRIC_GRID);
//        if(mPatternWasFound) {
//            Calib3d.drawChessboardCorners(dispFrame, new Size(6, 8), PatternCorners, mPatternWasFound);
//        }

        //salt(lastFrame.getNativeObjAddr(), 2000);
        return mOpenCvCameraView.getFrame(FrameRGBA);
        //return mOpenCvCameraView.getFrame(mCamProjCalib.pProjImage);
    }

    private void takeMeasurement() {
//            SimpleDateFormat sdf = new SimpleDateFormat("yyyy-MM-dd_HH-mm-ss");
//            String currentDateandTime = sdf.format(new Date());
//            String fileName = Environment.getExternalStorageDirectory().getPath() +
//                    "/sample_picture_" + currentDateandTime + ".jpg";
//            mOpenCvCameraView.takePicture(fileName);
//            Toast.makeText(this, fileName + " saved", Toast.LENGTH_SHORT).show();
        mCamProjCalib.doCalibrate();

    }


    @SuppressLint("SimpleDateFormat")
    @Override
    public boolean onTouch(View v, MotionEvent event) {
        //Log.i(TAG, "onTouch event");
        mButton.setEnabled(true);
        updatePresentation();
        if(!mCamProjCalib.addCorners()) {
            Toast.makeText(this, "Adding capture failed!", Toast.LENGTH_SHORT).show();
        }

//        if (mPresentation != null) {
//            mPresentation.setImageDynamic(lastFrame);
//        }

       //Toast.makeText(this, "disable touch view", Toast.LENGTH_SHORT).show();

/*        taFileStorage.create(root+ "/mats");
        int[][] intArray = new int[][]{{2,3,4},{5,6,7},{8,9,10}};
        Mat matObject = new Mat(3,3,CvType.CV_8UC1);
        for(int row=0;row<3;row++){
            for(int col=0;col<3;col++)
                matObject.put(row, col, intArray[row][col]);
        }
        taFileStorage.writeMat("demom", matObject);
        taFileStorage.release();*/

//        SimpleDateFormat sdf = new SimpleDateFormat("yyyy-MM-dd_HH-mm-ss");
//        String currentDateandTime = sdf.format(new Date());
//        String fileName = Environment.getExternalStorageDirectory().getPath() +
//                "/sample_picture_" + currentDateandTime + ".jpg";
//        mOpenCvCameraView.takePicture(fileName);


        //imageView.setImageResource(R.drawable.chessboard);
        //
        //taFileStorage.create(root + "/mats2");
        //taFileStorage.writeMat("dfa", homography);
        //taFileStorage.writeMat("dfaf", homography);
        //taFileStorage.release();


        //Toast.makeText(this, "Don't touch", Toast.LENGTH_SHORT).show();
        return false;
    }

    @Override
    public void onStart() {
        super.onStart();

    }

    @Override
    public void onStop() {
        super.onStop();

        // BEGIN_INCLUDE(onStop)
        // Dismiss the presentation when the activity is not visible.
        if (mPresentation != null) {
            mPresentation.dismiss();
            mPresentation = null;
        }
        // BEGIN_INCLUDE(onStop)

    }

    public native void salt(long matAddrGray, int nbrElem);

}
