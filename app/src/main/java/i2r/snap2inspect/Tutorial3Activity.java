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
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;

import android.annotation.SuppressLint;
import android.app.Activity;
import android.app.Fragment;
import android.content.Context;
import android.content.pm.PackageManager;
import android.hardware.camera2.CameraDevice;
import android.hardware.camera2.CameraManager;
import android.hardware.camera2.CameraMetadata;
import android.hardware.camera2.CaptureRequest;
import android.media.MediaRouter;
import android.os.Bundle;
import android.os.Environment;
import android.support.v4.app.ActivityCompat;
import android.support.v4.content.ContextCompat;
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
    private boolean calib_stat=false;
    private boolean measure_stat=false;
    private static final int REQUEST_WRITE_STORAGE = 112;

    @Override
    public void onCheckedChanged(CompoundButton buttonView, boolean isChecked) {
        if (mPresentation != null) {
            Toast.makeText(this, "Monitored switch is " + (isChecked ? "on" : "off"),
                    Toast.LENGTH_SHORT).show();
//            if (isChecked) {
//                mPresentation.setImageDynamic(mOpenCvCameraView.DispImg);
//            } else {
//                mPresentation.setImageDynamic(mCamProjCalib.pProjImage);
//            }
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

        if(!mCamProjCalib.isCalibrated()) {
            if (!mCamProjCalib.isInitial()) {
                mCamProjCalib.setup(1280, 720, 1280, 720);
            }

            mCamProjCalib.processFrame(FrameG, FrameRGBA);

        }
        else
        {
            Log.i(TAG, "start to measureFrmae");
            mOpenCvCameraView.measureFrame(FrameG, FrameRGBA);
            Log.i(TAG, "measureFrmae Complete");

            if(measure_stat) {
                measure_stat=false;
                Log.i(TAG, "save measurement start");
                Log.i(TAG, "set flags to disable save measurement");
                SimpleDateFormat sdf = new SimpleDateFormat("yyyy-MM-dd_HH-mm-ss");
                String currentDateandTime = sdf.format(new Date());
                String fileName = Environment.getExternalStorageDirectory().getPath() + "/S2I_Data/measurement_" + currentDateandTime + ".jpg";
                Imgcodecs.imwrite(fileName, FrameRGBA);
                Log.i(TAG, "save measurement complete");

            }
        }

        return mOpenCvCameraView.getFrame(FrameRGBA);
        //salt(lastFrame.getNativeObjAddr(), 2000);
        //return mOpenCvCameraView.getFrame(mCamProjCalib.pProjImage);
    }

    private void takeMeasurement() {
        if(mCamProjCalib.isCalibrated())
        {
            //mCamProjCalib.resetCalibrated();
            //mCamProjCalib.setup(1280, 720, 1280, 720);
        }
        else {
            if (mCamProjCalib.getCornersBufferSize() > 2) {
                Log.i(TAG, "call doCalibration");
                mCamProjCalib.doCalibrate();
                Log.i(TAG, "call clearCorners");
                mCamProjCalib.clearCorners();
                Log.i(TAG, "call setCalibrated");
                mCamProjCalib.setCalibrated();
                Toast.makeText(this, "Camera projector calibration complete!", Toast.LENGTH_SHORT).show();
                mButton.setEnabled(false);
                Log.i(TAG, "call mOpenCvCameraView.setup()");
                mOpenCvCameraView.setup();
                if (mPresentation != null) {
                    Log.i(TAG, "call  mPresentation.setImageDynamic(mOpenCvCameraView.DispImg)");
                    mPresentation.setImageDynamic(mOpenCvCameraView.DispImg);
                }
                Log.i(TAG, "do 6calibration and projection pattern setup completes");

            }else {
                if (mCamProjCalib.getCornersBufferSize() == 0)
                {
                    Toast.makeText(this, "Load calibration parameters!", Toast.LENGTH_SHORT).show();
                    Log.i(TAG, "Load calibration parameters");
                    mCamProjCalib.setCalibrated();
                    mButton.setEnabled(false);
                    Log.i(TAG, "call mOpenCvCameraView.setup()");
                    mOpenCvCameraView.setup();
                    if (mPresentation != null) {
                        Log.i(TAG, "call  mPresentation.setImageDynamic(mOpenCvCameraView.DispImg)");
                        mPresentation.setImageDynamic(mOpenCvCameraView.DispImg);
                    }
                    Log.i(TAG, "load calibration and projection pattern setup completes");
                }
                else {
                    Toast.makeText(this, "At least 3 captures is needed for calibration", Toast.LENGTH_SHORT).show();
                }
            }

        }
    }


    @Override
    public void onRequestPermissionsResult(int requestCode, String[] permissions, int[] grantResults) {
        super.onRequestPermissionsResult(requestCode, permissions, grantResults);
        switch (requestCode)
        {
            case REQUEST_WRITE_STORAGE: {
                if (grantResults.length > 0 && grantResults[0] == PackageManager.PERMISSION_GRANTED)
                {
                    Toast.makeText(this, "WRITE_STORAGE_PERMISSION_GRANTED", Toast.LENGTH_LONG).show();
                    //reload my activity with permission granted or use the features what required the permission
                } else
                {
                    Toast.makeText(this, "The app was not allowed to write to your storage. Hence, it cannot function properly. Please consider granting it this permission", Toast.LENGTH_LONG).show();
                }
            }
        }

    }

    @SuppressLint("SimpleDateFormat")
    @Override
    public boolean onTouch(View v, MotionEvent event) {
        Log.i(TAG, "onTouch event");
        boolean hasPermission = (ContextCompat.checkSelfPermission(this,
                Manifest.permission.WRITE_EXTERNAL_STORAGE) == PackageManager.PERMISSION_GRANTED);
        if (!hasPermission) {
            ActivityCompat.requestPermissions(this,
                    new String[]{Manifest.permission.WRITE_EXTERNAL_STORAGE},
                    REQUEST_WRITE_STORAGE);
        }

        if(!mCamProjCalib.isCalibrated())
        {
            mButton.setEnabled(true);
            updatePresentation();
            if (mPresentation != null){
                mPresentation.setImageDynamic(mCamProjCalib.pProjImage);
            }

            if(!mCamProjCalib.addCorners()) {
                Toast.makeText(this, "Adding capture failed!", Toast.LENGTH_SHORT).show();
            }
        }
        else
        {
            Log.i(TAG, "set flags to save measurement");
            measure_stat=true;
            Toast.makeText(this, "Measurement saved!", Toast.LENGTH_SHORT).show();
        }
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

   // public native void salt(long matAddrGray, int nbrElem);

}
