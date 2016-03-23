/*
 * Copyright 2013 The Android Open Source Project
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *       http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

package i2r.snap2inspect;

import android.app.Presentation;
import android.content.Context;
import android.graphics.Bitmap;
import android.os.Bundle;
import android.view.Display;
import android.widget.ImageView;
import android.widget.LinearLayout;
import android.widget.TextView;

import org.opencv.android.Utils;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfKeyPoint;
import org.opencv.features2d.FeatureDetector;
import org.opencv.imgproc.Imgproc;

/**
 * <p>
 * A {@link android.app.Presentation} used to demonstrate interaction between primary and
 * secondary screens.
 * </p>
 * <p>
 * It displays the name of the display in which it has been embedded (see
 * {@link android.app.Presentation#getDisplay()}) and exposes a facility to change its
 * background color and display its text.
 * </p>
 */
public class SamplePresentation extends Presentation {

    private LinearLayout mLayout;
    private ImageView mImageView;
    private Mat mat_cb_disp; // mat of chessboard
    private Mat mat_4pt_disp; // mat of markers with 4 points
    private boolean init;
    private int icols=1280;
    private int irows=720;
    public SamplePresentation(Context outerContext, Display display) {
        super(outerContext, display);
    }

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);

        // Set the content view to the custom layout
        setContentView(R.layout.display);

        // Get the Views
        mLayout = (LinearLayout) findViewById(R.id.display_layout);
        mImageView = (ImageView) findViewById(R.id.imageView1);
        init = true;


    }


    public void setImage1() {
        //mLayout.setBackgroundColor(0);
        //mText.setText("Marker Image Set");
        mImageView.setImageResource(R.drawable.marker4);
    }

    public void setImage2() {

        if(init) {
            Mat mat_cb = new Mat(irows, icols, CvType.CV_8UC1);
            for (int ic = 0; ic < icols; ic++) {
                for (int ir = 0; ir < irows; ir++) {
                    if ((ic % 80) == 0 && (ir % 80) == 0) {
                        mat_cb.put(ir, ic, 255);
                    } else {
                        mat_cb.put(ir, ic, 0);
                    }
                }
            }
            mat_cb_disp = new Mat(irows, icols, CvType.CV_8UC3);
            mLayout.setBackgroundColor(0);
            //mText.setText("Marker Image Set");
            Imgproc.cvtColor(mat_cb, mat_cb_disp, Imgproc.COLOR_GRAY2RGB);
            init=false;
        }
        this.setImageDynamic(mat_cb_disp);
        //mImageView.setImageResource(R.drawable.chessboard);
    }

    public void setImageDynamic(Mat m) {
        //mLayout.setBackgroundColor(0);
        //mText.setText("Marker Image Set");
        // convert to bitmap:
        Bitmap bm = Bitmap.createBitmap(m.cols(), m.rows(),Bitmap.Config.ARGB_8888);
        Utils.matToBitmap(m, bm);
        mImageView.setImageBitmap(bm);
    }

}
