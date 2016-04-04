#ifndef OPENCV_NATIVE_ANDROIDSTUDIO_NATIVE_OPENCV_H
#define OPENCV_NATIVE_ANDROIDSTUDIO_NATIVE_OPENCV_H

#include <jni.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <vector>
#include <opencv2/imgproc.hpp>
extern "C" {
JNIEXPORT void JNICALL Java_i2r_snap2inspect_Tutorial3Activity_salt(JNIEnv *env, jobject instance,
                                                                                         jlong matAddrGray,
                                                                                         jint nbrElem);

}
#endif //OPENCV_NATIVE_ANDROIDSTUDIO_NATIVE_OPENCV_H
