package jp.jaxa.iss.kibo.rpc.sampleapk;

import android.util.Log;

import org.opencv.aruco.Aruco;
import org.opencv.aruco.Dictionary;
import org.opencv.calib3d.Calib3d;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfDouble;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.MatOfPoint3f;
import org.opencv.core.Point3;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;

import gov.nasa.arc.astrobee.Kinematics;
import gov.nasa.arc.astrobee.Result;
import gov.nasa.arc.astrobee.types.Point;
import gov.nasa.arc.astrobee.types.Quaternion;
import jp.jaxa.iss.kibo.rpc.api.KiboRpcService;


public class YourService extends KiboRpcService {

    final int LOOP_MAX = 3;
    final String TAG = "BCL";
    Mat cameraMatrix, distCoeffs;
    ObjectDetection model;
    Guide guide;
    Quaternion normQuater;
    Point normPoint;


    @Override
    protected void runPlan1() {
        long startTime = System.currentTimeMillis();
        api.startMission();

        initializeCamera();
        initializeModel();
        initializeGuide();


        // Area 1
        guide.next();

        Mat detected_mat = AR_detect(getMat(), 1);
        DetectionResult detectionResult = model.detect(detected_mat);
        api.setAreaInfo(1, detectionResult.classname, detectionResult.num);

        // Area 2
        guide.next();
        guide.next();

        detected_mat = AR_detect(getMat(), 2);
        detectionResult = model.detect(detected_mat);
        api.setAreaInfo(2, detectionResult.classname, detectionResult.num);

        // Area 3
        guide.next();

        detected_mat = AR_detect(getMat(), 3);
        detectionResult = model.detect(detected_mat);
        api.setAreaInfo(3, detectionResult.classname, detectionResult.num);

        // Area 4
        guide.next();

        detected_mat = AR_detect(getMat(), 4);
        detectionResult = model.detect(detected_mat);
        api.setAreaInfo(4, detectionResult.classname, detectionResult.num);

        api.reportRoundingCompletion();


        // Scientist
        guide.next();
        detected_mat = AR_detect(getMat(), 5);
        detectionResult = model.detect(detected_mat);
        Log.i(TAG, "Scientist wants: " + detectionResult.classname);
        api.notifyRecognitionItem();

        // TODO : Implement shortest path to item.
        api.takeTargetItemSnapshot();
        long deltaTime = System.currentTimeMillis() - startTime;
        Log.i(TAG, "Plan 1 executed in: " + deltaTime / 1000 + "s.");
        api.shutdownFactory();
    }

    @Override
    protected void runPlan2() {
        long startTime = System.currentTimeMillis();
        api.startMission();
        initializeCamera();
        initializeGuide();

        guide.next();
        Mat detected_mat = AR_detect(getMat(), 1);


//        normPoint = api.getRobotKinematics().getPosition();
        normQuater = new Quaternion(0.000f, 0.000f, -0.707f, 0.707f);
        Log.i(TAG, "Moving to: (" + normPoint.getX() + ", " + normPoint.getY() + ", " + normPoint.getZ() + ")");
        Log.i(TAG, "         : (" + normQuater.getX() + ", " + normQuater.getY() + ", " + normQuater.getZ() + ", " + normQuater.getW() + ")");
        Result result;

        int counter = 0;
        int LOOP_MAX = 5;
        do {
            Log.i(TAG, "[" + counter + "] Calling moveTo");
            result = api.moveTo(normPoint, normQuater, false);
            counter += 1;
        } while (!result.hasSucceeded() && counter < LOOP_MAX);



//        while (!guide.empty()) {
//            guide.next();
//            try {
//                Thread.sleep(1000);
//            } catch (InterruptedException e) {
//                e.printStackTrace();
//            }
//        }

        long deltaTime = System.currentTimeMillis() - startTime;
        Log.i(TAG, "Plan 2 executed in: " + deltaTime / 1000 + "s.");
        api.notifyRecognitionItem();
        api.takeTargetItemSnapshot();
        api.shutdownFactory();
    }

    @Override
    protected void runPlan3() {
        long startTime = System.currentTimeMillis();
        api.startMission();
        initializeCamera();
        initializeGuide();
        while (!guide.empty()) {
            guide.next();
        }
        long deltaTime = System.currentTimeMillis() - startTime;
        Log.i(TAG, "Plan 2 executed in: " + deltaTime / 1000 + "s.");
        api.notifyRecognitionItem();
        api.takeTargetItemSnapshot();
        api.shutdownFactory();
    }

    public static void getQuaternion(Mat R, double[] Q) {
        double trace = R.get(0, 0)[0] + R.get(1, 1)[0] + R.get(2, 2)[0];

        if (trace > 0.0) {
            double s = Math.sqrt(trace + 1.0);
            Q[3] = (s * 0.5);
            s = 0.5 / s;
            Q[0] = ((R.get(2, 1)[0] - R.get(1, 2)[0]) * s);
            Q[1] = ((R.get(0, 2)[0] - R.get(2, 0)[0]) * s);
            Q[2] = ((R.get(1, 0)[0] - R.get(0, 1)[0]) * s);
        } else {
            int i = R.get(0, 0)[0] < R.get(1, 1)[0] ? (R.get(1, 1)[0] < R.get(2, 2)[0] ? 2 : 1) : (R.get(0, 0)[0] < R.get(2, 2)[0] ? 2 : 0);
            int j = (i + 1) % 3;
            int k = (i + 2) % 3;

            double s = Math.sqrt(R.get(i, i)[0] - R.get(j, j)[0] - R.get(k, k)[0] + 1.0);
            Q[i] = s * 0.5;
            s = 0.5 / s;

            Q[3] = (R.get(k, j)[0] - R.get(j, k)[0]) * s;
            Q[j] = (R.get(j, i)[0] + R.get(i, j)[0]) * s;
            Q[k] = (R.get(k, i)[0] + R.get(i, k)[0]) * s;
        }
    }

    private void rvec2qua( Mat rvec ) {
        Mat rMat = new Mat();
        Calib3d.Rodrigues(rvec, rMat);
        double[] Q = new double[4];
        getQuaternion(rMat, Q);
        Log.i(TAG, rMat.dump());
        normQuater = new Quaternion((float)Q[0], (float)Q[1], (float)Q[2], (float)Q[3]);
    }

    private Mat AR_detect(Mat image, int area) {
        int contents = 0, count = 0;
        api.saveMatImage(image, "original_" + area + ".png");
        while (contents == 0 && count < LOOP_MAX) {
            Mat source = image.clone();
//            Calib3d.undistort(image, source, cameraMatrix, distCoeffs);
            Mat ids = new Mat();
            Dictionary dictionary = Aruco.getPredefinedDictionary(Aruco.DICT_5X5_250);
            List<Mat> corners = new ArrayList<>();
            try {
                Aruco.detectMarkers(source, dictionary, corners, ids);
                contents = (int) ids.get(0, 0)[0];
                if (corners.size() == 0) {
                    return new Mat();
                }

                Log.i(TAG, "Found AR");

                Mat rvecs = new Mat();
                Mat tvecs = new Mat();
                Aruco.estimatePoseSingleMarkers(corners, 5.0f, cameraMatrix, distCoeffs, rvecs, tvecs);

                if (rvecs.empty() || tvecs.empty()) {
                    Log.d(TAG, "rvecs, tvecs missing");
                    return new Mat();
                }
                Log.i(TAG, "Obtained rvecs, tvecs");

                rvec2qua(rvecs);
                Point curPoint = api.getRobotKinematics().getPosition();
                normPoint = new Point(curPoint.getX() + tvecs.get(0, 0)[0] / 100, -10.15, curPoint.getZ() + tvecs.get(0, 0)[1] / 100);
                Log.i(TAG, "rvec: " + rvecs.dump());
                Log.i(TAG, "tvec: " + tvecs.dump());

                MatOfPoint3f objPts = new MatOfPoint3f(
                        new Point3(-23.25f, 3.75f, 0.0f),
                        new Point3(-3.75f, 3.75f, 0.0f),
                        new Point3(-3.75f, -11.25f, 0.0f),
                        new Point3(-23.25f, -11.25f, 0.0f)
                );
                MatOfPoint2f imgPts = new MatOfPoint2f();
                MatOfDouble distCo = new MatOfDouble(distCoeffs);
                Calib3d.projectPoints(objPts, rvecs, tvecs, cameraMatrix, distCo, imgPts);
                Log.i(TAG, "Project `objPts` to `imgPts` Succeeded");

                List<org.opencv.core.Point> imgPts_list = imgPts.toList();
                org.opencv.core.Point pt_A = imgPts_list.get(0);
                org.opencv.core.Point pt_D = imgPts_list.get(1);
                org.opencv.core.Point pt_C = imgPts_list.get(2);
                org.opencv.core.Point pt_B = imgPts_list.get(3);

                double width_AD = Math.sqrt(Math.pow(pt_A.x - pt_D.x, 2) + Math.pow(pt_A.y - pt_D.y, 2));
                double width_BC = Math.sqrt(Math.pow(pt_B.x - pt_C.x, 2) + Math.pow(pt_B.y - pt_C.y, 2));
                double maxWidth = Math.max(width_AD, width_BC);

                double height_AB = Math.sqrt(Math.pow(pt_A.x - pt_B.x, 2) + Math.pow(pt_A.y - pt_B.y, 2));
                double height_CD = Math.sqrt(Math.pow(pt_D.x - pt_C.x, 2) + Math.pow(pt_D.y - pt_C.y, 2));
                double maxHeight = Math.max(height_AB, height_CD);

                Size dstImgSize = new Size(maxWidth, maxHeight);
                Log.i(TAG, "After warp size : (" + (int) dstImgSize.width + ", " + (int) dstImgSize.height + ")");

                MatOfPoint2f dstPts = new MatOfPoint2f(
                        new org.opencv.core.Point(0, 0),
                        new org.opencv.core.Point(maxWidth, 0),
                        new org.opencv.core.Point(maxWidth, maxHeight),
                        new org.opencv.core.Point(0, maxHeight)
                );

                Mat warpMat = Imgproc.getPerspectiveTransform(imgPts, dstPts);
                Mat dstImage = new Mat();
                Imgproc.warpPerspective(source, dstImage, warpMat, dstImgSize);
                api.saveMatImage(dstImage, "warpPerspective_" + area + ".png");
                Log.i(TAG, "Saved warpPerspective image");
                return dstImage;
            } catch (Exception e) {
                Log.d(TAG, e.toString());
            }
            count++;
        }
        return new Mat();
    }

    private void initializeModel() {
        model = new ObjectDetection(YourService.this, "model.tflite");
    }

    private void initializeCamera() {
        cameraMatrix = new Mat(3, 3, CvType.CV_64F);
        distCoeffs = new Mat(1, 5, CvType.CV_64F);

        double[][] navCamIntrinsics = api.getNavCamIntrinsics();
        cameraMatrix.put(0, 0, navCamIntrinsics[0]);
        distCoeffs.put(0, 0, navCamIntrinsics[1]);
        distCoeffs.convertTo(distCoeffs, CvType.CV_64F);

        Log.i(TAG, "Initialized camera");
    }

    private void initializeGuide() {
        guide = new Guide(api, false);
    }

    private Mat getMat() {
        api.flashlightControlFront(0.05f);
        Mat image = api.getMatNavCam();
        api.flashlightControlFront(0.00f);
        return image;
    }
}
