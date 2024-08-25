package jp.jaxa.iss.kibo.rpc.sampleapk;

import android.content.Context;
import android.util.Log;

import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.tensorflow.lite.DataType;
import org.tensorflow.lite.Interpreter;
import org.tensorflow.lite.Interpreter.Options;
import org.tensorflow.lite.support.common.FileUtil;
import org.tensorflow.lite.support.tensorbuffer.TensorBuffer;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Comparator;
import java.util.Iterator;
import java.util.List;

public class ObjectDetection {
    private static final int numChannel = 14;
    private static final int numElements = 8400;
    private static final float CONFIDENCE_THRESHOLD = 0.8F;
    private static final float IOU_THRESHOLD = 0.5F;
    private final String TAG = "BCL";
    private Interpreter interpreter = null;
    private List<String> labels;

    ObjectDetection(Context context, String modelPath) {
        long startTime = System.currentTimeMillis();
        try {
            ByteBuffer model = FileUtil.loadMappedFile(context, modelPath);
            Options options = new Interpreter.Options().setNumThreads(4);
            interpreter = new Interpreter(model, options);
        } catch (Exception e) {
            Log.d(TAG, e.toString());
        }
        labels = new ArrayList<>(Arrays.asList("beaker", "goggle", "hammer", "kapton_tape", "pipette", "screwdriver", "thermometer", "top", "watch", "wrench"));
        Log.i(TAG, "Initialized TFLite model [" + (System.currentTimeMillis() - startTime) + " ms]");
    }

    private static Mat LetterBox(Mat img, int target) {
        int width = img.cols();
        int height = img.rows();

        Mat square = new Mat(target, target, img.type(), new Scalar(114, 114, 114));

        int maxDim = Math.max(width, height);
        float scale = (float) target / maxDim;
        Rect roi = new Rect();

        if (width >= height) {
            roi.width = target;
            roi.x = 0;
            roi.height = Math.round(height * scale);
            roi.y = (target - roi.height) / 2;
        } else {
            roi.y = 0;
            roi.height = target;
            roi.width = Math.round(width * scale);
            roi.x = (target - roi.width) / 2;
        }

        Mat resizedImg = new Mat();
        Imgproc.resize(img, resizedImg, new Size(roi.width, roi.height), Imgproc.INTER_AREA);
        resizedImg.copyTo(square.submat(roi));

        return square;
    }

    private static List<BoundingBox> applyNMS(List<BoundingBox> boxes) {
        List<BoundingBox> sortedBoxes = new ArrayList<>(boxes);
        sortedBoxes.sort(Comparator.comparingDouble((BoundingBox b) -> b.cnf).reversed());
        List<BoundingBox> selectedBoxes = new ArrayList<>();

        while (!sortedBoxes.isEmpty()) {
            BoundingBox first = sortedBoxes.get(0);
            selectedBoxes.add(first);
            sortedBoxes.remove(0);

            Iterator<BoundingBox> iterator = sortedBoxes.iterator();
            while (iterator.hasNext()) {
                BoundingBox nextBox = iterator.next();
                float iou = calculateIoU(first, nextBox);
                if (iou >= IOU_THRESHOLD) {
                    iterator.remove();
                }
            }
        }
        return selectedBoxes;
    }

    private static float calculateIoU(BoundingBox box1, BoundingBox box2) {
        float x1 = Math.max(box1.x1, box2.x1);
        float y1 = Math.max(box1.y1, box2.y1);
        float x2 = Math.min(box1.x2, box2.x2);
        float y2 = Math.min(box1.y2, box2.y2);
        float intersectionArea = Math.max(0F, x2 - x1) * Math.max(0F, y2 - y1);
        float box1Area = box1.w * box1.h;
        float box2Area = box2.w * box2.h;
        return intersectionArea / (box1Area + box2Area - intersectionArea);
    }

    DetectionResult detect(Mat mat) {
        long startTime = System.currentTimeMillis();
        Log.i(TAG, "----- START -----");


        Mat inputMat = preProcess(mat);
        ByteBuffer imageBuffer = Mat2ByteBuffer(inputMat);

        if (imageBuffer == null) {
            Log.d(TAG, "`imageBuffer` cannot be null");
            return new DetectionResult();
        }

        TensorBuffer output = TensorBuffer.createFixedSize(new int[]{1, numChannel, numElements}, DataType.FLOAT32);
        try {
            interpreter.run(imageBuffer, output.getBuffer());
        } catch (Exception e) {
            Log.d(TAG, e.toString());
        }
        DetectionResult detectionResult = postProcess(output);

        Log.i(TAG, "TIME : " + (System.currentTimeMillis() - startTime) + " ms");
        Log.i(TAG, "-----  END  -----");
        return detectionResult;
    }

    private DetectionResult postProcess(TensorBuffer output) {
        List<BoundingBox> bestBoxes = bestBox(output.getFloatArray());

        if (bestBoxes == null) {
            Log.d(TAG, "No detection");
            return null;
        }

        DetectionResult detectionResult = new DetectionResult();

        int[] detections = new int[10];
        Arrays.fill(detections, 0);
        int n_classDetected = 0;
        for (BoundingBox box : bestBoxes) {
            if (detections[box.cls]++ == 0) {
                n_classDetected += 1;
            }
        }

        if (n_classDetected == 1) {
            detectionResult.setClassname(bestBoxes.get(0).clsName);
            detectionResult.setNum(detections[bestBoxes.get(0).cls]);
            Log.i(TAG, "Found : " + detections[bestBoxes.get(0).cls] + " " + bestBoxes.get(0).clsName);
            return detectionResult;
        }

        Log.i(TAG, n_classDetected + " class detected");
        int mx = -1;
        int mxClass = -1;
        for (int i = 0; i < 10; i++) {
            if (detections[i] != 0) {
                Log.i(TAG, detections[i] + " " + labels.get(i));
            }
            if (detections[i] > mx) {
                mx = detections[i];
                mxClass = i;
            }
        }

        if (mx > bestBoxes.size() / 2) {
            detectionResult.setClassname(labels.get(mxClass));
            detectionResult.setNum(bestBoxes.size());
            detectionResult.setAssume();
            Log.i(TAG, "Assumed : " + bestBoxes.size() + " " + labels.get(mxClass));
            return detectionResult;
        }
        Log.i(TAG, "Rejected : Try different position");
        return detectionResult;
    }

    private Mat preProcess(Mat mat) {
        long startTime = System.currentTimeMillis();
        Mat retMat = new Mat();
        Imgproc.cvtColor(mat, retMat, Imgproc.COLOR_BGR2RGB);
        Imgproc.GaussianBlur(retMat, retMat, new Size(3, 3), 0, 0);
        retMat = LetterBox(retMat, 480);
        retMat = LetterBox(retMat, 640);
        retMat.convertTo(retMat, CvType.CV_32F, 1.0 / 255.0);
        Log.i(TAG, "Pre-processed [" + (System.currentTimeMillis() - startTime) + " ms]");
        return retMat;
    }

    private ByteBuffer Mat2ByteBuffer(Mat mat) {
        long startTime = System.currentTimeMillis();
        ByteBuffer imageBuffer = ByteBuffer.allocateDirect(640 * 640 * 3 * Float.BYTES);
        try {
            imageBuffer.order(ByteOrder.nativeOrder());
            imageBuffer.rewind();
            float[] matFloat = new float[(int) mat.total() * mat.channels()];
            mat.get(0, 0, matFloat);
            imageBuffer.asFloatBuffer().put(matFloat);
        } catch (Exception e) {
            Log.d(TAG, e.toString());
        }
        Log.i(TAG, "Mat -> ByteBuffer [" + (System.currentTimeMillis() - startTime) + " ms]");
        return imageBuffer;
    }

    private List<BoundingBox> bestBox(float[] array) {
        long startTime = System.currentTimeMillis();
        List<BoundingBox> boundingBoxes = new ArrayList<>();
        for (int c = 0; c < numElements; c++) {
            float maxConf = -1.0f;
            int maxIdx = -1;
            int j = 4;
            int arrayIdx = c + numElements * j;
            while (j < numChannel) {
                if (array[arrayIdx] > maxConf) {
                    maxConf = array[arrayIdx];
                    maxIdx = j - 4;
                }
                j++;
                arrayIdx += numElements;
            }

            if (maxConf > CONFIDENCE_THRESHOLD) {
                String clsName = labels.get(maxIdx);
                float cx = array[c]; // 0
                float cy = array[c + numElements]; // 1
                float w = array[c + numElements * 2];
                float h = array[c + numElements * 3];
                float x1 = cx - (w / 2F);
                float y1 = cy - (h / 2F);
                float x2 = cx + (w / 2F);
                float y2 = cy + (h / 2F);
                if (x1 < 0F || x1 > 1F) continue;
                if (y1 < 0F || y1 > 1F) continue;
                if (x2 < 0F || x2 > 1F) continue;
                if (y2 < 0F || y2 > 1F) continue;

                boundingBoxes.add(
                        new BoundingBox(
                                x1, y1, x2, y2,
                                cx, cy, w, h,
                                maxConf, maxIdx, clsName
                        )
                );
            }
        }
        Log.i(TAG, "NMS Applied [" + (System.currentTimeMillis() - startTime) + " ms]");
        if (boundingBoxes.isEmpty()) return null;
        return applyNMS(boundingBoxes);
    }


}
