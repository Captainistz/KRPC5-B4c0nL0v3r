package jp.jaxa.iss.kibo.rpc.sampleapk;

import android.content.pm.LauncherApps;
import android.util.Log;

import java.util.ArrayList;
import java.util.List;

import gov.nasa.arc.astrobee.Result;
import gov.nasa.arc.astrobee.types.Point;
import gov.nasa.arc.astrobee.types.Quaternion;
import jp.jaxa.iss.kibo.rpc.api.KiboRpcApi;

class MoveSet {
    Point point;
    Quaternion quaternion;


    MoveSet(double x, double y, double z, float qx, float qy, float qz, float qw) {
        this.point = new Point(x, y, z);
        this.quaternion = new Quaternion(qx, qy, qz, qw);
    }

    MoveSet(Point point, Quaternion quaternion) {
        this.point = point;
        this.quaternion = quaternion;
    }
}

class Guide {
    private final String TAG = "BCL";
    private KiboRpcApi api;
    private List<MoveSet> moveSets = new ArrayList<>();

    Guide(KiboRpcApi api, boolean forward) {
        long startTime = System.currentTimeMillis();
        this.api = api;

        if (forward) {
            // 1
            Point point = new Point(10.925d, -10.000d, 5.1950d);
            Quaternion quaternion = new Quaternion(0.000f, 0.000f, -0.707f, 0.707f);
            moveSets.add(new MoveSet(point, quaternion));

            // 2
            point = new Point(10.925d, -9.0000d, 4.9700d);
            quaternion = new Quaternion(0.5f, 0.5f, -0.5f, 0.5f);
            moveSets.add(new MoveSet(point, quaternion));

            point = new Point(10.925d, -9.0000d, 4.5000d);
            moveSets.add(new MoveSet(point, quaternion));

            // 3
            point = new Point(10.925d, -8.0000d, 4.5000d);
            moveSets.add(new MoveSet(point, quaternion));

            // 4
            point = new Point(10.600d, -6.8800d, 4.9450d);
            quaternion = new Quaternion(0, -0.707f, 0.707f,0);
            moveSets.add(new MoveSet(point, quaternion));

            // SCIENTIST
            point = new Point(11.143d, -6.8000d, 4.9654d);
            quaternion = new Quaternion(0, 0, 0.707f, 0.707f);
            moveSets.add(new MoveSet(point, quaternion));
        } else {
            // 1
            Point point = new Point(11.45d, -10.000d, 5.5d);
            Quaternion quaternion = new Quaternion(0.000f, 0.000f, -0.707f, 0.707f);
            moveSets.add(new MoveSet(point, quaternion));

            // 2
            point = new Point(11.45d, -8.4000d, 4.28d);
            moveSets.add(new MoveSet(point, quaternion));


            // 4

            point = new Point(10.400d, -7.300d, 4.28d);
            moveSets.add(new MoveSet(point, quaternion));


            // SCIENTIST
            point = new Point(11.143d, -6.8000d, 4.28d);
            moveSets.add(new MoveSet(point, quaternion));

        }

        Log.i(TAG, "Initialized Guide [" + (System.currentTimeMillis() - startTime) / 1000.0 + " s]");
    }

    boolean next() {
        long startTime = System.currentTimeMillis();
        boolean isSucceeded = moveTo(moveSets.get(0).point, moveSets.get(0).quaternion);
        if (isSucceeded) {
            Log.i(TAG, "Move succeeded [" + (System.currentTimeMillis() - startTime) + " ms]");
        } else {
            Log.d(TAG, "Try KOZ maybe? [" + (System.currentTimeMillis() - startTime) + " ms]");
        }
        moveSets.remove(0);
        return isSucceeded;
    }

    boolean empty() {
        return moveSets.isEmpty();
    }

    private boolean moveTo(Point point, Quaternion quaternion) {
        Log.i(TAG, "Moving to: (" + point.getX() + ", " + point.getY() + ", " + point.getZ() + ")");
        Log.i(TAG, "         : (" + quaternion.getX() + ", " + quaternion.getY() + ", " + quaternion.getZ() + ", " + quaternion.getW() + ")");
        Result result;
        int counter = 0;
        int LOOP_MAX = 5;
        do {
            Log.i(TAG, "[" + counter + "] Calling moveTo");
            result = api.moveTo(point, quaternion, false);
            counter += 1;
        } while (!result.hasSucceeded() && counter < LOOP_MAX);
        return result.hasSucceeded();
    }
}
