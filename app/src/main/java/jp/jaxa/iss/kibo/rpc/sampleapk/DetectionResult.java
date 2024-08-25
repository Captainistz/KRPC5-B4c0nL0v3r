package jp.jaxa.iss.kibo.rpc.sampleapk;

public class DetectionResult {

    private final String TAG = "BCL";

    public String classname = "";
    public int num = 0;
    private boolean isAssume = false;

    void setAssume() {
        this.isAssume = true;
    }

    void setClassname(String classname) {
        this.classname = classname;
    }

    void setNum(int num) {
        this.num = num;
    }

    @Override
    public String toString() {
        return num + " " + classname;
    }
}
