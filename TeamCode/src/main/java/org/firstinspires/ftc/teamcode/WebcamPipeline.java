package org.firstinspires.ftc.teamcode;

import org.opencv.core.Mat;
import org.openftc.easyopencv.OpenCvPipeline;

public class WebcamPipeline extends OpenCvPipeline {

    private static Mat lastMat;

    @Override
    public Mat processFrame(Mat input) {
        if (lastMat == null) {
            TestAuton.telem.addLine("Webcam ready");
            TestAuton.telem.update();
        }
        lastMat = input;
        return input;
    }

    public static Mat getLastMat() {
        return lastMat;
    }
}
