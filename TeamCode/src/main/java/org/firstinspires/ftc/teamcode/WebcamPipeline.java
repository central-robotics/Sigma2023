package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.auto.util.OpModeHolder;
import org.opencv.core.Mat;
import org.openftc.easyopencv.OpenCvPipeline;

public class WebcamPipeline extends OpenCvPipeline {

    private static Mat lastMat;

    @Override
    public Mat processFrame(Mat input) {
        if (lastMat == null) {
            OpModeHolder.opMode.telemetry.addLine("Webcam ready");
            OpModeHolder.opMode.telemetry.update();
        }
        lastMat = input;
        return input;
    }

    public static Mat getLastMat() {
        return lastMat;
    }
}
