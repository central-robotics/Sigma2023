package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.auto.AutonCore;
import org.opencv.core.Mat;
import org.openftc.easyopencv.OpenCvPipeline;

public class WebcamPipeline extends OpenCvPipeline {

    private static Mat lastMat;

    @Override
    public Mat processFrame(Mat input) {
        if (lastMat == null) {
            AutonCore.opMode.telemetry.addLine("Webcam ready");
            AutonCore.opMode.telemetry.update();
        }
        lastMat = input;
        return input;
    }

    public static Mat getLastMat() {
        return lastMat;
    }
}
