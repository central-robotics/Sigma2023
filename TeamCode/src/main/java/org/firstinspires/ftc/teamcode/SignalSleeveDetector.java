package org.firstinspires.ftc.teamcode;

import com.chsrobotics.ftccore.hardware.HardwareManager;

import org.opencv.core.Mat;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

public class SignalSleeveDetector {

    private HardwareManager manager;

    public SignalSleeveDetector(HardwareManager manager) {
        this.manager = manager;
    }

    public int detectOrientation() {
        OpenCvCamera cvCamera = OpenCvCameraFactory.getInstance().createWebcam(manager.getWebcam());
        WebcamPipeline cameraPipeline = new WebcamPipeline();
        cvCamera.setPipeline(cameraPipeline);
        cvCamera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                cvCamera.startStreaming(1280, 720, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                TestAuton.telem.addData("OpenCV failed with code", errorCode);
                TestAuton.telem.update();
            }
        });

        while (WebcamPipeline.getLastMat() == null && !TestAuton.opMode.isStopRequested()) {

        }

        // Do color detection
        Mat mat = WebcamPipeline.getLastMat();


        return 0; // Return number of dots
    }

}
