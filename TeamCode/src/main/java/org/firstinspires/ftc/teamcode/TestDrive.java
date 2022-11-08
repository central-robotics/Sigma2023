package org.firstinspires.ftc.teamcode;


import com.chsrobotics.ftccore.actions.integratedactions.SampleIMUAction;
import com.chsrobotics.ftccore.hardware.HardwareManager;
import com.chsrobotics.ftccore.hardware.config.Config;
import com.chsrobotics.ftccore.teleop.Drive;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class TestDrive extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Config config = new Config.Builder()
                .setDriveMotors("m0", "m1", "m2", "m3")
                .setIMU("imu")
                .setTeleopValues(1, 1)
//                .addAccessory(new Accessory(AccessoryType.WEBCAM, "webcam"))
                .setOpMode(this)
                .build();

        HardwareManager manager = new HardwareManager(config, hardwareMap);

        Drive drive = new Drive.Builder(manager)
                .bindActionToButton(Drive.Builder.GamepadButtons.A, new SampleIMUAction(manager), 1)
                .Build();

        waitForStart();
        drive.runDriveLoop();
    }
}
