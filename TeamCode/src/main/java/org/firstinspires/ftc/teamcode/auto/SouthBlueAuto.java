package org.firstinspires.ftc.teamcode.auto;

import com.chsrobotics.ftccore.geometry.Position;
import com.chsrobotics.ftccore.hardware.HardwareManager;
import com.chsrobotics.ftccore.hardware.config.Config;
import com.chsrobotics.ftccore.hardware.config.accessory.Accessory;
import com.chsrobotics.ftccore.hardware.config.accessory.AccessoryType;
import com.chsrobotics.ftccore.pipeline.Pipeline;
import com.chsrobotics.ftccore.vision.CVUtility;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

import org.firstinspires.ftc.teamcode.auto.actions.UpdateDashboardAction;
import org.firstinspires.ftc.teamcode.auto.util.OpModeHolder;
import org.firstinspires.ftc.teamcode.auto.util.SignalSleeveDetector;
import org.firstinspires.ftc.teamcode.auto.actions.ArmPositionAction;
import org.firstinspires.ftc.teamcode.auto.actions.DelayAction;
import org.firstinspires.ftc.teamcode.auto.actions.FullStopAction;
import org.firstinspires.ftc.teamcode.auto.actions.SetArmAction;
import org.firstinspires.ftc.teamcode.auto.actions.ToggleClawAction;
import org.firstinspires.ftc.teamcode.auto.actions.WaitAction;
import org.firstinspires.ftc.teamcode.auto.util.WebcamPipeline;

@Autonomous(name = "South Blue Auto")
public class SouthBlueAuto extends LinearOpMode
{
    @Override
    public void runOpMode() throws InterruptedException {
        OpModeHolder.opMode = this;
        WebcamPipeline.clearLastMat();
        Config config = new Config.Builder()
                .setDebugMode(true)
                .setDriveMotors("m0", "m1", "m2", "m3")
                .setMotorDirection(DcMotorSimple.Direction.FORWARD)
//                .addAccessory(new Accessory(AccessoryType.MOTOR, "l0"))
//                .addAccessory(new Accessory(AccessoryType.MOTOR, "l1"))
                .addAccessory(new Accessory(AccessoryType.SERVO, "c0"))
                .addAccessory(new Accessory(AccessoryType.SERVO, "c1"))
                .addAccessory(new Accessory(AccessoryType.WEBCAM, "webcam"))
                .addAccessory(new Accessory(AccessoryType.ODOMETRY_POD, "odo0"))
                .addAccessory(new Accessory(AccessoryType.ODOMETRY_POD, "odo1"))
                .setOpMode(this)
                .setIMU("imu")
                .setPIDCoefficients(new PIDCoefficients(2.6, 0.0005, 0), new PIDCoefficients(700, 0.03, 0))
                .build();

        HardwareManager manager = new HardwareManager(config, hardwareMap);

        manager.accessoryOdometryPods[0].setDirection(DcMotorSimple.Direction.REVERSE);
        manager.accessoryOdometryPods[1].setDirection(DcMotorSimple.Direction.REVERSE);

        CVUtility cv = null;
        try {
            cv = new CVUtility(manager, telemetry);
        } catch (Exception e) {
            telemetry.addLine("CVUtility failed to initialized");
            telemetry.update();
        }

        ArmPositionAction armPositionAction = new ArmPositionAction(manager);
        ToggleClawAction toggleClawAction = new ToggleClawAction(manager);
        UpdateDashboardAction updateDashboardAction = new UpdateDashboardAction(manager, null);

        waitForStart();

        int dots = 1;
        if (cv != null && cv.initialized && cv.grabFrame() != null) {
            dots = SignalSleeveDetector.detectOrientation(cv.grabFrame());

            telemetry.addData("Dots: ", dots);
            cv.stopStreaming();
        } else {
            telemetry.addLine("Signal sleeve detection failed");
        }
        double parkingPos = dots == 1 ? -600 :
                (dots == 2 ? 0 : 700);
        telemetry.update();
        Pipeline pipeline = new Pipeline.Builder(manager)
                .addContinuousAction(updateDashboardAction)
//                .addContinuousAction(armPositionAction)
//                .addAction(toggleClawAction)
//                .addAction(new DelayAction(manager, 1000))
//                .addAction(new SetArmAction(manager, 1700))
                .addLinearPath(
//                        new Position(0, 80, 0),
                        new Position(680, 0, 0),
                        new Position(680, 1010, 0),
                        new Position(680, 1010, Math.PI / 2)
                )
//                .addAction(new FullStopAction(manager))
//                .addAction(toggleClawAction)
//                .addAction(new DelayAction(manager, 300))
//                .addAction(new SetArmAction(manager, 800))
//                .addAction(new DelayAction(manager, 200))
                .addLinearPath(
                        new Position(680, 1010, 7 * Math.PI / 4),
                        new Position(710, 1190, 7 * Math.PI / 4)
                )
//                .addAction(new FullStopAction(manager))
//                .addAction(toggleClawAction)
//                .addAction(new DelayAction(manager, 300))
//                .addAction(new SetArmAction(manager, 4000))
//                .addAction(new DelayAction(manager, 300))
                .addLinearPath(
                        new Position(300, 1400, 7 * Math.PI / 4),
                        new Position(0, 1350, 7 * Math.PI / 4),
                        new Position(0, 1350, Math.PI / 4),
                        new Position(-80, 1560, Math.PI / 4)
                )
//                .addAction(new FullStopAction(manager))
//                .addAction(toggleClawAction)
//                .addAction(new DelayAction(manager, 300))
//                .addAction(new SetArmAction(manager, 0))
                .addLinearPath(
                        new Position(0, 1350, Math.PI / 4),
                        new Position(0, 1350, 0),
                        new Position(parkingPos, 1350, 0)
                )
//                .addAction(new FullStopAction(manager))
//                .addAction(new WaitAction(manager, armPositionAction))
                .build();
        updateDashboardAction.localization = pipeline.localization;
        pipeline.execute();
    }
}
