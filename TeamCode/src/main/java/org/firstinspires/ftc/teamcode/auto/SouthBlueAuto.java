package org.firstinspires.ftc.teamcode.auto;

import com.chsrobotics.ftccore.geometry.Position;
import com.chsrobotics.ftccore.hardware.HardwareManager;
import com.chsrobotics.ftccore.hardware.config.Config;
import com.chsrobotics.ftccore.hardware.config.accessory.Accessory;
import com.chsrobotics.ftccore.hardware.config.accessory.AccessoryType;
import com.chsrobotics.ftccore.pipeline.Pipeline;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

import org.firstinspires.ftc.teamcode.auto.actions.ArmPositionAction;
import org.firstinspires.ftc.teamcode.auto.actions.DelayAction;
import org.firstinspires.ftc.teamcode.auto.actions.FullStopAction;
import org.firstinspires.ftc.teamcode.auto.actions.SetArmAction;
import org.firstinspires.ftc.teamcode.auto.actions.ToggleClawAction;
import org.firstinspires.ftc.teamcode.auto.actions.WaitAction;
import org.firstinspires.ftc.teamcode.auto.util.OpModeHolder;
import org.firstinspires.ftc.teamcode.auto.util.SignalSleeveDetector;
import org.firstinspires.ftc.teamcode.auto.util.WebcamPipeline;

@Autonomous(name="South Blue Auto")
public class SouthBlueAuto extends LinearOpMode
{
    @Override
    public void runOpMode()
    {
        OpModeHolder.opMode = this;
        WebcamPipeline.clearLastMat();
        Config config = new Config.Builder()
                .setDebugMode(false)
                .setDriveMotors("m0", "m1", "m2", "m3")
                .addAccessory(new Accessory(AccessoryType.MOTOR, "l0"))
                .addAccessory(new Accessory(AccessoryType.MOTOR, "c0"))
                .addAccessory(new Accessory(AccessoryType.WEBCAM, "webcam"))
                .setOpMode(this)
                .setIMU("imu")
                .setPIDCoefficients(new PIDCoefficients(1.3, 0.002, 0), new PIDCoefficients(550, 0.7, 0))
                .build();

        HardwareManager manager = new HardwareManager(config, hardwareMap);

        ArmPositionAction armPositionAction = new ArmPositionAction(manager);
        ToggleClawAction toggleClawAction = new ToggleClawAction(manager);
        SignalSleeveDetector detector = new SignalSleeveDetector(manager);

        waitForStart();

        int dots = detector.detectOrientation();
        double parkingPos = dots == 1 ? -600 :
                (dots == 2 ? 0 :
                        600);

        Pipeline pipeline = new Pipeline.Builder(manager)
                .addContinuousAction(armPositionAction)
                .addAction(toggleClawAction)
                .addAction(new DelayAction(manager, 1500))
                .addAction(new SetArmAction(manager, 11000))
                .addLinearPath(
                        new Position(600, 160,  0),
                        new Position(600, 770,  0),
                        new Position(600, 770,  Math.PI / 4),
                        new Position(810, 990,  Math.PI / 4)
                )
                .addAction(new FullStopAction(manager))
                .addAction(new WaitAction(manager, armPositionAction))
                .addAction(toggleClawAction)
                .addAction(new SetArmAction(manager, 0))
                .addLinearPath(
                        new Position(600, 770, Math.PI / 4),
                        new Position(600, 770, 0),
                        new Position(600, 1400, 0),
                        new Position(parkingPos, 1400, 0)
                )
                .addAction(new FullStopAction(manager))
                .addAction(new WaitAction(manager, armPositionAction))
                .build();


        pipeline.execute();
    }
}
