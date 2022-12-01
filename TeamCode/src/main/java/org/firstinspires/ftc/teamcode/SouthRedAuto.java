package org.firstinspires.ftc.teamcode;

import com.chsrobotics.ftccore.geometry.Position;
import com.chsrobotics.ftccore.hardware.HardwareManager;
import com.chsrobotics.ftccore.hardware.config.Config;
import com.chsrobotics.ftccore.hardware.config.accessory.Accessory;
import com.chsrobotics.ftccore.hardware.config.accessory.AccessoryType;
import com.chsrobotics.ftccore.pipeline.Pipeline;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

import org.firstinspires.ftc.teamcode.actions.ArmPositionAction;
import org.firstinspires.ftc.teamcode.actions.SetArmAction;

@Autonomous(name = "South Red Auto")
public class SouthRedAuto extends LinearOpMode
{
    @Override
    public void runOpMode()
    {
        Config config = new Config.Builder()
                .setDebugMode(false)
                .setDriveMotors("m0", "m1", "m2", "m3")
                .addAccessory(new Accessory(AccessoryType.MOTOR, "liftMotor"))
                .addAccessory(new Accessory(AccessoryType.WEBCAM, "webcam"))
                .setOpMode(this)
                .setIMU("imu")
                .setPIDCoefficients(new PIDCoefficients(3.3, 0.002, 0), new PIDCoefficients(400, 0.07, 0))
                .build();

        HardwareManager manager = new HardwareManager(config, hardwareMap);

        Pipeline pipeline = new Pipeline.Builder(manager)
                .addContinuousAction(new ArmPositionAction(manager))
                .addLinearPath(
                        new Position(0, 330,  0),
                        new Position(75, 693,  7 * Math.PI / 4),
                        new Position(0, 700, Math.PI / 2),
                        new Position(-630, 700, Math.PI / 2)
                )
                .addAction(new SetArmAction(manager, 100))
                .build();

        waitForStart();

        pipeline.execute();
    }
}
