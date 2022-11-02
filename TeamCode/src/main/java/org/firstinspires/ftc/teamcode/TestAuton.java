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

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Autonomous
public class TestAuton extends LinearOpMode
{
    @Override
    public void runOpMode()
    {
        Config config = new Config.Builder()
                .setDebugMode(false)
                .setDriveMotors("m0", "m1", "m2", "m3")
                .setIMU("imu")
//                .addAccessory(new Accessory(AccessoryType.WEBCAM, "webcam"))
                .setPIDCoefficients(new PIDCoefficients(0.015, 0.000005, 0), new PIDCoefficients(0.2, 0.0005, 0))
                .setOpMode(this)
                .build();

        HardwareManager manager = new HardwareManager(config, hardwareMap);

        Pipeline pipeline = new Pipeline.Builder(manager)
                .addLinearPath(
                        new Position(0, 0, 0),
                        new Position(1000, 0, 0),
                        new Position(1000, 1000, 0),
                        new Position(0, 1000, 0),
                        new Position(0, 0, 0))
                .build();

        telemetry.addLine("test");

        telemetry.update();
        waitForStart();

        pipeline.execute();
    }
}
