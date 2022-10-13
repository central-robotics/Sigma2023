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

@Autonomous
public class TestAuton extends LinearOpMode
{
    @Override
    public void runOpMode() throws InterruptedException
    {
        Config config = new Config.Builder()
                .setDriveMotors("m0", "m1", "m2", "m3")
                .setIMU("imu")
                .addAccessory(new Accessory(AccessoryType.WEBCAM, "webcam"))
                .setPIDCoefficients(new PIDCoefficients(0.01, 0.00003, 0), new PIDCoefficients(0.25, 0.0004, 0))
                .build();

        HardwareManager manager = new HardwareManager(config, hardwareMap);

        Pipeline pipeline = new Pipeline.Builder(manager)
                .addLinearPath(new Position(0, 0, 0), new Position(100, 0, 0), new Position(100, 100, Math.PI))
                .build();

        pipeline.execute();
    }
}
