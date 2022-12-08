package org.firstinspires.ftc.teamcode.auto;

import com.chsrobotics.ftccore.geometry.Position;
import com.chsrobotics.ftccore.hardware.HardwareManager;
import com.chsrobotics.ftccore.hardware.config.Config;
import com.chsrobotics.ftccore.pipeline.Pipeline;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

import org.firstinspires.ftc.teamcode.auto.util.OpModeHolder;
import org.firstinspires.ftc.teamcode.auto.util.WebcamPipeline;

public class NorthBlueAuto extends LinearOpMode
{
    @Override
    public void runOpMode()
    {
        OpModeHolder.opMode = this;
        WebcamPipeline.clearLastMat();
        Config config = new Config.Builder()
                .setDebugMode(false)
                .setDriveMotors("m0", "m1", "m2", "m3")
                .setOpMode(this)
                .setIMU("imu")
                .setPIDCoefficients(new PIDCoefficients(1.3, 0.002, 0), new PIDCoefficients(550, 0.7, 0))
                .build();

        HardwareManager manager = new HardwareManager(config, hardwareMap);

        Pipeline pipeline = new Pipeline.Builder(manager)
                .addCurvedPath(
                        new Position(0, 0, 0),
                        new Position(500, 900, 0),
                        new Position(1000, -900, 0),
                        new Position(1500, 0, 0))
                .build();

        waitForStart();

        pipeline.execute();
    }
}
