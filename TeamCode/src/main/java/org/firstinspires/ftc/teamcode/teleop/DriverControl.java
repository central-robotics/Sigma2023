package org.firstinspires.ftc.teamcode.teleop;


import com.chsrobotics.ftccore.hardware.HardwareManager;
import com.chsrobotics.ftccore.hardware.config.Config;
import com.chsrobotics.ftccore.hardware.config.accessory.Accessory;
import com.chsrobotics.ftccore.hardware.config.accessory.AccessoryType;
import com.chsrobotics.ftccore.teleop.Drive;
import com.chsrobotics.ftccore.teleop.UserDriveLoop;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.auto.actions.ArmPositionAction;
import org.firstinspires.ftc.teamcode.auto.actions.SetArmAction;
import org.firstinspires.ftc.teamcode.auto.actions.ToggleClawAction;

@TeleOp(name="Sigma TeleOp")
public class DriverControl extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Config config = new Config.Builder()
                .setDriveMotors("m0", "m1", "m2", "m3")
                .addAccessory(new Accessory(AccessoryType.MOTOR, "l0"))
                .addAccessory(new Accessory(AccessoryType.MOTOR, "c0"))
                .setIMU("imu")
                .setTeleopValues(0.6, 0.6)
                .setOpMode(this)
                .build();

        HardwareManager manager = new HardwareManager(config, hardwareMap);

        UserDriveLoop armLoop = new UserDriveLoop(manager, this) {
            long bLastPressed;
            boolean clawClosed;

            long bumperLastPressed;
            boolean isPrecision;

            @Override
            public void loop() {
                ArmPositionAction armPositionAction = new ArmPositionAction(manager);
                double armPos = manager.accessoryMotors[0].getCurrentPosition();

                if (gamepad1.right_trigger > 0.1)
                {
                    if (armPos < 11000)
                        manager.accessoryMotors[0].setPower(1);
                    else
                        armPositionAction.execute();
                } else if (gamepad1.left_trigger > 0.1)
                {
                    if (armPos > 0)
                        manager.accessoryMotors[0].setPower(-1);
                    else
                        armPositionAction.execute();
                } else
                {
                    armPositionAction.execute();
                }

                ArmPositionAction.targetArmPos = armPos;

                if (gamepad1.b && System.currentTimeMillis() - bLastPressed > 250) {
                    clawClosed = !clawClosed;
                    bLastPressed = System.currentTimeMillis();
                }

                if (clawClosed) {
                    manager.accessoryMotors[1].setPower(0.1);
                } else
                {
                    if (System.currentTimeMillis() - bLastPressed < 1250)
                    {
                        manager.accessoryMotors[1].setPower(-0.2);
                    }
                    else{
                        manager.accessoryMotors[1].setPower(0);
                    }
                }

                if (gamepad1.right_bumper && System.currentTimeMillis() - bumperLastPressed > 250) {
                    isPrecision = !isPrecision;
                    bumperLastPressed = System.currentTimeMillis();
                }

                if (isPrecision) {
                    manager.linearSpeed = 0.3;
                    manager.rotSpeed = 0.3;
                } else
                {
                    manager.linearSpeed = 0.6;
                    manager.rotSpeed = 0.6;
                }
            }
        };

        Drive drive = new Drive.Builder(manager)
                .addUserLoop(armLoop)
                .build();

        waitForStart();

        drive.runDriveLoop();
    }
}
