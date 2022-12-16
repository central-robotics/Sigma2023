package org.firstinspires.ftc.teamcode.teleop;


import com.chsrobotics.ftccore.hardware.HardwareManager;
import com.chsrobotics.ftccore.hardware.config.Config;
import com.chsrobotics.ftccore.hardware.config.accessory.Accessory;
import com.chsrobotics.ftccore.hardware.config.accessory.AccessoryType;
import com.chsrobotics.ftccore.teleop.Drive;
import com.chsrobotics.ftccore.teleop.UserDriveLoop;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.auto.actions.ArmPositionAction;

@TeleOp(name="Sigma TeleOp")
public class DriverControl extends LinearOpMode {
    public static Telemetry telem;
    @Override
    public void runOpMode() throws InterruptedException {
        telem = telemetry;
        Config config = new Config.Builder()
                .setDriveMotors("m0", "m1", "m2", "m3")
                .setMotorDirection(DcMotorSimple.Direction.FORWARD)
                .reverseTheta()
                .addAccessory(new Accessory(AccessoryType.MOTOR, "l0"))
                .addAccessory(new Accessory(AccessoryType.MOTOR, "l1"))
                .addAccessory(new Accessory(AccessoryType.SERVO, "c0"))
                .addAccessory(new Accessory(AccessoryType.SERVO, "c1"))
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

            private void setLiftPower(double val) {
                telemetry.addData("val", val);
                manager.accessoryMotors[0].setPower(val);
                manager.accessoryMotors[1].setPower(-val);
            }

            private void setClawPower(double val) {
                manager.accessoryServos[0].setPosition(val);
                manager.accessoryServos[1].setPosition(0.9-val);
            }

            @Override
            public void loop() {
                ArmPositionAction armPositionAction = new ArmPositionAction(manager);
                double armPos = manager.accessoryMotors[0].getCurrentPosition();

                if (gamepad1.right_trigger > 0.1)
                {
                    if (armPos < 3950)
                        setLiftPower(0.6);
                    else
                        armPositionAction.execute();
                } else if (gamepad1.left_trigger > 0.1)
                {
                    if (armPos > 0)
                        setLiftPower(-0.6);
                    else
                        armPositionAction.execute();
                } else
                {
                    armPositionAction.execute();
                }
                telemetry.addData("l0", manager.accessoryMotors[0].getCurrentPosition());
                telemetry.addData("l1", manager.accessoryMotors[1].getCurrentPosition());
                telemetry.addData("c0", clawClosed);
                telemetry.update();

                ArmPositionAction.targetArmPos = armPos;

                if (gamepad1.b && System.currentTimeMillis() - bLastPressed > 250) {
                    clawClosed = !clawClosed;
                    bLastPressed = System.currentTimeMillis();
                }

                if (clawClosed) {
                    setClawPower(0.85);
                } else
                {
                    setClawPower(0.5);
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
