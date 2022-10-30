package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp
public class Drive extends Core {
    double positive_power, negative_power, rot_power;
    double joystick_x, joystick_y, joystick_power;
    double orientation;
    long bLastPressed = -1;
    boolean isClawClosed = false;
    Orientation gyro_angles;
    double liftPower;

    PID liftController = new PID(new PIDCoefficients(0.003, 0, 0));
    double liftTarget;
    long prevTime = System.currentTimeMillis();
    boolean turboMode;

    public void loop()
    {
        turboMode = gamepad1.y;

        prevTime = System.currentTimeMillis();
        // Get all the info we from the gamepad
        joystick_y = gamepad1.left_stick_y > 0 ? Math.pow(gamepad1.left_stick_y, 2) :
                -Math.pow(gamepad1.left_stick_y, 2);
        joystick_x = (gamepad1.left_stick_x == 0) ? 0.000001 :
                (gamepad1.left_stick_x > 0 ? Math.pow(gamepad1.left_stick_x, 2) :
                        -Math.pow(gamepad1.left_stick_x, 2));
        rot_power = 0.4 * (gamepad1.right_stick_x);

        // Find out the distance of the joystick from resting position to control speed
        joystick_power = Math.sqrt(Math.pow(joystick_x, 2) + Math.pow(joystick_y, 2));

        // Pull raw orientation values from the gyro
        gyro_angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
        double theta = gyro_angles.firstAngle; // Add pi for CPU's robot

        // Turn the joystick coordinates into an angle in radians
        orientation = (joystick_x > 0) ? (Math.atan(-joystick_y / joystick_x) - Math.PI / 4) - theta :
                (Math.atan(-joystick_y / joystick_x) + Math.PI - Math.PI / 4) - theta;

        if (gamepad1.right_trigger > 0.5 && (lift.getCurrentPosition() < 4800 || gamepad1.dpad_left)) {
            liftPower = 1;
            liftTarget = lift.getCurrentPosition();
        } else if (gamepad1.left_trigger > 0.5 && (lift.getCurrentPosition() > 0 || gamepad1.dpad_left)){
            liftPower = -1;
            liftTarget = lift.getCurrentPosition();
        } else {
            liftPower = liftController.getOutput(liftTarget - lift.getCurrentPosition(), 0);
            // todo: lift motor goes slower in PID. solution: override PID when it's mildly far from it's target
            // current problem: manual lift and then claw open leads to lift continuing to go up
//            if (liftTarget - liftMotor.getCurrentPosition() > 150) {
//                liftPower = -1;
//            } else if (liftTarget == 0 && liftMotor.getCurrentPosition() > 150) {
//                liftPower = 1;
//            }
        }

        if (!gamepad1.dpad_left) {
            if (liftTarget > 4800) {
                liftTarget = 4800;
            } else if (liftTarget < 0) {
                liftTarget = 0;
            }
        }


        lift.setPower(liftPower);

        telemetry.addData("theta", theta);
        telemetry.addData("orientation", orientation);
        telemetry.update();

        // Pass that angle through a pair of wave functions to get the power for each corresponding pair of parallel wheels
        negative_power = (joystick_power * Math.sin(orientation));
        positive_power = (orientation != 0) ? (joystick_power * Math.cos(orientation)) :
                negative_power;

        if (!turboMode) {
            negative_power *= 0.6;
            positive_power *= 0.6;
            rot_power *= 0.6;
        }

        // This is all we need to actually move the robot, method decs in Core
        move(positive_power, negative_power, rot_power);

        if (gamepad1.b && System.currentTimeMillis() - bLastPressed > 250) {
            bLastPressed = System.currentTimeMillis();
            isClawClosed = !isClawClosed;
        }
        if (isClawClosed){
            claw.setPower(-0.3);
        } else if (System.currentTimeMillis() - bLastPressed < 200) {
            claw.setPower(0.15);
        }

    }
}
