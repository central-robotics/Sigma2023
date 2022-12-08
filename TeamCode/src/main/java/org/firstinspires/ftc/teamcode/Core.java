package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
//import com.qualcomm.robotcore.hardware.Servo;


public class Core extends OpMode {
    DcMotor leftfront, rightfront, leftback, rightback, lift, claw;
    BNO055IMU imu;
    BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

    public void loop(){}

    public void init()
    {
        leftfront = hardwareMap.dcMotor.get("m0");
        leftfront.setDirection(DcMotorSimple.Direction.FORWARD);

        rightfront = hardwareMap.dcMotor.get("m1");
        rightfront.setDirection(DcMotorSimple.Direction.FORWARD);

        rightback = hardwareMap.dcMotor.get("m2");
        rightback.setDirection(DcMotorSimple.Direction.FORWARD);

        leftback = hardwareMap.dcMotor.get("m3");
        leftback.setDirection(DcMotorSimple.Direction.FORWARD);

        lift = hardwareMap.dcMotor.get("l0");
        lift.setDirection(DcMotorSimple.Direction.FORWARD);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        claw = hardwareMap.dcMotor.get("c0");
        claw.setDirection(DcMotorSimple.Direction.FORWARD);

        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        parameters.loggingEnabled = false;

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
    }

    public void move(double posinput, double neginput, double rotinput)
    {
        leftfront.setPower(-posinput+rotinput);
        rightfront.setPower(neginput+rotinput);
        leftback.setPower(-neginput+rotinput);
        rightback.setPower(posinput+rotinput);
    }

}
