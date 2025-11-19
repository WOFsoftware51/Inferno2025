package org.firstinspires.ftc.teamcode.Autons;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;


@Autonomous(name = "JustShoot")
public class JustDrive extends LinearOpMode {
    boolean inferno = false;
    DcMotor motorFrontLeft;
    DcMotor motorBackLeft;
    DcMotor motorFrontRight;
    DcMotor motorBackRight;
    IMU imu;


    public double yaw = 0;


    double yp;
    double xp;

    double timer;

    @Override
    public void runOpMode() {
        motorFrontLeft  = hardwareMap.get(DcMotor.class, "frontLeft");
        motorBackLeft  = hardwareMap.get(DcMotor.class, "backLeft");
        motorFrontRight = hardwareMap.get(DcMotor.class, "frontRight");
        motorBackRight = hardwareMap.get(DcMotor.class, "backRight");
        imu = hardwareMap.get(IMU.class, "imu");

        if(inferno) {
            motorFrontLeft.setDirection(DcMotor.Direction.REVERSE);
            motorBackLeft.setDirection(DcMotor.Direction.FORWARD);
            motorFrontRight.setDirection(DcMotor.Direction.FORWARD);
            motorBackRight.setDirection(DcMotor.Direction.REVERSE);
        }

        else {
            motorFrontLeft.setDirection(DcMotor.Direction.FORWARD);
            motorBackLeft.setDirection(DcMotor.Direction.REVERSE);
            motorFrontRight.setDirection(DcMotor.Direction.REVERSE);
            motorBackRight.setDirection(DcMotor.Direction.FORWARD);
        }

        timer = 0;

        waitForStart();

        if (opModeIsActive()) {
            while (opModeIsActive()) {
                telemetry.addData("Status", "Initialized");
                telemetry.update();


                while(timer<100) {
                    drive(0, -0.5, 0);
                }
                while (timer > 100) {
                    drive(0, 0, 0);
                }

            }

            sleep(200);

        }


    }


    public void drive(double x, double y, double rx) {
        yp = y;
        xp = x;


        // Combine the joystick requests for each axis-motion to determine each wheel's power.
        // Set up a variable for each drive wheel to save the power level for telemetry.
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (yp + xp + rx) / denominator;
        double frontRightPower = (yp - xp - rx) / denominator;
        double backLeftPower = (yp - xp + rx) / denominator;
        double backRightPower = (yp + xp - rx) / denominator;
        motorFrontLeft.setPower(-frontLeftPower);
        motorFrontRight.setPower(-frontRightPower);
        motorBackLeft.setPower(-backLeftPower);
        motorBackRight.setPower(-backRightPower);

    }
}
