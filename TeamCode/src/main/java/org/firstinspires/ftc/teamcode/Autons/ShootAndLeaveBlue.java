package org.firstinspires.ftc.teamcode.Autons;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.teamcode.Constants;


@Autonomous(name = "ShootAndLeaveBlue")
public class ShootAndLeaveBlue extends LinearOpMode {
    DcMotor motorFrontLeft;
    DcMotor motorBackLeft;
    DcMotor motorFrontRight;
    DcMotor motorBackRight;
    IMU imu;
    DcMotorEx motorShooterLeft;
    DcMotorEx motorShooterRight;
    DcMotor motorTransfer;

    private enum State {
        START,
        READYSHOOTER,
        SHOOT,
        RAMPDOWN,
        DRIVEBACK,
        TURN,
        DRIVEFORWARD,
        STOP
    }

    double yp;
    double xp;
    double driveEncoder;
    State currentState = State.START;
    double shooterLimit = -2450;
    double shooterRightVelocity;
    double timer;
    @Override
    public void runOpMode() {

        // Initialize hardware
        motorFrontLeft  = hardwareMap.get(DcMotor.class, "frontLeft");
        motorBackLeft  = hardwareMap.get(DcMotor.class, "backLeft");
        motorFrontRight = hardwareMap.get(DcMotor.class, "frontRight");
        motorBackRight = hardwareMap.get(DcMotor.class, "backRight");
        motorShooterLeft = (DcMotorEx) hardwareMap.get(DcMotor.class, "motorShooterLeft");
        motorShooterRight = (DcMotorEx) hardwareMap.get(DcMotor.class, "motorShooterRight");
        motorTransfer = (DcMotorEx) hardwareMap.get(DcMotor.class, "motorTransfer");

        imu = hardwareMap.get(IMU.class, "imu");

        if(Constants.inferno) {
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
                telemetry.addData("Current State", currentState);
                telemetry.addData("Right Velocity", shooterRightVelocity);
                telemetry.addData("Drive Encoder", driveEncoder);
                driveEncoder = motorBackLeft.getCurrentPosition();
                shooterRightVelocity = ((motorShooterRight.getVelocity() / 28) * 60);
                telemetry.update();

                switch (currentState) {
                    case START:
                        motorShooterLeft.setPower(0);
                        motorShooterRight.setPower(0);
                        motorTransfer.setPower(0.0);
                        drive(0, 0, 0);
                        timer++;
                        if (timer > 100) {
                            timer = 0;
                            currentState = State.READYSHOOTER;
                        }
                        break;
                    case READYSHOOTER:
                        motorShooterLeft.setPower(1.0);
                        motorShooterRight.setPower(-1.0);
                        if (shooterRightVelocity < shooterLimit) {
                            timer = 0;
                            currentState = State.SHOOT;
                        }
                        break;
                    case SHOOT:
                        if(shooterRightVelocity >= shooterLimit){ //-.2600 works
                            motorShooterRight.setPower(-1.0);
                            motorShooterLeft.setPower(1.0);
                            telemetry.addData("Shooting Speed:", "Speeding Up");
                        }
                        else {
                            motorShooterRight.setPower(-0.75);
                            motorShooterLeft.setPower(0.75);
                            telemetry.addData("Shooting Speed:", "Slowing Down");
                        }
                        motorTransfer.setPower(-0.6);
                        timer++;
                        if (timer > 400) {
                            timer = 0;
                            currentState = State.RAMPDOWN;
                        }
                        break;
                    case RAMPDOWN:
                        motorShooterLeft.setPower(0.0);
                        motorShooterRight.setPower(0.0);
                        motorTransfer.setPower(0.3);
                        timer++;
                        if (timer > 200) {
                            timer = 0;
                            currentState = State.DRIVEBACK;
                        }
                        break;
                    case DRIVEBACK:
                        motorShooterLeft.setPower(0.0);
                        motorShooterRight.setPower(0.0);
                        motorTransfer.setPower(0.0);
                        drive(0.5, 0.2, 0);
                        timer++;
                        if (timer > 170) {
                            timer = 0;
                            currentState = State.TURN;
                        }
                        break;
                    case TURN:
                        motorShooterLeft.setPower(0.0);
                        motorShooterRight.setPower(0.0);
                        motorTransfer.setPower(0.0);
                        drive(-0.1, 0.0, 0.5);
                        timer++;
                        if (timer > 170) {
                            timer = 0;
                            currentState = State.STOP;
                        }
                        break;
                    case STOP:
                        motorShooterLeft.setPower(0.0);
                        motorShooterRight.setPower(0.0);
                        motorTransfer.setPower(0.0);
                        drive(0, 0, 0);
                        break;
                    default:
                        currentState = State.STOP;
                        break;
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
