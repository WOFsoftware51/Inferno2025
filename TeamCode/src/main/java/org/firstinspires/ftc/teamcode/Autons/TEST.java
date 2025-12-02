package org.firstinspires.ftc.teamcode.Autons;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.teamcode.Constants;


@Autonomous(name = "TEST")
public class TEST extends LinearOpMode {
    DcMotor motorFrontLeft;
    DcMotor motorBackLeft;
    DcMotor motorFrontRight;
    DcMotor motorBackRight;
    DcMotor motorIntake;

    IMU imu;
    IMU imu2;
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
        DRIVEFORWARDINTAKE,
        TURN2,
        DRIVEFORWARD,
        RESET,
        STOP
    }

    double yp;
    double xp;

    static double rx = 0;
    double yaw;
    double yaw2;

    double driveEncoder;
    State currentState = State.START;
    double shooterLimit = -2500;
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
        motorIntake = hardwareMap.get(DcMotor.class, "motorIntake");


        imu = hardwareMap.get(IMU.class, "imu");
        imu2 = hardwareMap.get(IMU.class, "imu2");
        final_Orientation(imu);
        final_Orientation2(imu2);


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
                telemetry.addData("Yaw", yaw);
                telemetry.addData("Yaw2", yaw2);
                telemetry.addData("rx", rx);


                driveEncoder = motorBackLeft.getCurrentPosition();
                yaw = imu.getRobotYawPitchRollAngles().getYaw();
                yaw2 = imu2.getRobotYawPitchRollAngles().getYaw();

                if(yaw == 0.0){
                    yaw = yaw2;
                }

                shooterRightVelocity = ((motorShooterRight.getVelocity() / 28) * 60);
                telemetry.update();

                switch (currentState) {
                    case START:
                        motorShooterLeft.setPower(0);
                        motorShooterRight.setPower(0);
                        motorTransfer.setPower(0.0);
                        motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        imu.resetYaw();
                        motorIntake.setPower(0.0);
                        drive(0, 0, 0);
                        timer++;
                        if (timer > 25) {
                            timer = 0;
                            currentState = State.STOP;
                        }
                        break;
                    case READYSHOOTER:
                        motorShooterLeft.setPower(1.0);
                        motorShooterRight.setPower(-1.0);
                        motorIntake.setPower(0.0);
                        drive(0, 0, 0);
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
                        if (timer > 200) {
                            timer = 0;
                            currentState = State.RAMPDOWN;
                        }
                        break;
                    case RAMPDOWN:
                        motorShooterLeft.setPower(0.0);
                        motorShooterRight.setPower(0.0);
                        motorTransfer.setPower(0.3);
                        timer++;
                        if (timer > 100) {
                            timer = 0;
                            motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                            currentState = State.STOP;
                        }
                        break;
                    case DRIVEBACK:
                        motorShooterLeft.setPower(0.0);
                        motorShooterRight.setPower(0.0);
                        motorTransfer.setPower(0.0);
                        drive(0, 0.8, 0);
                        timer++;
                        if (driveEncoder < -4000) {
                            timer = 0;
                            rx = 0;
                            yp = 0;
                            xp = 0;
                            currentState = State.TURN;
                        }
                        break;
                    case TURN:
                        motorShooterLeft.setPower(0.0);
                        motorShooterRight.setPower(0.0);
                        motorTransfer.setPower(0.0);
                        driveTurnCCW(0.5, 135);
                        timer++;
                        if (timer > 200 || yaw > 133) {
                            timer = 0;
                            currentState = State.DRIVEFORWARD;
                        }
                        break;
                    case RESET:
                        motorShooterLeft.setPower(0.0);
                        motorShooterRight.setPower(0.0);
                        motorTransfer.setPower(0.3);
                        timer++;
                        if (timer > 50) {
                            timer = 0;
                            motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                            currentState = State.DRIVEFORWARDINTAKE;
                        }
                        break;
                    case TURN2:
                        motorShooterLeft.setPower(0.0);
                        motorShooterRight.setPower(0.0);
                        motorTransfer.setPower(0.0);
                        driveTurnCW(0.5, 125);
                        timer++;
                        if (timer > 1000 || yaw < 126) {
                            timer = 0;
                            currentState = State.READYSHOOTER;
                        }
                        break;
                    case DRIVEFORWARDINTAKE:
                        motorShooterLeft.setPower(0.0);
                        motorShooterRight.setPower(0.0);
                        motorTransfer.setPower(0.0);
                        motorIntake.setPower(1.0);
                        drive(-0.5, -0.5, 0);
                        timer++;
                        if (timer > 1000) {
                            timer = 0;
                            currentState = State.READYSHOOTER;
                        }
                        break;
                    case DRIVEFORWARD:
                        motorShooterLeft.setPower(0.0);
                        motorShooterRight.setPower(0.0);
                        motorTransfer.setPower(0.0);
                        motorIntake.setPower(1.0);
                        drive(-0.3, -0.2, 0);
                        timer++;
                        if (timer > 100) {
                            timer = 0;
                            currentState = State.TURN2;
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

    public void driveStraight(double speed, double target_yaw, double rx) {
        //yp = speed;
        xp = 0;
        if(yp<speed){
            yp = yp + 0.03;
        } else if (yp > speed) {
            yp = yp - 0.03;
        }
        if (Math.abs(target_yaw - yaw) < 0.1){
            TEST.rx = 0;
        }
        else  {
            TEST.rx =  (target_yaw - yaw)/9;
        }


        xp = -rx * 0.7;

        // Combine the joystick requests for each axis-motion to determine each wheel's power.
        // Set up a variable for each drive wheel to save the power level for telemetry.
        double denominator = Math.max(Math.abs(yp)  + Math.abs(rx), 1);
        double frontLeftPower = (yp + xp + rx) / denominator;
        double frontRightPower = (yp - xp - rx) / denominator;
        double backLeftPower = (yp - xp + rx) / denominator;
        double backRightPower = (yp + xp - rx) / denominator;
        motorFrontLeft.setPower(-frontLeftPower);
        motorFrontRight.setPower(-frontRightPower);
        motorBackLeft.setPower(-backLeftPower);
        motorBackRight.setPower(-backRightPower);

    }

    public void driveTurnCCW(double speed, double target_yaw) {
        double m_rx;
        if (target_yaw - yaw < 1){
            m_rx = 0;
        }
        else if(target_yaw - yaw < 5){
            m_rx = speed * 0.3;
        }
        else if (target_yaw - yaw < 15){
            m_rx = speed * 0.5;
        }
        else{
            m_rx = speed;
        }

        // Combine the joystick requests for each axis-motion to determine each wheel's power.
        // Set up a variable for each drive wheel to save the power level for telemetry.
        double denominator =  1;
        double frontLeftPower = (yp + xp + m_rx) / denominator;
        double frontRightPower = (yp - xp - m_rx) / denominator;
        double backLeftPower = (yp - xp + m_rx) / denominator;
        double backRightPower = (yp + xp - m_rx) / denominator;
        motorFrontLeft.setPower(-frontLeftPower);
        motorFrontRight.setPower(-frontRightPower);
        motorBackLeft.setPower(-backLeftPower);
        motorBackRight.setPower(-backRightPower);

    }

    public void driveTurnCW(double speed, double target_yaw) {
        double m_rx;
        if (target_yaw - yaw > -1){
            m_rx = 0;
        }
        else if(target_yaw - yaw > -5){
            m_rx = -speed * 0.3;
        }
        else if (target_yaw - yaw > -15){
            m_rx = -speed * 0.5;
        }
        else{
            m_rx = -speed;
        }

        // Combine the joystick requests for each axis-motion to determine each wheel's power.
        // Set up a variable for each drive wheel to save the power level for telemetry.
        double denominator =  1;
        double frontLeftPower = (yp + xp + m_rx) / denominator;
        double frontRightPower = (yp - xp - m_rx) / denominator;
        double backLeftPower = (yp - xp + m_rx) / denominator;
        double backRightPower = (yp + xp - m_rx) / denominator;
        motorFrontLeft.setPower(-frontLeftPower);
        motorFrontRight.setPower(-frontRightPower);
        motorBackLeft.setPower(-backLeftPower);
        motorBackRight.setPower(-backRightPower);

    }



    public void final_Orientation(IMU m_imu){
        RevHubOrientationOnRobot orientationOnRobot =
                new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.RIGHT, RevHubOrientationOnRobot.UsbFacingDirection.UP);
        m_imu.initialize(new IMU.Parameters(orientationOnRobot));
    }
    public void final_Orientation2(IMU m_imu){
        RevHubOrientationOnRobot orientationOnRobot =
                new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.LEFT, RevHubOrientationOnRobot.UsbFacingDirection.DOWN);
        m_imu.initialize(new IMU.Parameters(orientationOnRobot));
    }
}
