package org.firstinspires.ftc.teamcode;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;
//import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;


import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

//import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;


@TeleOp(name="Mecanum")
public class Mecanum extends LinearOpMode {

    boolean inferno = true;
    ElapsedTime runtime = new ElapsedTime();

    // Declare OpMode members for each of the 4 motors.
    DcMotor motorFrontLeft;
    DcMotor motorBackLeft;
    DcMotor motorFrontRight;
    DcMotor motorBackRight;
    DcMotor motorTransfer;
    IMU imu;
//    Limelight3A limelight;
//    LLResult result;
    double tx;
    double ty;
    double ta;
    double poseX;
    double poseY;
    Pose3D botpose2;

    DcMotor motorIntake;
    DcMotorEx motorShooterLeft;
    DcMotorEx motorShooterRight;

    PIDFCoefficients shooterPID;

    public double yaw = 0;
    public double pitch = 0;
    public double roll = 0;
    double yp;
    double xp;

    TelemetryManager panelsTelemetry;


    @Override
    public void runOpMode() {
        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        motorFrontLeft  = hardwareMap.get(DcMotor.class, "frontLeft");
        motorBackLeft  = hardwareMap.get(DcMotor.class, "backLeft");
        motorFrontRight = hardwareMap.get(DcMotor.class, "frontRight");
        motorBackRight = hardwareMap.get(DcMotor.class, "backRight");
        imu = hardwareMap.get(IMU.class, "imu");


        motorShooterLeft = (DcMotorEx)hardwareMap.get(DcMotor.class, "motorShooterLeft");
        motorShooterRight = (DcMotorEx)hardwareMap.get(DcMotor.class, "motorShooterRight");
        motorTransfer = (DcMotorEx)hardwareMap.get(DcMotor.class, "motorTransfer");
        motorIntake = hardwareMap.get(DcMotor.class, "motorIntake");

        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

//        limelight = hardwareMap.get(Limelight3A.class, "limelight");
//        limelight.setPollRateHz(100); // This sets how often we ask Limelight for data (100 times per second)
//
//        limelight.start(); // This tells Limelight to start looking!
//        result = limelight.getLatestResult();
//        limelight.updateRobotOrientation(yaw);
//        if (result != null && result.isValid()) {
//            tx = result.getTx(); // How far left or right the target is (degrees)
//            ty = result.getTy(); // How far up or down the target is (degrees)
//            ta = result.getTa(); // How big the target looks (0%-100% of the image)
//        }

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


        // Wait for the game to start (driver presses START)
        telemetry.addData("Status", "Initialized");
        telemetry.update();


        shooterPID = new PIDFCoefficients(15, 0, 0, 0);
        motorShooterLeft.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, shooterPID);
        motorShooterRight.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, shooterPID);



        String centric;
        yp = 0;
        xp = 0;

        boolean slow = false;
        int isFieldCentric = 0;


        waitForStart();
        runtime.reset();
        if (isStopRequested()) return;

        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.FORWARD)));
        imu.resetYaw();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            yaw = imu.getRobotYawPitchRollAngles().getYaw();
            pitch = imu.getRobotYawPitchRollAngles().getPitch();
            roll = imu.getRobotYawPitchRollAngles().getRoll();

            double shooterLeftVelocity = ((motorShooterLeft.getVelocity()/28)*60);
            double shooterRightVelocity = ((motorShooterRight.getVelocity()/28)*60);

            double transferAngle = ((double)motorTransfer.getCurrentPosition()/28)*100/360;

            if(gamepad2.right_bumper){
                //motorShooterRight.setPower(1.0);
                //motorShooterLeft.setPower(1.0);
                if(shooterRightVelocity >= -2450){ //-.2600 works
                    motorShooterRight.setPower(-1.0);
                    motorShooterLeft.setPower(1.0);
                    telemetry.addData("Shooting Speed:", "Speeding Up");
                } else {
                    motorShooterRight.setPower(-0.75);
                    motorShooterLeft.setPower(0.75);
                    telemetry.addData("Shooting Speed:", "Slowing Down");
                }
//                motorShooterRight.setTargetPosition(3500);
//                motorShooterLeft.setTargetPosition(3500);
            }
            else{
                motorShooterRight.setPower(0.0);
                motorShooterLeft.setPower(0.0);
//                motorShooterRight.setTargetPosition(0);
//                motorShooterLeft.setTargetPosition(0);
            }



            //Transfer
            if(gamepad2.y) {
                motorTransfer.setPower(-0.6);
//                 motorTransfer.setTargetPosition(0);
            }
            else if(gamepad2.a){
                motorTransfer.setPower(0.6);
//                motorTransfer.setTargetPosition(0);
            }
//            else if(gamepad2.a){
//                motorTransfer.setPower(-0.1);
//                motorTransfer.setTargetPosition(0);
//            }
            else {
                motorTransfer.setPower(0.0);
            }



            if(gamepad2.x) {
                motorIntake.setPower(1.0);
            }
            else if(gamepad2.b){
                motorIntake.setPower(-1.0);           }
            else{
                motorIntake.setPower(0);
            }



            if(gamepad1.start && isFieldCentric == 0){
                isFieldCentric = 1;

            }
            if(gamepad1.start && isFieldCentric == 1){
                isFieldCentric = 0;

            }
            if(gamepad1.back)
            {
                imu.resetYaw();
            }
            if(isFieldCentric==1){
                centric = "Field";
            }
            else{
                centric = "Robot";
            }

            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            double y = gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
            double x = -gamepad1.left_stick_x;
            double rx = -gamepad1.right_stick_x*0.8;

            slow = gamepad1.right_bumper;


            if(isFieldCentric==1){
                yp = (y*cos(yaw) - x*sin(yaw));
                xp = (x*cos(yaw) + y*sin(yaw));
            }
            else{
                yp = y;
                xp = x;
            }


            // Combine the joystick requests for each axis-motion to determine each wheel's power.
            // Set up a variable for each drive wheel to save the power level for telemetry.
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (yp + xp + rx) / denominator;
            double frontRightPower = (yp - xp - rx) / denominator;
            double backLeftPower = (yp - xp + rx) / denominator;
            double backRightPower = (yp + xp - rx) / denominator;




            if(slow) {
                motorFrontLeft.setPower(-frontLeftPower*0.25);
                motorFrontRight.setPower(-frontRightPower*0.25);
                motorBackLeft.setPower(-backLeftPower*0.25);
                motorBackRight.setPower(-backRightPower*0.25);
            }
            else{
                motorFrontLeft.setPower(-frontLeftPower);
                motorFrontRight.setPower(-frontRightPower);
                motorBackLeft.setPower(-backLeftPower);
                motorBackRight.setPower(-backRightPower);
            }


//            result = limelight.getLatestResult();
//            limelight.updateRobotOrientation(yaw);
//            if (result != null && result.isValid()) {
//                tx = result.getTx(); // How far left or right the target is (degrees)
//                ty = result.getTy(); // How far up or down the target is (degrees)
//                ta = result.getTa(); // How big the target looks (0%-100% of the image)
//
//                botpose2 = result.getBotpose_MT2();
//                if (botpose2 != null) {
//                    poseX = botpose2.getPosition().x;
//                    poseY = botpose2.getPosition().y;
//                }
//            }
//            else {
//                telemetry.addData("Limelight", "No Targets");
//            }



            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
//            telemetry.addData("Front left/Right", "%4.2f, %4.2f", frontLeftPower, frontRightPower);
//            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", backLeftPower, backRightPower);
            telemetry.addData(centric, "Centric");
            telemetry.addData("Yaw", yaw);
//            telemetry.addData("Motor Front Left Encoder Pos", motorFrontLeft.getCurrentPosition());

            telemetry.addData("Shooter LEFT Velocity", shooterLeftVelocity);
            telemetry.addData("Shooter RIGHT Velocity", shooterRightVelocity);
            telemetry.addData("Transfer Angle", transferAngle);



            telemetry.addData("Target X", tx);
            telemetry.addData("Target Y", ty);
            telemetry.addData("Target Area", ta);
            telemetry.addData("Pose X", poseX);
            telemetry.addData("Pose Y", poseY);

            panelsTelemetry.addData("Yaw", yaw);
            panelsTelemetry.addData("Field Centric", isFieldCentric);

            telemetry.update();
            panelsTelemetry.update();
            sleep(20);
        }


    }


    public void final_Orientation(){
        RevHubOrientationOnRobot orientationOnRobot =
                new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.RIGHT, RevHubOrientationOnRobot.UsbFacingDirection.UP);
        imu.initialize(new IMU.Parameters(orientationOnRobot));
    }

    public double cos(double degrees){
        return Math.cos(Math.toRadians(degrees));
    }
    public double sin(double degrees) {
        return Math.sin(Math.toRadians(degrees));
    }


}