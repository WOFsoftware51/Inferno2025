package org.firstinspires.ftc.teamcode.Autons;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;



@Autonomous(name = "JustShoot")
public class JustShoot extends LinearOpMode {
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

    double timer;
    State currentState = State.START;
    double shooterLimit = -2450;
    double shooterRightVelocity;
    int where;

    @Override
    public void runOpMode() {

        // Initialize hardware
        motorShooterLeft = (DcMotorEx) hardwareMap.get(DcMotor.class, "motorShooterLeft");
        motorShooterRight = (DcMotorEx) hardwareMap.get(DcMotor.class, "motorShooterRight");
        motorTransfer = (DcMotorEx) hardwareMap.get(DcMotor.class, "motorTransfer");

        timer = 0;
        where = 0;

        waitForStart();

        if (opModeIsActive()) {
            while (opModeIsActive()) {
                telemetry.addData("Status", "Initialized");
                telemetry.addData("Where", where);
                telemetry.addData("Current State", currentState);
                telemetry.addData("Right Velocity", shooterRightVelocity);
                shooterRightVelocity = ((motorShooterRight.getVelocity() / 28) * 60);
                telemetry.update();

                switch (currentState) {
                    case START:
                        motorShooterLeft.setPower(0);
                        motorShooterRight.setPower(0);
                        motorTransfer.setPower(0.0);
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
                        if (timer > 500) {
                            timer = 0;
                            currentState = State.RAMPDOWN;
                        }
                        break;
                    case RAMPDOWN:
                        motorShooterLeft.setPower(0.0);
                        motorShooterRight.setPower(0.0);
                        motorTransfer.setPower(0.3);
                        timer++;
                        if (timer > 500) {
                            timer = 0;
                            currentState = State.STOP;
                        }
                        break;
                    case STOP:
                        motorShooterLeft.setPower(0.0);
                        motorShooterRight.setPower(0.0);
                        motorTransfer.setPower(0.0);
                        break;
                    default:
                        currentState = State.STOP;
                        break;
                }
            }

            sleep(200);

        }
    }
}
