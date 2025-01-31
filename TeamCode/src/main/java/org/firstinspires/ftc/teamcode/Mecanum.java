package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class Mecanum extends LinearOpMode {
    private DcMotor arm;
    private DcMotor wrist;
    private Servo claw;
    private Servo intake;
    private DcMotor leftFrontDrive;
    private DcMotor rightFrontDrive;
    private DcMotor leftBackDrive;
    private DcMotor rightBackDrive;

    int armpos = 0;
    int targetpos = 0;

    boolean is_open_claw = true;
    boolean is_open_intake = false;

    @Override
    public void runOpMode() {
        leftFrontDrive = hardwareMap.get(DcMotor.class, "leftFrontDrive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "rightFrontDrive");
        leftBackDrive = hardwareMap.get(DcMotor.class, "leftBackDrive");  // FIXED
        rightBackDrive = hardwareMap.get(DcMotor.class, "rightBackDrive"); // FIXED
        arm = hardwareMap.get(DcMotor.class, "arm");
        intake = hardwareMap.get(Servo.class, "intake");
        claw = hardwareMap.get(Servo.class, "claw");
        wrist = hardwareMap.get(DcMotor.class, "wrist");

        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        wrist.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            //  Read joystick inputs
            double drive = -gamepad1.left_stick_y;  // Forward/Backward
            double strafe = gamepad1.left_stick_x;  // Left/Right
            double rotate = gamepad1.right_stick_x; // Rotation

            //  Mecanum drive calculations
            double frontLeftPower = drive + strafe + rotate;
            double frontRightPower = drive - strafe - rotate;
            double backLeftPower = drive - strafe + rotate;
            double backRightPower = drive + strafe - rotate;

            //  Normalize values if any exceed 1.0
            double maxPower = Math.max(1.0, Math.max(
                    Math.abs(frontLeftPower),
                    Math.max(Math.abs(frontRightPower),
                            Math.max(Math.abs(backLeftPower), Math.abs(backRightPower)))
            ));

            leftFrontDrive.setPower(frontLeftPower / maxPower);
            rightFrontDrive.setPower(frontRightPower / maxPower);
            leftBackDrive.setPower(backLeftPower / maxPower);
            rightBackDrive.setPower(backRightPower / maxPower);

            if (gamepad2.square) {
                if (is_open_claw){
                    claw.setPosition(0.515);
                    is_open_claw = false;
                }
            }else if(gamepad2.cross) {
                if (!is_open_claw){
                    claw.setPosition(0.4);
                    is_open_claw = true;
                }
            }

            if (gamepad2.triangle) {
                //if (timer_intake <= 0) {
                //    if (is_open_intake) {
                //        intake.setPosition(1);
                //        is_open_intake = false;
                //        timer_intake = 2.5F;
                //    } else {
                //        intake.setPosition(0);
                //        is_open_intake = true;
                //        timer_intake = 2.5F;
                //    }
                //}
            //}
                if (is_open_intake) {
                    gamepad1.rumble(500);
                    intake.setPosition(1);
                    is_open_intake = false;
                }
            } else if(gamepad2.circle) {
                if (!is_open_intake) {
                    intake.setPosition(0);
                    is_open_intake = true;
                }
            }

            //if (timer_intake > 0) {
            //    timer_intake -= 0.01F;
            //}

            if (gamepad2.dpad_up) {
                arm.setPower(0.6);
                targetpos = 0;
            } else if (gamepad2.dpad_down) {
                if (armpos > 200) {
                    arm.setPower(-0.6);
                }
                targetpos = 0;
            } else {
                if (targetpos == 0) {
                    targetpos = armpos;
                }
                if (targetpos + 15 < armpos) {
                    arm.setPower(-0.1);
                } else if (targetpos - 15 > armpos){
                    arm.setPower(0.1);
                } else {
                    arm.setPower(0);
                }
            }


            if (gamepad2.dpad_left) {
                wrist.setPower(0.6);
            } else if (gamepad2.dpad_right) {
                wrist.setPower(-0.6);
            } else {
                wrist.setPower(0);
            }


        }

    }
}
