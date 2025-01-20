package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class MyFIRSTJavaOpmode extends LinearOpMode {
    private Blinker control_Hub;
    private DcMotor arm;
    private Servo claw;
    private Servo intake;
    private DcMotor leftDrive;
    private DcMotor rightDrive;
    private DcMotor wrist;

    int armpos = 0;
    int targetpos = 0;

    boolean is_open_claw = true;

    boolean is_open_intake = false;
    float timer_intake = 0;

    @Override
    public void runOpMode() {
        leftDrive = hardwareMap.get(DcMotor.class, "leftDrive");
        rightDrive = hardwareMap.get(DcMotor.class, "rightDrive");
        arm = hardwareMap.get(DcMotor.class, "arm");
        intake = hardwareMap.get(Servo.class, "intake");
        claw = hardwareMap.get(Servo.class, "claw");
        wrist = hardwareMap.get(DcMotor.class, "wrist");

        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        wrist.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            telemetry.addData("Status", "Waiting for input");
            telemetry.addData("ly", gamepad1.left_stick_y);
            telemetry.addData("rx", gamepad1.right_stick_x);

            telemetry.addData("Arm-pos", arm.getCurrentPosition());
            telemetry.addData("Wrist-pos", wrist.getCurrentPosition());
            telemetry.addData("is-open_claw", is_open_claw);
            telemetry.addData("is-open_intake", is_open_intake);
            telemetry.addData("timer_intake", timer_intake);


            telemetry.update();

            armpos = arm.getCurrentPosition();

            if (gamepad1.left_stick_y != 0) {
                leftDrive.setPower(gamepad1.left_stick_y);
                rightDrive.setPower(gamepad1.left_stick_y);
            } else if (gamepad1.right_stick_x != 0) {
                leftDrive.setPower(-gamepad1.right_stick_x);
                rightDrive.setPower(gamepad1.right_stick_x);
            } else {
                leftDrive.setPower(0);
                rightDrive.setPower(0);
            }

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
