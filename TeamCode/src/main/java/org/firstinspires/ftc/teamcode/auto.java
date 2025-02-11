package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous
public class auto extends LinearOpMode {
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
    float timer_intake = 0;
    boolean is_open = true;

    private void move_x(int mm, float left) {
        leftFrontDrive.setPower(-0.5 * left);
        rightFrontDrive.setPower(0.5 * left);
        leftBackDrive.setPower(0.5 * left);
        rightBackDrive.setPower(-0.5 * left);
        sleep(mm);
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);
    }

    private void move_y(int mm, float forward) {
        leftFrontDrive.setPower(0.5 * forward);
        rightFrontDrive.setPower(0.5 * forward);
        leftBackDrive.setPower(0.5 * forward);
        rightBackDrive.setPower(0.5 * forward);
        sleep(mm);
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);
    }

    private void turn(int mm, float left) {
        leftFrontDrive.setPower(0.5 * left);
        rightFrontDrive.setPower(-0.5 * left);
        leftBackDrive.setPower(0.5 * left);
        rightBackDrive.setPower(-0.5 * left);
        sleep(mm);
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);
    }

    private void arm(int targetpos) {
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

        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            telemetry.addData("Status", "Waiting for input");
            telemetry.addData("lx", gamepad1.left_stick_x);
            telemetry.addData("ly", gamepad1.left_stick_y);
            telemetry.addData("rx", gamepad1.right_stick_x);

            telemetry.addData("Arm-pos", arm.getCurrentPosition());
            telemetry.addData("Wrist-pos", wrist.getCurrentPosition());
            telemetry.addData("is-open_claw", is_open_claw);
            telemetry.addData("is-open_intake", is_open_intake);
            telemetry.addData("timer_intake", timer_intake);

            telemetry.update();

            arm(target);

            move_y(1000, 1);
            sleep(1000);
            move_x(250, 1);
            sleep(250);
            move_y(500, -1);
            sleep(500);
            turn(500, 1);
            sleep(30000 - 1000 + 250 + 500 + 500);
        }

    }
}
