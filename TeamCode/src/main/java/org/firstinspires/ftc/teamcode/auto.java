package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous
public class auto extends LinearOpMode {
    private Blinker control_Hub;
    private DcMotor arm;
    private Servo claw;
    private Servo intake;
    private DcMotor leftDrive;
    private DcMotor rightDrive;
    private DcMotor wrist;

    private void move_z(int power, int ms) {
        leftDrive.setPower(power);
        rightDrive.setPower(power);
        sleep(ms);
        leftDrive.setPower(0);
        rightDrive.setPower(0);
    }

    private void move_x(boolean left, int power, int ms) {
        if (left) {
            leftDrive.setPower(-power);
            rightDrive.setPower(power);
        } else {
            leftDrive.setPower(power);
            rightDrive.setPower(-power);
        }
        sleep(ms);
        leftDrive.setPower(0);
        rightDrive.setPower(0);
    }

    private void park(boolean left_p) {
        // hier aanpassen
        move_x(left_p, 1, 100);
        move_z(1, 100);
        move_x(left_p, 1, 100);
    }

    boolean isopen = true;

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
            telemetry.addData("is-open", isopen);

            telemetry.update();

        }

    }
}
