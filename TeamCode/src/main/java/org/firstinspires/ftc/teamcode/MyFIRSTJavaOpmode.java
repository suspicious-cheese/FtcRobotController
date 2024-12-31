package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class MyFIRSTJavaOpmode extends LinearOpMode {
    private Blinker control_Hub;
    private DcMotor arm;
    private Servo claw;
    private IMU imu;
    private CRServo intake;
    private DcMotor leftDrive;
    private DcMotor rightDrive;
    private DcMotor wrist;


    @Override
    public void runOpMode() {
        imu = hardwareMap.get(IMU.class, "imu");
        leftDrive = hardwareMap.get(DcMotor.class, "leftDrive");
        rightDrive = hardwareMap.get(DcMotor.class, "rightDrive");
        arm = hardwareMap.get(DcMotor.class, "arm");
        intake = hardwareMap.get(CRServo.class, "intake");
        claw = hardwareMap.get(Servo.class, "claw");

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            telemetry.addData("Status", "Cheese");
            telemetry.update();
            if (gamepad1.left_stick_y != 0) {
                leftDrive.setPower(gamepad1.left_stick_y);
                rightDrive.setPower(gamepad1.left_stick_y);
            } else if (gamepad1.right_stick_x != 0) {
                leftDrive.setPower(-gamepad1.right_stick_x);
                rightDrive.setPower(gamepad1.right_stick_x);
            } else if (gamepad1.right_trigger != 0) {
                intake.setPower(1);
            } else {
                leftDrive.setPower(0);
                rightDrive.setPower(0);
                intake.setPower(0);
                claw.setPosition(0);
            }
        }

    }
}
