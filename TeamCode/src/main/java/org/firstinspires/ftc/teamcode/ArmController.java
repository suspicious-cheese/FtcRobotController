package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

public class ArmController {
    private final DcMotor arm;
    public ArmController(DcMotor arm, int initialPosition) {
        this.arm = arm;
        this.arm.setTargetPosition(initialPosition);
    }

    public void moveToPosition(int pos) {
        this.arm.setTargetPosition(pos);
    }
}
