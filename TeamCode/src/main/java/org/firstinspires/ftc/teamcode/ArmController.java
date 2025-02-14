package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

// The state of the arm.
enum ArmState {
    MOVING,  // Transitioning to a new position.
    IDLE     // Not moving – holding position with a small oscillation.
}

public class ArmController {

    // The motor controlling the arm.
    private final DcMotor motor;

    // Conversion: normalized [0.0,1.0] to encoder ticks.
    private final int MAX_TICKS = 2800; // Adjust this value to your arm's encoder range.
    // Motor power used during moves.
    private final float MOVE_POWER = 0.6f;

    // Idle oscillation parameters.
    // idleAmplitude is in encoder ticks.
    private final int idleAmplitude = 5;
    private final float idleFrequency = 0.5f; // Hz

    // Positions (in encoder ticks)
    private int holdPosition;   // Position held during idle.
    private int targetPosition; // The commanded target position.
    private int startPosition;  // Position when a move begins.

    // Timing information for a move.
    private long moveStartTime;
    private long moveDuration;  // in milliseconds

    // Timing for idle oscillation.
    private long idleStartTime;

    private ArmState state;

    /**
     * Constructor.
     *
     * @param motor          The DC motor with an encoder powering the arm.
     * @param initialPosition The initial normalized position (0.0 to 1.0).
     */
    public ArmController(DcMotor motor, float initialPosition) {
        this.motor = motor;
        // Convert the normalized initial position to encoder ticks.
        this.holdPosition = (int)(initialPosition * MAX_TICKS);
        this.targetPosition = holdPosition;
        this.startPosition = holdPosition;
        this.state = ArmState.IDLE;
        this.idleStartTime = System.currentTimeMillis();

        // Configure the motor:
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setTargetPosition(holdPosition);
        motor.setPower(MOVE_POWER);
    }

    /**
     * Command the arm to move to a new normalized position (0.0 - 1.0).
     *
     * @param newTarget The target position as a float.
     */
    public void moveToPosition(float newTarget) {
        long now = System.currentTimeMillis();

        // Determine the current motor position.
        if (state == ArmState.MOVING) {
            // If already moving, get the current encoder reading.
            startPosition = motor.getCurrentPosition();
        } else if (state == ArmState.IDLE) {
            // When idle, include the current idle oscillation offset.
            float tIdle = (now - idleStartTime) / 1000.0f;
            startPosition = holdPosition + (int)(idleAmplitude * Math.sin(2 * Math.PI * idleFrequency * tIdle));
        }

        // Convert the normalized target to encoder ticks.
        targetPosition = (int)(newTarget * MAX_TICKS);
        moveStartTime = now;
        moveDuration = computeMoveDuration(startPosition, targetPosition);
        state = ArmState.MOVING;

        // Ensure the motor is in RUN_TO_POSITION mode and using the desired power.
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setPower(MOVE_POWER);
    }

    /**
     * Call this method repeatedly (e.g., in your OpMode loop) to update the arm’s movement.
     */
    public void update() {
        long now = System.currentTimeMillis();

        if (state == ArmState.MOVING) {
            float fraction = (float)(now - moveStartTime) / moveDuration;
            if (fraction >= 1.0f) {
                // Movement complete.
                holdPosition = targetPosition;
                motor.setTargetPosition(holdPosition);
                state = ArmState.IDLE;
                idleStartTime = now;
            } else {
                // Compute an interpolated target position.
                int newPos = startPosition + (int)(fraction * (targetPosition - startPosition));
                motor.setTargetPosition(newPos);
            }
        } else if (state == ArmState.IDLE) {
            // While idle, apply a small sine-wave oscillation around holdPosition.
            float tIdle = (now - idleStartTime) / 1000.0f; // time in seconds
            int idleOffset = (int)(idleAmplitude * Math.sin(2 * Math.PI * idleFrequency * tIdle));
            int idlePos = holdPosition + idleOffset;
            // Clamp idlePos between 0 and MAX_TICKS.
            idlePos = Math.max(0, Math.min(idlePos, MAX_TICKS));
            motor.setTargetPosition(idlePos);
        }
    }

    /**
     * Computes the move duration based on the distance between positions.
     * Adjust this method to match your hardware's acceleration/speed.
     *
     * @param start The starting encoder count.
     * @param end   The target encoder count.
     * @return Duration in milliseconds.
     */
    private long computeMoveDuration(int start, int end) {
        int distance = Math.abs(end - start);
        // For example: a full range move (MAX_TICKS ticks) takes 1000 ms.
        long duration = (long)(1000 * (distance / (float)MAX_TICKS));
        return Math.max(duration, 100); // Minimum duration of 100 ms.
    }

    /**
     * Getter for the current encoder position.
     *
     * @return The motor's current encoder count.
     */
    public int getCurrentPosition() {
        return motor.getCurrentPosition();
    }
}