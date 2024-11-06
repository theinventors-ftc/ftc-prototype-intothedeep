package org.firstinspires.ftc.teamcode.PrototypeRobot.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;

public class ClawRotationSubsystem extends SubsystemBase {
    private static final double  LEFT_POS = 0.72, CENTER_POS = 0.4, RIGHT_POS = 0.12;
    private ServoImplEx clawRotServo;

    public enum State {
        LEFT,
        CENTER,
        RIGHT
    }

    private State state;
    public ClawRotationSubsystem(HardwareMap hm) {
        clawRotServo = hm.get(ServoImplEx.class, "claw_rot");
        this.go_center();
    }

    public void go_left() {
        state = State.LEFT;
        clawRotServo.setPosition(LEFT_POS);
    }

    public void go_center() {
        state = State.CENTER;
        clawRotServo.setPosition(CENTER_POS);
    }

    public void go_right() {
        state = State.RIGHT;
        clawRotServo.setPosition(RIGHT_POS);
    }

    public State getState() {
        return state;
    }
}
