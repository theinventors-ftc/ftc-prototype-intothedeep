package org.firstinspires.ftc.teamcode.PrototypeRobot.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;

public class ClawSubsystem extends SubsystemBase {
    private static final double  OPEN_POS = 0.16, CLOSE_POS = 0.52;
    private ServoImplEx clawServo;

    public enum State {
        SECURED,
        RELEASED
    }

    private State state;
    public ClawSubsystem(HardwareMap hm) {
        clawServo = hm.get(ServoImplEx.class, "claw");
        this.release();
    }

    public void grab() {
        state = State.SECURED;
        clawServo.setPosition(CLOSE_POS);
    }

    public void release() {
        state = State.RELEASED;
        clawServo.setPosition(OPEN_POS);
    }

    public State getState() {
        return state;
    }
}
