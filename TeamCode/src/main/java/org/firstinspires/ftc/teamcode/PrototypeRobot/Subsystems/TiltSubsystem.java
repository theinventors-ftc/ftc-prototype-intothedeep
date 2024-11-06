package org.firstinspires.ftc.teamcode.PrototypeRobot.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.TeleOPs.TeleOpBase;
import org.inventors.ftc.robotbase.hardware.MotorExEx;

import java.util.HashMap;
import java.util.logging.Handler;

public class TiltSubsystem extends SubsystemBase {
    public final MotorExEx motor;
    public Telemetry telemetry;
    public double MAX_SPEED = 0.5;

    public enum Level {
        START, PARK, INTAKE, FIRST_BASKET, SECOND_BASKET
    }

    private Level level;

    private HashMap<TiltSubsystem.Level, Integer> levels = new HashMap<TiltSubsystem.Level, Integer>() {{
        put(TiltSubsystem.Level.START, 0);
        put(TiltSubsystem.Level.PARK, 150);
        put(TiltSubsystem.Level.INTAKE, 45);
        put(TiltSubsystem.Level.FIRST_BASKET, 390);
        put(TiltSubsystem.Level.SECOND_BASKET, 540);
    }};

    public TiltSubsystem(HardwareMap hm, Telemetry telemetry) {
        this.motor = new MotorExEx(hm, "tilt", 383.6, 312);
        this.telemetry = telemetry;

        motor.setMaxPower(MAX_SPEED);
        motor.setInverted(false);
        motor.setZeroPowerBehavior(MotorEx.ZeroPowerBehavior.BRAKE);
        motor.setPositionTolerance(30);
        motor.setPositionCoefficient(0.06);
        motor.resetEncoder();
        motor.setRunMode(Motor.RunMode.PositionControl);

        level = Level.START;
    }

    @Override
    public void periodic() {
        telemetry.addData("Tilt: ", getPosition());

        this.run();
    }

    public void run() {
        motor.set(MAX_SPEED);
    }

    public void stop() {
        motor.stopMotor();
    }

    public void setLevel(TiltSubsystem.Level levelPicked) {
        motor.setTargetPosition((int)levels.get(levelPicked));
    }

    public void go_intake() {
        motor.setTargetPosition((int)levels.get(TiltSubsystem.Level.INTAKE));
    }

    public void go_park() {
        motor.setTargetPosition((int)levels.get(TiltSubsystem.Level.PARK));
    }

    public void go_first_basket() {
        motor.setTargetPosition((int)levels.get(TiltSubsystem.Level.FIRST_BASKET));
    }

    public void go_second_basket() {
        motor.setTargetPosition((int)levels.get(TiltSubsystem.Level.SECOND_BASKET));
    }

    public void go_start() {
        motor.setTargetPosition((int)levels.get(TiltSubsystem.Level.START));
    }

    public void setPower(double power) {
        motor.set(power);
    }

    public int getPosition() {
        return motor.getCurrentPosition();
    }

    public TiltSubsystem.Level getLevel() {
        return level;
    }
    public boolean atTargetLevel() {
        return motor.atTargetPosition();
    }
    public void reset() {
        motor.resetEncoder();
    }
}
