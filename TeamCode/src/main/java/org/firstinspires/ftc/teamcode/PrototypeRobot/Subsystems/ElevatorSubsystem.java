package org.firstinspires.ftc.teamcode.PrototypeRobot.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.wpilibcontroller.ElevatorFeedforward;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.inventors.ftc.robotbase.hardware.MotorExEx;


import java.util.HashMap;
import java.util.function.DoubleSupplier;

public class ElevatorSubsystem extends SubsystemBase {
    public final MotorExEx motor;

    public double MAX_SPEED = 0.9;

    public enum Level {
        CLOSED, PARK, OPEN
    }

    private Level level;

    private boolean isAuto = false;

    private HashMap<Level, Integer> levels = new HashMap<Level, Integer>() {{
        put(Level.CLOSED, 0);
        put(Level.PARK, 200);
        put(Level.OPEN, 2400);
    }};

    private Telemetry telemetry;
    private DoubleSupplier leftY;

    public final double kS = 230, kG = 0, kV = 1.0, kA= 0.0;

    ElevatorFeedforward feedforward;

    public double joystickPower = 0.0, clippedPower = 0.0;

    public double feedForwardValue = 0.0;

    public double max_ticks_per_second = 0;


    public ElevatorSubsystem(HardwareMap hm, Telemetry telemetry, DoubleSupplier leftY) {

        this.leftY = leftY;
        this.telemetry = telemetry;
        motor = new MotorExEx(hm, "slider", 383.6, 312);
        motor.setMaxPower(MAX_SPEED);

        motor.setZeroPowerBehavior(MotorEx.ZeroPowerBehavior.BRAKE);
        motor.setPositionTolerance(30);
        motor.setPositionCoefficient(0.014);

        motor.resetEncoder();

        this.max_ticks_per_second = motor.ACHIEVABLE_MAX_TICKS_PER_SECOND;
        feedforward = new ElevatorFeedforward(
                kS, kG, kV, kA
        );
    }

    private double calculateFeedForwardPower(double rawPower) {
        feedForwardValue = feedforward.calculate(0.9 * rawPower * max_ticks_per_second);
        return (feedForwardValue / max_ticks_per_second) * MAX_SPEED;
    }

    @Override
    public void periodic() {
        joystickPower = leftY.getAsDouble();

        clippedPower = joystickPower;

        if(getHeight() < 0 && joystickPower < 0) clippedPower = 0.0;
        if(getHeight() > 2430 && joystickPower > 0) clippedPower = 0.0;

        if (Math.abs(joystickPower) > 0.06) {
            setManual();
        }

        if (!isAuto) {
            if (getHeight() < 100) setPower(clippedPower);
            else setPower(calculateFeedForwardPower(clippedPower));

            telemetry.addData("Lenght:", getHeight());
        }
    }

    public void run() {
        motor.set(MAX_SPEED);
    }

    public void stop() {
        motor.stopMotor();
    }

    public void setLevel(Level levelPicked) {
        motor.setTargetPosition((int)levels.get(levelPicked));
    }

    public void setManual() {
        motor.setRunMode(Motor.RunMode.RawPower);
        isAuto = false;
    }

    public void setAuto() {
        motor.setRunMode(Motor.RunMode.PositionControl);
        isAuto = true;
    }

    public void setPower(double power) {
        motor.set(power);
    }

    public int getHeight() {
        return motor.getCurrentPosition();
    }

    public Level getLevel() {
        return level;
    }

    public boolean atTargetLevel() {
        return motor.atTargetPosition();
    }

    public void reset() {
        motor.resetEncoder();
    }
}
