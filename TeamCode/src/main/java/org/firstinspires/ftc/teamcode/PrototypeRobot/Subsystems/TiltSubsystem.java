package org.firstinspires.ftc.teamcode.PrototypeRobot.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PDController;
import com.arcrobotics.ftclib.controller.wpilibcontroller.ArmFeedforward;
import com.arcrobotics.ftclib.controller.wpilibcontroller.ElevatorFeedforward;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.TeleOPs.TeleOpBase;
import org.inventors.ftc.robotbase.controllers.PIDFControllerEx;
import org.inventors.ftc.robotbase.hardware.MotorExEx;

import java.util.HashMap;
import java.util.function.DoubleSupplier;
import java.util.logging.Handler;

public class TiltSubsystem extends SubsystemBase {
    public final MotorExEx motor, motor2;
    public final MotorGroup motors;
    private PIDFControllerEx controller;
    public Telemetry telemetry;
    public double MAX_SPEED = 0.5;

    public enum Level {
        START, PARK, INTAKE, OVER_INTAKE, FIRST_BASKET, SECOND_BASKET
    }

    private Level level;

    private HashMap<TiltSubsystem.Level, Integer> levels = new HashMap<TiltSubsystem.Level, Integer>() {{
        put(TiltSubsystem.Level.START, 0);
        put(TiltSubsystem.Level.PARK, 70);
        put(TiltSubsystem.Level.INTAKE, 0);
        put(TiltSubsystem.Level.OVER_INTAKE, 30);
        put(TiltSubsystem.Level.FIRST_BASKET, 390);
        put(TiltSubsystem.Level.SECOND_BASKET, 460);
    }};


    private DoubleSupplier extent;

    public double clip(double x, double min, double max) {
        return Math.min(Math.max(x, min), max);
    }

    public double ff_calculate(double input, double angle) {
        return input*Math.cos(angle);
    }

    public TiltSubsystem(HardwareMap hm, Telemetry telemetry, DoubleSupplier extent) {
        this.motor = new MotorExEx(hm, "tilt", 383.6, 312);
        this.motor2 = new MotorExEx(hm, "tilt2", 383.6, 312);
        this.telemetry = telemetry;

        motor.setMaxPower(MAX_SPEED);
        motor2.setMaxPower(MAX_SPEED);
        motor.setInverted(false);
        motor2.setInverted(false);
        motor.setZeroPowerBehavior(MotorEx.ZeroPowerBehavior.BRAKE);
        motor2.setZeroPowerBehavior(MotorEx.ZeroPowerBehavior.BRAKE);

        motors = new MotorGroup(motor, motor2);


        controller = new PIDFControllerEx(0.03, 0.0, 0.000, 0.0, 0.2, 0, 0, 0);


//        motor.setPositionTolerance(30);
//        motor.setPositionCoefficient(0.06);
        motors.resetEncoder();
        motors.setRunMode(Motor.RunMode.RawPower);

        level = Level.START;

        this.extent = extent;
    }

    double off = 0.32;

    @Override
    public void periodic() {
        telemetry.addData("Tilt: ", getPosition());
        telemetry.addData("Tilt Angles: ", getArmAngle());
        telemetry.addData("FF Output: ", ff_calculate(0.22, Math.toRadians(getArmAngle())));

//        motors.set(0.35 + clip(controller.calculate(getPosition()), -0.34, 0.3)); // down, up // for closed extendo
//        motors.set(0.35 + clip(controller.calculate(getPosition()), -0.2, 0.3)); // down, up // for opened extendo
//        motors.set(
//            clip(
//                    controller.calculate(getPosition()),
//                    mapping(extent.getAsDouble(), 0, 2400, 0, 0.3),
//                    mapping(extent.getAsDouble(), 0, 2400, 0, 0.6)
//            )
//        ); // down, up
        motors.set(
                ff_calculate(mapping(extent.getAsDouble(), 0,2400,0.22, 0.36), Math.toRadians(getArmAngle())) +
                clip(controller.calculate(getPosition()), -0.2, 0.2));
    }

    public void run() {
        motors.set(MAX_SPEED);
    }

    public void stop() {
        motors.stopMotor();
    }

    public void setLevel(TiltSubsystem.Level levelPicked) {
//        motor.setTargetPosition((int)levels.get(levelPicked));
        controller.setSetPoint((int)levels.get(levelPicked));
    }

    public void go_intake() {
//        motor.setTargetPosition((int)levels.get(TiltSubsystem.Level.INTAKE));
        controller.setSetPoint((int)levels.get(TiltSubsystem.Level.INTAKE));
    }

    public void go_over_intake() {
//        motor.setTargetPosition((int)levels.get(TiltSubsystem.Level.INTAKE));
        controller.setSetPoint((int)levels.get(TiltSubsystem.Level.OVER_INTAKE));
    }

    public void go_park() {
//        motor.setTargetPosition((int)levels.get(TiltSubsystem.Level.PARK));
        controller.setSetPoint((int)levels.get(TiltSubsystem.Level.PARK));
    }

    public void go_first_basket() {
//        motor.setTargetPosition((int)levels.get(TiltSubsystem.Level.FIRST_BASKET));
        controller.setSetPoint((int)levels.get(TiltSubsystem.Level.FIRST_BASKET));
    }

    public void go_second_basket() {
//        motor.setTargetPosition((int)levels.get(TiltSubsystem.Level.SECOND_BASKET));
        controller.setSetPoint((int)levels.get(TiltSubsystem.Level.SECOND_BASKET));
    }

    public void go_start() {
//        motor.setTargetPosition((int)levels.get(TiltSubsystem.Level.START));
        controller.setSetPoint((int)levels.get(TiltSubsystem.Level.START));
    }

    public void setPower(double power) {
        motors.set(power);
    }

    public int getPosition() {
        return motor.getCurrentPosition();
    }

    public double mapping(double x, double in_min, double in_max, double out_min, double out_max) {
        return (x-in_min) * (out_max-out_min) / (in_max - in_min) + out_min;
    }

    public double getArmAngle() {
        return mapping(this.getPosition(), 90, 635, 0, 90);
    }

    public TiltSubsystem.Level getLevel() {
        return level;
    }
    public boolean atTargetLevel() {
//        return motor.atTargetPosition();
        return Math.abs(controller.getPositionError()) < 5;
    }
    public void reset() {
        motor.resetEncoder();
    }
}
