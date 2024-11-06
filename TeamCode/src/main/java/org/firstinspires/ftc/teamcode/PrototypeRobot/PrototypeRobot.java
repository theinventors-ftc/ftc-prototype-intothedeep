package org.firstinspires.ftc.teamcode.PrototypeRobot;

import android.app.TaskInfo;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.PrototypeRobot.Commands.ElevatorCommand;
import org.firstinspires.ftc.teamcode.PrototypeRobot.Commands.TiltCommand;
import org.firstinspires.ftc.teamcode.PrototypeRobot.Subsystems.ClawRotationSubsystem;
import org.firstinspires.ftc.teamcode.PrototypeRobot.Subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.PrototypeRobot.Subsystems.ElevatorSubsystem;
import org.firstinspires.ftc.teamcode.PrototypeRobot.Subsystems.TiltSubsystem;
import org.inventors.ftc.robotbase.RobotEx;
import org.inventors.ftc.robotbase.drive.DriveConstants;
import org.inventors.ftc.robotbase.hardware.GamepadExEx;

public class PrototypeRobot extends RobotEx {
    //----------------------------------- Initialize Subsystems ----------------------------------//
    protected ClawSubsystem clawSubsystem;
    protected ClawRotationSubsystem clawRotationSubsystem;
    protected ElevatorSubsystem elevatorSubsystem;
    protected TiltSubsystem tiltSubsystem;
    public PrototypeRobot(HardwareMap hm, DriveConstants RobotConstants, Telemetry telemetry,
                            GamepadExEx driverOp, GamepadExEx toolOp, OpModeType opModeType,
                            Alliance alliance, String imuName, boolean camera, Pose2d startingPose) {
        super(hm, RobotConstants, telemetry, driverOp, toolOp, opModeType, alliance, imuName, camera,
                false, startingPose);
    }

    @Override
    public void initMechanismsAutonomous(HardwareMap hardwareMap) {
        super.initMechanismsAutonomous(hardwareMap);
    }

    @Override
    public void initMechanismsTeleOp(HardwareMap hardwareMap) {
        clawSubsystem = new ClawSubsystem(hardwareMap);
        clawRotationSubsystem = new ClawRotationSubsystem(hardwareMap);
        elevatorSubsystem = new ElevatorSubsystem(hardwareMap, telemetry, () -> toolOp.getLeftY());
        tiltSubsystem = new TiltSubsystem(hardwareMap, telemetry);

        toolOp.getGamepadButton(GamepadKeys.Button.A).toggleWhenPressed(
                new InstantCommand(clawSubsystem::grab, clawRotationSubsystem),
                new InstantCommand(clawSubsystem::release, clawRotationSubsystem)
        );

        toolOp.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).whenPressed(
                new InstantCommand(clawRotationSubsystem::go_left, clawRotationSubsystem)
        );

        toolOp.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(
                new InstantCommand(clawRotationSubsystem::go_center, clawRotationSubsystem)
        );

        toolOp.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT).whenPressed(
                new InstantCommand(clawRotationSubsystem::go_right, clawRotationSubsystem)
        );

        toolOp.getGamepadButton(GamepadKeys.Button.Y).toggleWhenPressed(
                new ElevatorCommand(elevatorSubsystem, ElevatorSubsystem.Level.CLOSED),
                new ElevatorCommand(elevatorSubsystem, ElevatorSubsystem.Level.OPEN)
        );

        toolOp.getGamepadButton(GamepadKeys.Button.B).toggleWhenPressed(
//                new TiltCommand(tiltSubsystem, TiltSubsystem.Level.PARK),
//                new TiltCommand(tiltSubsystem, TiltSubsystem.Level.INTAKE)
                new InstantCommand(tiltSubsystem::go_intake, tiltSubsystem),
                new InstantCommand(tiltSubsystem::go_park, tiltSubsystem)
        );

        toolOp.getGamepadButton(GamepadKeys.Button.RIGHT_STICK_BUTTON).whenPressed(
//                new TiltCommand(tiltSubsystem, TiltSubsystem.Level.PARK),
//                new TiltCommand(tiltSubsystem, TiltSubsystem.Level.INTAKE)
                new InstantCommand(tiltSubsystem::go_second_basket, tiltSubsystem)
        );
    }
}
