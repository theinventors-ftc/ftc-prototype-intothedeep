package org.firstinspires.ftc.teamcode.PrototypeRobot;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.PrototypeRobot.Commands.ElevatorCommand;
import org.firstinspires.ftc.teamcode.PrototypeRobot.Subsystems.ClawRotationSubsystem;
import org.firstinspires.ftc.teamcode.PrototypeRobot.Subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.PrototypeRobot.Subsystems.ElevatorSubsystem;
import org.firstinspires.ftc.teamcode.PrototypeRobot.Subsystems.IntakeSubsystem;
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

    protected IntakeSubsystem intakeSubsystem;

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
        tiltSubsystem = new TiltSubsystem(hardwareMap, telemetry, () -> elevatorSubsystem.getHeight());

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
                new InstantCommand(tiltSubsystem::go_intake, tiltSubsystem),
                new InstantCommand(tiltSubsystem::go_park, tiltSubsystem)
        );

        toolOp.getGamepadButton(GamepadKeys.Button.RIGHT_STICK_BUTTON).whenPressed(
                new InstantCommand(tiltSubsystem::go_second_basket, tiltSubsystem)
        );

        driverOp.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(
                new InstantCommand(tiltSubsystem::go_intake, tiltSubsystem)
        ).whenReleased(
                new InstantCommand(tiltSubsystem::go_over_intake, tiltSubsystem)
        );

        toolOp.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(
                new SequentialCommandGroup(
                        new InstantCommand(tiltSubsystem::go_intake, tiltSubsystem),
                        new WaitCommand(400),
                        new InstantCommand(clawSubsystem::grab, clawSubsystem),
                        new WaitCommand(600),
                        new InstantCommand(tiltSubsystem::go_park, tiltSubsystem),
                        new WaitCommand(500),
                        new InstantCommand(clawRotationSubsystem::go_center, clawRotationSubsystem),
                        new ElevatorCommand(elevatorSubsystem, ElevatorSubsystem.Level.CLOSED)
                )
        );

        toolOp.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenPressed(
                new SequentialCommandGroup(
                        new InstantCommand(tiltSubsystem::go_over_intake, tiltSubsystem),
                        new InstantCommand(clawRotationSubsystem::go_center, clawRotationSubsystem),
                        new WaitCommand(600),
                        new InstantCommand(clawSubsystem::release, clawSubsystem),
                        new WaitCommand(1200),
                        new ElevatorCommand(elevatorSubsystem, ElevatorSubsystem.Level.OPEN)
                )
        );

        new Trigger(() -> toolOp.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.5).whenActive(
                new SequentialCommandGroup(
                        new InstantCommand(clawSubsystem::release, clawSubsystem),
                        new WaitCommand(600),
                        new InstantCommand(clawRotationSubsystem::go_center, clawRotationSubsystem),
                        new ElevatorCommand(elevatorSubsystem, ElevatorSubsystem.Level.CLOSED),
                        new WaitCommand(400),
                        new InstantCommand(tiltSubsystem::go_park, tiltSubsystem)
                )
        );
    }
}
