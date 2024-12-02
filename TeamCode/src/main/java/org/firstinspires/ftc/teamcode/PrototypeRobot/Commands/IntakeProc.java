package org.firstinspires.ftc.teamcode.PrototypeRobot.Commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.PrototypeRobot.Subsystems.IntakeSubsystem;

public class IntakeProc extends SequentialCommandGroup {
    private IntakeSubsystem intakeSubsystem;

    public IntakeProc(IntakeSubsystem intakeSubsystem) {
        this.intakeSubsystem = intakeSubsystem;;
        addCommands(
                new IntakeCommand1(intakeSubsystem),
                new InstantCommand(intakeSubsystem::run, intakeSubsystem),
                new InstantCommand(intakeSubsystem::brake_reverse, intakeSubsystem),
                new WaitCommand(100),
                new InstantCommand(intakeSubsystem::stop, intakeSubsystem)
        );
    }
}
