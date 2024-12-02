package org.firstinspires.ftc.teamcode.PrototypeRobot.Commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.PrototypeRobot.Subsystems.IntakeSubsystem;

public class IntakeCommand1 extends CommandBase {
    private IntakeSubsystem intakeSubsystem;
    public IntakeCommand1(IntakeSubsystem intakeSubsystem) {
        this.intakeSubsystem = intakeSubsystem;
        addRequirements(this.intakeSubsystem);
    }

    @Override
    public void initialize() {
        intakeSubsystem.run();
    }

    @Override
    public void end(boolean interrupted) {
        intakeSubsystem.stop();
    }

    @Override
    public boolean isFinished() {
        return intakeSubsystem.isSample() && intakeSubsystem.getSampleColor() != IntakeSubsystem.COLOR.BLUE && intakeSubsystem.getSampleColor() != IntakeSubsystem.COLOR.NONE;
    }
}
