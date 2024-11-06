package org.firstinspires.ftc.teamcode.PrototypeRobot.Commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.PrototypeRobot.Subsystems.TiltSubsystem;

public class TiltCommand extends CommandBase {
    private final TiltSubsystem tiltSubsystem;
    private final TiltSubsystem.Level targetLevel;

    public TiltCommand(TiltSubsystem tiltSubsystem, TiltSubsystem.Level levelPicked){
        this.tiltSubsystem = tiltSubsystem;
        targetLevel = levelPicked;
        addRequirements(this.tiltSubsystem);
    }

    @Override
    public void initialize() {
        tiltSubsystem.setLevel(targetLevel);
    }

    @Override
    public void execute() {
        tiltSubsystem.run();
    }

    @Override
    public void end(boolean interrupted) {
        tiltSubsystem.stop();
    }

    @Override
    public boolean isFinished() {
        return tiltSubsystem.atTargetLevel();
    }
}

