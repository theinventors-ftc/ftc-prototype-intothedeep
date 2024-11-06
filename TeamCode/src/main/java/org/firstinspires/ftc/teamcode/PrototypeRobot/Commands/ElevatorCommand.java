package org.firstinspires.ftc.teamcode.PrototypeRobot.Commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.PrototypeRobot.Subsystems.ElevatorSubsystem;

public class ElevatorCommand extends CommandBase {
    private final ElevatorSubsystem elevator;
    private final ElevatorSubsystem.Level targetLevel;

    public ElevatorCommand(ElevatorSubsystem elevator, ElevatorSubsystem.Level levelPicked){
        this.elevator = elevator;
        targetLevel = levelPicked;
        addRequirements(this.elevator);
    }

    @Override
    public void initialize() {
        elevator.setAuto();
        elevator.setLevel(targetLevel);
    }

    @Override
    public void execute() {
        elevator.run();
    }

    @Override
    public void end(boolean interrupted) {
        elevator.stop();
        elevator.setManual();
    }

    @Override
    public boolean isFinished() {
        return elevator.atTargetLevel();
    }
}

