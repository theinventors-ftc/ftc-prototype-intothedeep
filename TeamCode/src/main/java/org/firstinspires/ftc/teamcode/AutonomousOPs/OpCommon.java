package org.firstinspires.ftc.teamcode.AutonomousOPs;

import static org.firstinspires.ftc.teamcode.AutonomousOPs.Features.BuilderFunctions.Tile;
import static org.firstinspires.ftc.teamcode.AutonomousOPs.Features.BuilderFunctions.robotX;
import static org.firstinspires.ftc.teamcode.AutonomousOPs.Features.BuilderFunctions.robotY;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.PrototypeRobot.Commands.ElevatorCommand;
import org.firstinspires.ftc.teamcode.PrototypeRobot.Subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.PrototypeRobot.Subsystems.ElevatorSubsystem;
import org.firstinspires.ftc.teamcode.PrototypeRobot.Subsystems.TiltSubsystem;

import java.util.function.DoubleSupplier;

public class OpCommon extends CommandBase {

    public static TiltSubsystem tiltSubsystem;
    public static ElevatorSubsystem elevatorSubsystem;
    public static ClawSubsystem clawSubsystem;
    public static SequentialCommandGroup temp;

    /**
     *  Alliances Poses
     */
    public static final Pose2d
        /*- RED -*/
        startPoseRedLeft = new Pose2d(
            -Tile + robotX/2, (-3 * Tile) + robotY/2, Math.toRadians(90)
        ),

        submersibleSideRed = new Pose2d(
            (-Tile) - robotY/2  , -Tile/2, Math.toRadians(0)
        ),

        basketRed = new Pose2d(
            (-2.5 * Tile) + robotX/2, -2 * Tile, Math.toRadians(225)
        ),

        neutralSampleLeftRed = new Pose2d(
            -2.5 * Tile, -1.5 * Tile, Math.toRadians(135)
        ),

        neutralSampleMidRed = new Pose2d(
            -2.5 * Tile, -1.5 * Tile, Math.toRadians(90)
        ),

        neutralSampleRightRed = new Pose2d(
            -2.5 * Tile, -1.5 * Tile, Math.toRadians(45)
        );

    /**
     * Commands and Subsystems for mechanisms
     */
    public static SequentialCommandGroup takeSample() {
        return new SequentialCommandGroup(
            new InstantCommand(tiltSubsystem::go_intake, tiltSubsystem),
            new WaitCommand(500),
            new ElevatorCommand(elevatorSubsystem, ElevatorSubsystem.Level.OPEN),
            new InstantCommand(clawSubsystem::grab, clawSubsystem),
            new ElevatorCommand(elevatorSubsystem, ElevatorSubsystem.Level.CLOSED)
        );
    }

    public static SequentialCommandGroup scoreBasket() {
        return new SequentialCommandGroup(
            new InstantCommand(tiltSubsystem::go_second_basket, tiltSubsystem),
            new WaitCommand(500),
            new ElevatorCommand(elevatorSubsystem, ElevatorSubsystem.Level.OPEN)
        );
    }

    public static SequentialCommandGroup release() {
        return new SequentialCommandGroup(
            new InstantCommand(clawSubsystem::release, clawSubsystem),
            new ElevatorCommand(elevatorSubsystem, ElevatorSubsystem.Level.CLOSED),
            new InstantCommand(tiltSubsystem::go_park, tiltSubsystem)
        );
    }

    /**
     * Initialization of all subsystems and mechanisms
     */
    public static void init_mechanisms(HardwareMap hm, Telemetry tele) {
        tiltSubsystem = new TiltSubsystem(hm, tele);
        elevatorSubsystem = new ElevatorSubsystem(hm, tele, ()-> 0.0);
        clawSubsystem = new ClawSubsystem(hm);
    }
}
