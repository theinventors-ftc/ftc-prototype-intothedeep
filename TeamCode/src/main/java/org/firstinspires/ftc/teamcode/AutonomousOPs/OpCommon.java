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
        submersibleSideRed = new Pose2d(
            (-Tile) - robotY/2  , -Tile/2, Math.toRadians(0)
        ),
        submersibleAllianceRed = new Pose2d(
            0 , -Tile - robotY/2, Math.toRadians(90)
        ),
        basketRed = new Pose2d(
            (-2.5 * Tile) + robotX/2, -2 * Tile, Math.toRadians(225)
        ),

        startPoseRedLeft = new Pose2d(
            -Tile + robotX/2, (-3 * Tile) + robotY/2, Math.toRadians(90)
        ),
        neutralSampleLeftRed = new Pose2d(
            -2.5 * Tile, -1.5 * Tile, Math.toRadians(135)
        ),
        neutralSampleMidRed = new Pose2d(
            -2.5 * Tile, -1.5 * Tile, Math.toRadians(90)
        ),
        neutralSampleRightRed = new Pose2d(
            -2.5 * Tile, -1.5 * Tile, Math.toRadians(45)
        ),

        startPoseRedRight = new Pose2d(
            -Tile + robotX/2, (3 * Tile) + robotY/2, Math.toRadians(90)
        ),
        allianceSampleLeftRed = new Pose2d(
            -2.5 * Tile, 1.5 * Tile, Math.toRadians(135)
        ),
        allianceSampleMidRed = new Pose2d(
            -2.5 * Tile, 1.5 * Tile, Math.toRadians(90)
        ),
        allianceSampleRightRed = new Pose2d(
            -2.5 * Tile, 1.5 * Tile, Math.toRadians(45)
        ),

        /*- BLUE -*/
        submersibleSideBlue = new Pose2d(
            (-Tile) - robotY/2  , Tile/2, Math.toRadians(0)
        ),
        submersibleAllianceBlue = new Pose2d(
            0 , Tile + robotY/2, Math.toRadians(270)
        ),
        basketBlue = new Pose2d(
            (-2.5 * Tile) + robotX/2, 2 * Tile, Math.toRadians(225)
        ),

        startPoseBlueLeft = new Pose2d(
            -Tile + robotX/2, (3 * Tile) + robotY/2, Math.toRadians(90)
        ),
        neutralSampleLeftBlue = new Pose2d(
            -2.5 * Tile, 1.5 * Tile, Math.toRadians(135)
        ),
        neutralSampleMidBlue = new Pose2d(
            -2.5 * Tile, 1.5 * Tile, Math.toRadians(90)
        ),
        neutralSampleRightBlue = new Pose2d(
            -2.5 * Tile, 1.5 * Tile, Math.toRadians(45)
        ),

        startPoseBlueRight = new Pose2d(
             Tile + robotX/2, (3 * Tile) + robotY/2, Math.toRadians(90)
        ),
        allianceSampleLeftBlue = new Pose2d(
            2.5 * Tile, 1.5 * Tile, Math.toRadians(135)
        ),
        allianceSampleMidBlue = new Pose2d(
            2.5 * Tile, 1.5 * Tile, Math.toRadians(90)
        ),
        allianceSampleRightBlue = new Pose2d(
            2.5 * Tile, 1.5 * Tile, Math.toRadians(45)
        );

    /**
     * Commands and Subsystems for mechanisms
     */
    public static SequentialCommandGroup takeSample() {
        return new SequentialCommandGroup(
            new InstantCommand(tiltSubsystem::go_intake, tiltSubsystem),
            new WaitCommand(500),
            new ElevatorCommand(elevatorSubsystem, ElevatorSubsystem.Level.OPEN)
        );
    }

    public static SequentialCommandGroup grab() {
        return new SequentialCommandGroup(
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
        elevatorSubsystem = new ElevatorSubsystem(hm, tele, ()-> 0.0);
        tiltSubsystem = new TiltSubsystem(hm, tele, () -> elevatorSubsystem.getHeight());
        clawSubsystem = new ClawSubsystem(hm);
    }
}
