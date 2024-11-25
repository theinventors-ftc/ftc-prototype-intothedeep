package org.firstinspires.ftc.teamcode.AutonomousOPs;

import static org.firstinspires.ftc.teamcode.AutonomousOPs.Features.BuilderFunctions.Tile;
import static org.firstinspires.ftc.teamcode.AutonomousOPs.Features.BuilderFunctions.robotX;
import static org.firstinspires.ftc.teamcode.AutonomousOPs.Features.BuilderFunctions.robotY;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.AutonomousOPs.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.AutonomousOPs.trajectorysequence.TrajectorySequenceBuilder;

@Autonomous(name = "Autonomous Left Red")
public class Auto_Op_Mode_Command extends CommandOpMode {

    private SampleMecanumDrive drive;

    private TrajectorySequenceBuilder
        toNeutral_0;

    private Pose2d
        startPoseRedLeft = new Pose2d(
            -Tile + robotX/2, (-3 * Tile) + robotY/2, Math.toRadians(90)
        ),

        neutralPixelLeft = new Pose2d(
            -2.5 * Tile, -1.5 * Tile, Math.toRadians(135)
        );

    public void init_toNeutral_0() {
        toNeutral_0 = drive.trajectorySequenceBuilder(startPoseRedLeft)
            .splineToLinearHeading(neutralPixelLeft, Math.toRadians(135));
    }

    @Override
    public void initialize(){
        drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(startPoseRedLeft);
    }

    @Override
    public void run() {
        isStopRequested();

        init_toNeutral_0();
        drive.followTrajectorySequenceAsync(toNeutral_0.build());
        while (!isStopRequested() && opModeIsActive() && drive.isBusy()) {
            drive.update();
        }
        drive.setWeightedDrivePower(new Pose2d(0, 0, 0));

//        PoseStorage.currentPose = drive.getPoseEstimate();
    }
}