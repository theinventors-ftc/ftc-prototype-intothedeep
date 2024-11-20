package org.firstinspires.ftc.teamcode.AutonomousOPs;

import static org.firstinspires.ftc.teamcode.AutonomousOPs.BuilderFunctions.Tile;
import static org.firstinspires.ftc.teamcode.AutonomousOPs.BuilderFunctions.robotX;
import static org.firstinspires.ftc.teamcode.AutonomousOPs.BuilderFunctions.robotY;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.AutonomousOPs.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.AutonomousOPs.trajectorysequence.TrajectorySequenceBuilder;

@Autonomous(name = "Autonomous Left Red")
public class Auto_Op_Mode extends LinearOpMode {

    private SampleMecanumDrive drive;

    private TrajectorySequenceBuilder
        toNeutral_0,
        toNeutral_1,
        toNeutral_2,
        toBasket,
        toAscentZone;

    private volatile Pose2d current_pose;

    private Pose2d
        startPoseRedLeft = new Pose2d(
            -Tile + robotX/2, (-3 * Tile) + robotY/2, Math.toRadians(90)
        ),

        submersibleSide = new Pose2d(
            (-Tile) - robotY/2  , -Tile/2, Math.toRadians(0)
        ),

        basket = new Pose2d(
            (-2.5 * Tile) + robotX/2, -2 * Tile, Math.toRadians(225)
        ),

        neutralPixelLeft = new Pose2d(
            -2.5 * Tile, -1.5 * Tile, Math.toRadians(135)
        ),

        neutralPixelMid = new Pose2d(
            -2.5 * Tile, -1.5 * Tile, Math.toRadians(90)
        ),

        neutralPixelRight = new Pose2d(
            -2.5 * Tile, -1.5 * Tile, Math.toRadians(45)
        );

    public void init_toNeutral_0() {
        toNeutral_0 = drive.trajectorySequenceBuilder(startPoseRedLeft)
            .splineToLinearHeading(neutralPixelLeft, Math.toRadians(135));
    }
    public void init_toBasket() {
        toBasket = drive.trajectorySequenceBuilder(current_pose)
            .lineToLinearHeading(basket);
    }
    public void init_toNeutral_1() {
        toNeutral_1 = drive.trajectorySequenceBuilder(current_pose)
            .lineToLinearHeading(neutralPixelMid);
    }
    public void init_toNeutral_2() {
        toNeutral_2 = drive.trajectorySequenceBuilder(current_pose)
            .lineToLinearHeading(neutralPixelRight);
    }
    public void init_toAscentZone() {
        toAscentZone = drive.trajectorySequenceBuilder(current_pose)
            .setTangent(0)
            .splineToSplineHeading(submersibleSide, Math.toRadians(0));
    }

    @Override
    public void runOpMode() {
        drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(startPoseRedLeft);

        waitForStart();

        if(isStopRequested()) return;

        init_toNeutral_0();
        drive.followTrajectorySequenceAsync(toNeutral_0.build());
        while (!isStopRequested() && opModeIsActive() && drive.isBusy()) {
            drive.update();
        }
        drive.setWeightedDrivePower(new Pose2d(0, 0, 0));
        current_pose = drive.getPoseEstimate();


        init_toBasket();
        drive.followTrajectorySequenceAsync(toBasket.build());
        while (!isStopRequested() && opModeIsActive() && drive.isBusy()) {
            drive.update();
        }
        drive.setWeightedDrivePower(new Pose2d(0, 0, 0));
        current_pose = drive.getPoseEstimate();


        init_toNeutral_1();
        drive.followTrajectorySequenceAsync(toNeutral_1.build());
        while (!isStopRequested() && opModeIsActive() && drive.isBusy()) {
            drive.update();
        }
        drive.setWeightedDrivePower(new Pose2d(0, 0, 0));
        current_pose = drive.getPoseEstimate();


        init_toBasket();
        drive.followTrajectorySequenceAsync(toBasket.build());
        while (!isStopRequested() && opModeIsActive() && drive.isBusy()) {
            drive.update();
        }
        drive.setWeightedDrivePower(new Pose2d(0, 0, 0));
        current_pose = drive.getPoseEstimate();


        init_toNeutral_2();
        drive.followTrajectorySequenceAsync(toNeutral_2.build());
        while (!isStopRequested() && opModeIsActive() && drive.isBusy()) {
            drive.update();
        }
        drive.setWeightedDrivePower(new Pose2d(0, 0, 0));
        current_pose = drive.getPoseEstimate();


        init_toBasket();
        drive.followTrajectorySequenceAsync(toBasket.build());
        while (!isStopRequested() && opModeIsActive() && drive.isBusy()) {
            drive.update();
        }
        drive.setWeightedDrivePower(new Pose2d(0, 0, 0));
        current_pose = drive.getPoseEstimate();


        init_toAscentZone();
        drive.followTrajectorySequenceAsync(toAscentZone.build());
        while (!isStopRequested() && opModeIsActive() && drive.isBusy()) {
            drive.update();
        }
        drive.setWeightedDrivePower(new Pose2d(0, 0, 0));
        current_pose = drive.getPoseEstimate();

//        PoseStorage.currentPose = drive.getPoseEstimate();
    }
}