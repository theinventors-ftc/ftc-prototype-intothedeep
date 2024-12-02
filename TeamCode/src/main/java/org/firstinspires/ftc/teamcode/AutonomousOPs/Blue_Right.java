package org.firstinspires.ftc.teamcode.AutonomousOPs;

import static org.firstinspires.ftc.teamcode.AutonomousOPs.Features.BuilderFunctions.Tile;
import static org.firstinspires.ftc.teamcode.AutonomousOPs.Features.BuilderFunctions.robotX;
import static org.firstinspires.ftc.teamcode.AutonomousOPs.Features.BuilderFunctions.robotY;
import static org.firstinspires.ftc.teamcode.AutonomousOPs.OpCommon.allianceSampleLeftBlue;
import static org.firstinspires.ftc.teamcode.AutonomousOPs.OpCommon.allianceSampleMidBlue;
import static org.firstinspires.ftc.teamcode.AutonomousOPs.OpCommon.allianceSampleRightBlue;
import static org.firstinspires.ftc.teamcode.AutonomousOPs.OpCommon.basketBlue;
import static org.firstinspires.ftc.teamcode.AutonomousOPs.OpCommon.basketRed;
import static org.firstinspires.ftc.teamcode.AutonomousOPs.OpCommon.grab;
import static org.firstinspires.ftc.teamcode.AutonomousOPs.OpCommon.init_mechanisms;
import static org.firstinspires.ftc.teamcode.AutonomousOPs.OpCommon.neutralSampleLeftBlue;
import static org.firstinspires.ftc.teamcode.AutonomousOPs.OpCommon.neutralSampleLeftRed;
import static org.firstinspires.ftc.teamcode.AutonomousOPs.OpCommon.neutralSampleMidBlue;
import static org.firstinspires.ftc.teamcode.AutonomousOPs.OpCommon.neutralSampleMidRed;
import static org.firstinspires.ftc.teamcode.AutonomousOPs.OpCommon.neutralSampleRightBlue;
import static org.firstinspires.ftc.teamcode.AutonomousOPs.OpCommon.neutralSampleRightRed;
import static org.firstinspires.ftc.teamcode.AutonomousOPs.OpCommon.release;
import static org.firstinspires.ftc.teamcode.AutonomousOPs.OpCommon.scoreBasket;
import static org.firstinspires.ftc.teamcode.AutonomousOPs.OpCommon.startPoseBlueLeft;
import static org.firstinspires.ftc.teamcode.AutonomousOPs.OpCommon.startPoseBlueRight;
import static org.firstinspires.ftc.teamcode.AutonomousOPs.OpCommon.startPoseRedLeft;
import static org.firstinspires.ftc.teamcode.AutonomousOPs.OpCommon.submersibleAllianceBlue;
import static org.firstinspires.ftc.teamcode.AutonomousOPs.OpCommon.submersibleSideBlue;
import static org.firstinspires.ftc.teamcode.AutonomousOPs.OpCommon.submersibleSideRed;
import static org.firstinspires.ftc.teamcode.AutonomousOPs.OpCommon.takeSample;
import static org.firstinspires.ftc.teamcode.AutonomousOPs.OpCommon.temp;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.AutonomousOPs.Features.CrashDetection;
import org.firstinspires.ftc.teamcode.AutonomousOPs.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.AutonomousOPs.trajectorysequence.TrajectorySequenceBuilder;
import org.firstinspires.ftc.teamcode.PrototypeRobot.Commands.ElevatorCommand;

@Autonomous(name = "Autonomous Right Blue")
public class Blue_Right extends CommandOpMode {

    private SampleMecanumDrive drive;
//    private CrashDetection cr;

    private TrajectorySequenceBuilder
            toNeutral_0,
            toNeutral_1,
            toNeutral_2,
            toObservationZone;

    private volatile Pose2d current_pose;
    public void init_toNeutral_0() {
        toNeutral_0 = drive.trajectorySequenceBuilder(startPoseBlueRight)
                .splineToLinearHeading(allianceSampleLeftBlue, Math.toRadians(135));
    }
    public void init_toNeutral_1() {
        toNeutral_1 = drive.trajectorySequenceBuilder(current_pose)
                .lineToLinearHeading(allianceSampleMidBlue);
    }
    public void init_toNeutral_2() {
        toNeutral_2 = drive.trajectorySequenceBuilder(current_pose)
                .lineToLinearHeading(allianceSampleRightBlue);
    }
    public void init_toObservationZone() {
        toObservationZone = drive.trajectorySequenceBuilder(current_pose)
                .setTangent(0)
                .splineToSplineHeading(submersibleAllianceBlue, Math.toRadians(0));
    }

    @Override
    public void initialize() {
        drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(startPoseBlueRight);
        init_mechanisms(hardwareMap, telemetry);
//        cr = new CrashDetection();
    }

    @Override
    public void run() {

        init_toNeutral_0();
        drive.followTrajectorySequenceAsync(toNeutral_0.build());
        while (!isStopRequested() && opModeIsActive() && drive.isBusy()) {
            drive.update();
//            cr.crashCheck(getCurrentRobotVelocity(), );
        }
        drive.setWeightedDrivePower(new Pose2d(0, 0, 0));
        current_pose = drive.getPoseEstimate();

        init_toObservationZone();
        drive.followTrajectorySequenceAsync(toObservationZone.build());
        while (!isStopRequested() && opModeIsActive() && drive.isBusy()) {
            drive.update();
        }
        drive.setWeightedDrivePower(new Pose2d(0, 0, 0));
        current_pose = drive.getPoseEstimate();

        //-----

        init_toNeutral_1();
        drive.followTrajectorySequenceAsync(toNeutral_1.build());
        while (!isStopRequested() && opModeIsActive() && drive.isBusy()) {
            drive.update();
//            cr.crashCheck(getCurrentRobotVelocity(), );
        }
        drive.setWeightedDrivePower(new Pose2d(0, 0, 0));
        current_pose = drive.getPoseEstimate();

        init_toObservationZone();
        drive.followTrajectorySequenceAsync(toObservationZone.build());
        while (!isStopRequested() && opModeIsActive() && drive.isBusy()) {
            drive.update();
        }
        drive.setWeightedDrivePower(new Pose2d(0, 0, 0));
        current_pose = drive.getPoseEstimate();

        //-----

        init_toNeutral_2();
        drive.followTrajectorySequenceAsync(toNeutral_2.build());
        while (!isStopRequested() && opModeIsActive() && drive.isBusy()) {
            drive.update();
//            cr.crashCheck(getCurrentRobotVelocity(), );
        }
        drive.setWeightedDrivePower(new Pose2d(0, 0, 0));
        current_pose = drive.getPoseEstimate();

        init_toObservationZone();
        drive.followTrajectorySequenceAsync(toObservationZone.build());
        while (!isStopRequested() && opModeIsActive() && drive.isBusy()) {
            drive.update();
        }
        drive.setWeightedDrivePower(new Pose2d(0, 0, 0));
        current_pose = drive.getPoseEstimate();


        init_toObservationZone();
        drive.followTrajectorySequenceAsync(toObservationZone.build());
        while (!isStopRequested() && opModeIsActive() && drive.isBusy()) {
            drive.update();
        }
        drive.setWeightedDrivePower(new Pose2d(0, 0, 0));
        current_pose = drive.getPoseEstimate();

//        PoseStorage.currentPose = drive.getPoseEstimate();
    }
}
