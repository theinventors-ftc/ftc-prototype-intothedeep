package org.firstinspires.ftc.teamcode.AutonomousOPs;

import static org.firstinspires.ftc.teamcode.AutonomousOPs.Features.BuilderFunctions.Tile;
import static org.firstinspires.ftc.teamcode.AutonomousOPs.Features.BuilderFunctions.robotX;
import static org.firstinspires.ftc.teamcode.AutonomousOPs.Features.BuilderFunctions.robotY;
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
import static org.firstinspires.ftc.teamcode.AutonomousOPs.OpCommon.startPoseRedLeft;
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

@Autonomous(name = "Autonomous Left Blue")
public class Blue_Left extends CommandOpMode {

    private SampleMecanumDrive drive;
//    private CrashDetection cr;

    private TrajectorySequenceBuilder
            toNeutral_0,
            toNeutral_1,
            toNeutral_2,
            toBasket,
            toAscentZone;

    private volatile Pose2d current_pose;

    public void init_toNeutral_0() {
        toNeutral_0 = drive.trajectorySequenceBuilder(startPoseBlueLeft)
                .splineToLinearHeading(neutralSampleLeftBlue, Math.toRadians(135));
    }
    public void init_toBasket() {
        toBasket = drive.trajectorySequenceBuilder(current_pose)
                .lineToLinearHeading(basketBlue);
    }
    public void init_toNeutral_1() {
        toNeutral_1 = drive.trajectorySequenceBuilder(current_pose)
                .lineToLinearHeading(neutralSampleMidBlue);
    }
    public void init_toNeutral_2() {
        toNeutral_2 = drive.trajectorySequenceBuilder(current_pose)
                .lineToLinearHeading(neutralSampleRightBlue);
    }
    public void init_toAscentZone() {
        toAscentZone = drive.trajectorySequenceBuilder(current_pose)
                .setTangent(0)
                .splineToSplineHeading(submersibleSideBlue, Math.toRadians(0));
    }

//    public double getCurrentRobotVelocity() {
//        return Math.hypot(drive.getWheelVelocities().get(0),
//                          drive.getWheelVelocities().get(1));
//    }

//    public double getCurrentTargetRobotVelocity() {
//        return drive.currentDuration;
//    }

    @Override
    public void initialize() {
        drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(startPoseBlueLeft);
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


        temp = takeSample();
        temp.schedule();
        while (!isStopRequested() && opModeIsActive() && CommandScheduler.getInstance().isScheduled(temp)) {
            run();
        }

        temp = grab();
        temp.schedule();
        while(!isStopRequested() && opModeIsActive() && CommandScheduler.getInstance().isScheduled(temp)) {
            run();
        }

        temp = scoreBasket();
        temp.schedule();
        init_toBasket();
        drive.followTrajectorySequenceAsync(toBasket.build());
        while (!isStopRequested() && opModeIsActive() && drive.isBusy()
                && CommandScheduler.getInstance().isScheduled(temp)) {
            run();
            drive.update();
        }
        drive.setWeightedDrivePower(new Pose2d(0, 0, 0));
        current_pose = drive.getPoseEstimate();

        temp = release();
        temp.schedule();
        while (!isStopRequested() && opModeIsActive() && CommandScheduler.getInstance().isScheduled(temp)) {
            run();
        }

        //-----
        temp = takeSample();
        temp.schedule();
        init_toNeutral_1();
        drive.followTrajectorySequenceAsync(toNeutral_1.build());
        while (!isStopRequested() && opModeIsActive() && drive.isBusy()
                && CommandScheduler.getInstance().isScheduled(temp)) {
            drive.update();
            run();
//            cr.crashCheck(getCurrentRobotVelocity(), );
        }
        drive.setWeightedDrivePower(new Pose2d(0, 0, 0));
        current_pose = drive.getPoseEstimate();

        temp = grab();
        temp.schedule();
        while(!isStopRequested() && opModeIsActive() && CommandScheduler.getInstance().isScheduled(temp)) {
            run();
        }

        temp = scoreBasket();
        temp.schedule();
        init_toBasket();
        drive.followTrajectorySequenceAsync(toBasket.build());
        while (!isStopRequested() && opModeIsActive() && drive.isBusy()
                && CommandScheduler.getInstance().isScheduled(temp)) {
            run();
            drive.update();
        }
        drive.setWeightedDrivePower(new Pose2d(0, 0, 0));
        current_pose = drive.getPoseEstimate();

        temp = release();
        temp.schedule();
        while (!isStopRequested() && opModeIsActive() && CommandScheduler.getInstance().isScheduled(temp)) {
            run();
        }

        //-----
        temp = takeSample();
        temp.schedule();
        init_toNeutral_2();
        drive.followTrajectorySequenceAsync(toNeutral_2.build());
        while (!isStopRequested() && opModeIsActive() && drive.isBusy()
                && CommandScheduler.getInstance().isScheduled(temp)) {
            drive.update();
            run();
//            cr.crashCheck(getCurrentRobotVelocity(), );
        }
        drive.setWeightedDrivePower(new Pose2d(0, 0, 0));
        current_pose = drive.getPoseEstimate();

        temp = grab();
        temp.schedule();
        while(!isStopRequested() && opModeIsActive() && CommandScheduler.getInstance().isScheduled(temp)) {
            run();
        }

        temp = scoreBasket();
        temp.schedule();
        init_toBasket();
        drive.followTrajectorySequenceAsync(toBasket.build());
        while (!isStopRequested() && opModeIsActive() && drive.isBusy()
                && CommandScheduler.getInstance().isScheduled(temp)) {
            run();
            drive.update();
        }
        drive.setWeightedDrivePower(new Pose2d(0, 0, 0));
        current_pose = drive.getPoseEstimate();

        temp = release();
        temp.schedule();
        while (!isStopRequested() && opModeIsActive() && CommandScheduler.getInstance().isScheduled(temp)) {
            run();
        }

        //-----

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
