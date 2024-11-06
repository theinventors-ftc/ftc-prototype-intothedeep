package org.firstinspires.ftc.teamcode.TeleOPs;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.PoseStorage;
import org.firstinspires.ftc.teamcode.PrototypeRobot.PrototypeRobot;
import org.inventors.ftc.robotbase.RobotEx;
import org.inventors.ftc.robotbase.drive.DriveConstants;
import org.inventors.ftc.robotbase.hardware.GamepadExEx;

@Disabled
@TeleOp(name = "Do not run this TeleOP", group = "")
public class TeleOpBase extends CommandOpMode {
    GamepadExEx driverOp, toolOp;
    private DriveConstants RobotConstants;
    public Pose2d pose;
    private ElapsedTime runtime;
    private PrototypeRobot robot;

    @Override
    public void initialize() {
        driverOp = new GamepadExEx(gamepad1);
        toolOp = new GamepadExEx(gamepad2);

        RobotConstants = new DriveConstants();

        RobotConstants.frontLeftInverted = true;
        RobotConstants.frontRightInverted = true;
        RobotConstants.rearRightInverted = true;
        RobotConstants.rearLeftInverted = true;

        RobotConstants.WHEEL_RADIUS = 1; // inch
        RobotConstants.GEAR_RATIO = 1; // output (wheel) speed / input (motor) speed
        RobotConstants.TRACK_WIDTH = 10; // in

        RobotConstants.MAX_VEL = 90;
        RobotConstants.MAX_ACCEL = 90;
        RobotConstants.MAX_ANG_VEL = Math.toRadians(360);
        RobotConstants.MAX_ANG_ACCEL = Math.toRadians(360);

        RobotConstants.frontLeftFeedForward[0] = 0;
        RobotConstants.frontLeftFeedForward[1] = 1;
        RobotConstants.frontLeftFeedForward[2] = 0;
        RobotConstants.frontRightFeedForward[0] = 0;
        RobotConstants.frontRightFeedForward[1] = 1;
        RobotConstants.frontRightFeedForward[2] = 0;
        RobotConstants.rearLeftFeedForward[0] = 0;
        RobotConstants.rearLeftFeedForward[1] = 1;
        RobotConstants.rearLeftFeedForward[2] = 0;
        RobotConstants.rearRightFeedForward[0] = 0;
        RobotConstants.rearRightFeedForward[1] = 1;
        RobotConstants.rearRightFeedForward[2] = 0;

        RobotConstants.VELO_KP = 0;
        RobotConstants.VELO_KI = 0;
        RobotConstants.VELO_KD = 0;

        RobotConstants.TICKS_PER_REV = 537;
        RobotConstants.MAX_RPM = 312;

        RobotConstants.DEFAULT_SPEED_PERC = 0.5;
        RobotConstants.SLOW_SPEED_PERC = 0.3;
        RobotConstants.FAST_SPEED_PERC = 1;

        pose = PoseStorage.currentPose;
    }

    public void initAllianceRelated(RobotEx.Alliance alliance) {
        robot = new PrototypeRobot(hardwareMap, RobotConstants, telemetry, driverOp, toolOp,
                RobotEx.OpModeType.TELEOP, alliance, "external_imu",
                false, pose);
    }

    @Override
    public void run() {
        super.run();
        telemetry.update();
    }
}