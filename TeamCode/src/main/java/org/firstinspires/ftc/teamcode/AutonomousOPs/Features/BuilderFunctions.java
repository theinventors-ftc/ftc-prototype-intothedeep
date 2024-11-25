package org.firstinspires.ftc.teamcode.AutonomousOPs.Features;

import com.acmerobotics.roadrunner.geometry.Pose2d;

public class BuilderFunctions {

    // TODO: Use this as an base for a way point so that its faster and easier
    /*

        name = new WayPoint.WaypointBuilder(
            poseAdjuster(new double[] {
                x * Tile, y * Tile, t
            }, BuilderFunctions.RobotSides.FRONT
            ), WayPoint.WaypointType.DEFAULT
        ).max_radius(v).build()

    */

    //    public Pose2d startPoseRedLeft = poseAdjuster(new Pose2d(Tile, (3 *  Tile), Math.toRadians(90)), BuilderFunctions.RobotSides.REAR);
    //    public Pose2d startPoseBlueRight = poseAdjuster(new Pose2d(Tile, (-3 *  Tile), Math.toRadians(270)), BuilderFunctions.RobotSides.REAR);
    //    public Pose2d startPoseBlueLeft = poseAdjuster(new Pose2d(Tile, (-3 *  Tile), Math.toRadians(270)), BuilderFunctions.RobotSides.REAR);

    public static double
        robotY = 15,
        robotX = 15;

    public static double
        Tile = 24; /*-inches-*/

    public enum RobotSides {FRONT, REAR, CENTER, LEFT, RIGHT}

    /**
     * Adjusts the target pose in order for a certain side of the robot to meet the target point
     * @param pose
     * @param side
     * @return
     */
    public static Pose2d poseAdjuster(Pose2d pose, RobotSides side) {
        if (side == RobotSides.CENTER)
            return pose;

        double X = pose.getX(), Y = pose.getY(), H = pose.getHeading(),
            Afb = (robotY / 2) * Math.sin(H), Bfb = (robotY / 2) * Math.cos(H),
            Arl = (robotX / 2) * Math.sin(H), Brl = (robotX / 2) * Math.cos(H);

        switch (side) {
            case FRONT:
                X -= Bfb;
                Y -= Afb;
                break;
            case REAR:
                X += Bfb;
                Y += Afb;
                break;
            case LEFT:
                X += Arl;
                Y -= Brl;
                break;
            default: // RIGHT
                X -= Arl;
                Y += Brl;
                break;
        }

        return new Pose2d(X, Y, H);
    }
}