package org.firstinspires.ftc.teamcode.configs;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

public class AutoPaths {
    // Near paths
    public final PathChain nearStartToShoot;
    public final PathChain nearShootToSpike1Lineup;
    public final PathChain nearSpike1Part1;
    public final PathChain nearSpike1Part2;
    public final PathChain nearSpike1ToShoot;
    public final PathChain nearShootLeave;
    public final PathChain nearShootToWall;

    // Far paths
    public final PathChain farStartToShoot;
    public final PathChain farShootToSpike3Lineup;
    public final PathChain farSpike3Part1;
    public final PathChain farSpike3Part2;
    public final PathChain farSpike3ToShoot;
    public final PathChain farShootLeave;

    public AutoPaths(Follower follower, boolean isBlue) {
        Pose nearStart = isBlue ? AutoPoses.BLUE_NEAR_START : AutoPoses.RED_NEAR_START;
        Pose nearShoot = isBlue ? AutoPoses.BLUE_NEAR_SHOOT : AutoPoses.RED_NEAR_SHOOT;
        Pose nearSpike1Go = isBlue ? AutoPoses.BLUE_NEAR_GOTO_SPIKE1 : AutoPoses.RED_NEAR_GOTO_SPIKE1;
        Pose nearSpike1P1 = isBlue ? AutoPoses.BLUE_NEAR_SPIKE1_P1 : AutoPoses.RED_NEAR_SPIKE1_P1;
        Pose nearSpike1P2 = isBlue ? AutoPoses.BLUE_NEAR_SPIKE1_P2 : AutoPoses.RED_NEAR_SPIKE1_P2;
        Pose nearLeave = isBlue ? AutoPoses.BLUE_NEAR_LEAVE : AutoPoses.RED_NEAR_LEAVE;
        Pose nearWall = isBlue ? AutoPoses.BLUE_NEAR_WALL : AutoPoses.RED_NEAR_WALL;

        Pose farStart = isBlue ? AutoPoses.BLUE_FAR_START : AutoPoses.RED_FAR_START;
        Pose farShoot = isBlue ? AutoPoses.BLUE_FAR_SHOOT : AutoPoses.RED_FAR_SHOOT;
        Pose farSpike3Lineup = isBlue ? AutoPoses.BLUE_FAR_SPIKE3_LINEUP: AutoPoses.RED_FAR_SPIKE3_LINEUP;
        Pose farSpike3P1 = isBlue ? AutoPoses.BLUE_FAR_SPIKE3_P1 : AutoPoses.RED_FAR_SPIKE3_P1;
        Pose farSpike3P2 = isBlue ? AutoPoses.BLUE_FAR_SPIKE3_P2 : AutoPoses.RED_FAR_SPIKE3_P2;
        Pose farLeave = isBlue ? AutoPoses.BLUE_FAR_LEAVE : AutoPoses.RED_FAR_LEAVE;

        // Near
        nearStartToShoot = line(follower, nearStart, nearShoot);
        nearShootToSpike1Lineup = curve(follower, nearShoot, midpoint(nearShoot, nearSpike1Go, 0, 8), nearSpike1Go);
        nearSpike1Part1 = line(follower, nearSpike1Go, nearSpike1P1);
        nearSpike1Part2 = line(follower, nearSpike1P1, nearSpike1P2);
        nearSpike1ToShoot = curve(follower, nearSpike1P2, midpoint(nearSpike1P2, nearShoot, 0, -6), nearShoot);
        nearShootLeave = line(follower, nearShoot, nearLeave);
        nearShootToWall = line(follower, nearShoot, nearWall);

        // Far
        farStartToShoot = line(follower, farStart, farShoot);
        farShootToSpike3Lineup = line(follower, farShoot, farSpike3Lineup);
        farSpike3Part1 = line(follower, farSpike3Lineup, farSpike3P1);
        farSpike3Part2 = line(follower, farSpike3P1, farSpike3P2);
        farSpike3ToShoot = curve(follower, farSpike3P2, midpoint(farSpike3P2, farShoot, 0, -8), farShoot);
        farShootLeave = line(follower, farShoot, farLeave);
    }

    private static PathChain line(Follower follower, Pose from, Pose to) {
        return follower.pathBuilder(PedroConstants.pathConstraints)
                .addPath(new BezierLine(from, to))
                .setConstantHeadingInterpolation(to.getHeading())
                .build();
    }

    private static PathChain curve(Follower follower, Pose from, Pose control, Pose to) {
        return follower.pathBuilder(PedroConstants.pathConstraints)
                .addPath(new BezierCurve(from, control, to))
                .setLinearHeadingInterpolation(from.getHeading(), to.getHeading())
                .build();
    }

    private static Pose midpoint(Pose a, Pose b, double offsetX, double offsetY) {
        return new Pose(
                (a.getX() + b.getX()) / 2.0 + offsetX,
                (a.getY() + b.getY()) / 2.0 + offsetY,
                (a.getHeading() + b.getHeading()) / 2.0
        );
    }
}
