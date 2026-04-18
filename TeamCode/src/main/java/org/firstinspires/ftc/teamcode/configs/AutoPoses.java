package org.firstinspires.ftc.teamcode.configs;

import com.pedropathing.geometry.Pose;

public class AutoPoses {
    private static final double FIELD_WIDTH = 144.0;

    public static Pose mirrorRed(Pose blue) {
        return new Pose(FIELD_WIDTH - blue.getX(), blue.getY(), Math.PI - blue.getHeading());
    }

    // Near
    public static final Pose BLUE_NEAR_START = new Pose(23, 119.8, Math.toRadians(-90));
    public static final Pose BLUE_NEAR_SHOOT = new Pose(59, 84, Math.toRadians(-90));
    public static final Pose BLUE_NEAR_GOTO_SPIKE1 = new Pose(41.5, 84, Math.toRadians(180));
    public static final Pose BLUE_NEAR_SPIKE1_P1 = new Pose(33, 84, Math.toRadians(180));
    public static final Pose BLUE_NEAR_SPIKE1_P2 = new Pose(16.5, 84, Math.toRadians(180));
    public static final Pose BLUE_NEAR_LEAVE = new Pose(44, 71.5, Math.toRadians(-135));
    public static final Pose BLUE_NEAR_WALL = new Pose(59, 133.6, Math.toRadians(-90));

    public static final Pose RED_NEAR_START = mirrorRed(BLUE_NEAR_START);
    public static final Pose RED_NEAR_SHOOT = mirrorRed(BLUE_NEAR_SHOOT);
    public static final Pose RED_NEAR_GOTO_SPIKE1 = mirrorRed(BLUE_NEAR_GOTO_SPIKE1);
    public static final Pose RED_NEAR_SPIKE1_P1 = mirrorRed(BLUE_NEAR_SPIKE1_P1);
    public static final Pose RED_NEAR_SPIKE1_P2 = mirrorRed(BLUE_NEAR_SPIKE1_P2);
    public static final Pose RED_NEAR_LEAVE = mirrorRed(BLUE_NEAR_LEAVE);
    public static final Pose RED_NEAR_WALL = mirrorRed(BLUE_NEAR_WALL);

    // Far
    public static final Pose BLUE_FAR_START = new Pose(60, 8.75, Math.toRadians(-90));
    public static final Pose BLUE_FAR_SHOOT = new Pose(60, 18.75, Math.toRadians(-90));
    public static final Pose BLUE_FAR_SPIKE3_LINEUP = new Pose(47.75, 24, Math.toRadians(180));
    public static final Pose BLUE_FAR_SPIKE3_P1 = new Pose(33, 24, Math.toRadians(180));
    public static final Pose BLUE_FAR_SPIKE3_P2 = new Pose(16.5, 24, Math.toRadians(180));
    public static final Pose BLUE_FAR_LEAVE = new Pose(60, 34.5, Math.toRadians(-90));

    public static final Pose RED_FAR_START = mirrorRed(BLUE_FAR_START);
    public static final Pose RED_FAR_SHOOT = mirrorRed(BLUE_FAR_SHOOT);
    public static final Pose RED_FAR_SPIKE3_LINEUP = mirrorRed(BLUE_FAR_SPIKE3_LINEUP);
    public static final Pose RED_FAR_SPIKE3_P1 = mirrorRed(BLUE_FAR_SPIKE3_P1);
    public static final Pose RED_FAR_SPIKE3_P2 = mirrorRed(BLUE_FAR_SPIKE3_P2);
    public static final Pose RED_FAR_LEAVE = mirrorRed(BLUE_FAR_LEAVE);
}
