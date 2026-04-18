package org.firstinspires.ftc.teamcode.configs;

public class TurretAimConfig {
    public double turretFarAimAdjustBlue = Constants.TURRET_FAR_AIM_ADJUST_BLUE;
    public double turretFarAimAdjustRed = Constants.TURRET_FAR_AIM_ADJUST_RED;

    public void reset() {
        turretFarAimAdjustBlue = Constants.TURRET_FAR_AIM_ADJUST_BLUE;
        turretFarAimAdjustRed = Constants.TURRET_FAR_AIM_ADJUST_RED;
    }
}
