package org.firstinspires.ftc.teamcode.configs;

public class FlywheelPidfConfig {
    public double launcherP = Constants.LAUNCHER_P;
    public double launcherI = Constants.LAUNCHER_I;
    public double launcherD = Constants.LAUNCHER_D;
    public double launcherF = Constants.LAUNCHER_F;

    public void reset() {
        launcherP = Constants.LAUNCHER_P;
        launcherI = Constants.LAUNCHER_I;
        launcherD = Constants.LAUNCHER_D;
        launcherF = Constants.LAUNCHER_F;
    }
}
