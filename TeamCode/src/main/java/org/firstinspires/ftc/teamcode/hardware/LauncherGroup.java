package org.firstinspires.ftc.teamcode.hardware;

import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.configs.Constants;
import org.firstinspires.ftc.teamcode.configs.FlywheelPidfConfig;
import org.firstinspires.ftc.teamcode.groups.DcMotorExGroup;

import java.util.Locale;

public class LauncherGroup {
    private final RobotHardware hardware;
    private final FlywheelPidfConfig flywheelPidfConfig;

    public DcMotorExGroup group;

    private double lastLauncherBaseP = Double.NaN;
    private double lastLauncherBaseI = Double.NaN;
    private double lastLauncherBaseD = Double.NaN;
    private double lastLauncherBaseF = Double.NaN;
    private double lastLauncherScaledP = Double.NaN;
    private double lastLauncherScaledF = Double.NaN;

    public LauncherGroup(RobotHardware hardware, FlywheelPidfConfig flywheelPidfConfig, DcMotorExGroup launcherMotors) {
        this.hardware = hardware;
        this.flywheelPidfConfig = flywheelPidfConfig;

        Telemetry telemetry = hardware.getTelemetry();
        if (launcherMotors == null || launcherMotors.motors.length < 2) {
            telemetry.addLine("ERROR: launcher motor is NULL!");
            return;
        }

        launcherMotors.motors[0].setDirection(DcMotorSimple.Direction.FORWARD); // launcher
        launcherMotors.motors[1].setDirection(DcMotorSimple.Direction.REVERSE); // launcher2

        this.group = launcherMotors;
        this.group.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        this.group.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.group.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.group.setTargetPositionTolerance(25);
    }

    public void applyLauncherPIDFTuning() {
        double gearScaledP = flywheelPidfConfig.launcherP * Constants.LAUNCHER_GEAR_REDUCTION;
        double gearScaledI = flywheelPidfConfig.launcherI * Constants.LAUNCHER_GEAR_REDUCTION;
        double gearScaledD = flywheelPidfConfig.launcherD * Constants.LAUNCHER_GEAR_REDUCTION;
        double gearScaledF = flywheelPidfConfig.launcherF * Constants.LAUNCHER_GEAR_REDUCTION;

        PIDFCoefficients pidf = new PIDFCoefficients(gearScaledP, gearScaledI, gearScaledD, gearScaledF);

        group.setVelocityPIDFCoefficients(pidf.p, pidf.i, pidf.d, pidf.f);

        lastLauncherBaseP = flywheelPidfConfig.launcherP;
        lastLauncherBaseI = flywheelPidfConfig.launcherI;
        lastLauncherBaseD = flywheelPidfConfig.launcherD;
        lastLauncherBaseF = flywheelPidfConfig.launcherF;
        lastLauncherScaledP = pidf.p;
        lastLauncherScaledF = pidf.f;

        publishLauncherPIDFTelemetry();
    }

    public void refreshLauncherPIDFFromConfig() {
        boolean baseChanged = flywheelPidfConfig.launcherP != lastLauncherBaseP
                || flywheelPidfConfig.launcherI != lastLauncherBaseI
                || flywheelPidfConfig.launcherD != lastLauncherBaseD
                || flywheelPidfConfig.launcherF != lastLauncherBaseF;

        if (baseChanged || !Double.isFinite(lastLauncherScaledP) || !Double.isFinite(lastLauncherScaledF)) {
            applyLauncherPIDFTuning();
            return;
        }

        publishLauncherPIDFTelemetry();
    }

    private void publishLauncherPIDFTelemetry() {
        if (!Double.isFinite(lastLauncherScaledP) || !Double.isFinite(lastLauncherScaledF)) {
            return;
        }
        TelemetryManager telemetryManager = hardware.getPanels();
        telemetryManager.addLine("--- PIDF ---");
        telemetryManager.debug("Launcher PIDF base (P,I,D,F)",
                String.format(Locale.US, "P=%.3f I=%.3f D=%.3f F=%.3f", lastLauncherBaseP, lastLauncherBaseI, lastLauncherBaseD, lastLauncherBaseF));
        telemetryManager.addLine("----------------");
    }
}
