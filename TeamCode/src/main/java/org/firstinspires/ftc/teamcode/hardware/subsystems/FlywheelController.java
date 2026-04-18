package org.firstinspires.ftc.teamcode.hardware.subsystems;

import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.configs.Constants;
import org.firstinspires.ftc.teamcode.drivers.RGBIndicator;
import org.firstinspires.ftc.teamcode.groups.LauncherGroup;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;

import java.util.List;
import java.util.Locale;

public class FlywheelController {
    private static final double TICKS_PER_REV = 28.0;

    private static final double MID_ZONE_DISTANCE_FT = 3.5;
    private static final double FAR_ZONE_DISTANCE_FT = 6.0;
    private static final double FAR_FAR_ZONE_DISTANCE_FT = 8.0;

    private final RobotHardware hardware;
    private boolean flywheelEnabled = false;
    private double targetRpm = 0.0;
    private double rpmTolerance = Constants.FLYWHEEL_TOLERANCE_RPM;

    private final ElapsedTime spinupTimer = new ElapsedTime();
    private final ElapsedTime flywheelAdjustmentTimer = new ElapsedTime();
    private boolean measuringSpinup = false;
    private double spinupSetpointRpm = 0.0;

    public FlywheelController(RobotHardware hardware) {
        this.hardware = hardware;
        resetDriverTuningFromConstants();
    }

    public void toggle() {
        flywheelEnabled = !flywheelEnabled;
        if (flywheelEnabled) setFlywheelRpm(Constants.DEFAULT_RPM);
        else stop();
    }

    public boolean isEnabled() {
        return flywheelEnabled;
    }

    public double getTargetRpm() {
        return targetRpm;
    }

    public double getCurrentRpm() {
        LauncherGroup launcherGroup = hardware.launcher;
        if (launcherGroup == null) {
            Telemetry telemetry = hardware.getTelemetry();
            telemetry.addLine("ERROR: launcher motor is NULL!");
            return 0.0;
        }
        return (launcherGroup.group.getVelocity() * 60.0) / TICKS_PER_REV;
    }

    public boolean isAtSpeed() {
        // EXAMPLE:
        // RPM : 2,200
        // RPM TOLERANCE : 100
        // RANGE : 2,300 - 2,150
        return (getCurrentRpm() >= (targetRpm - (rpmTolerance/2))) && ( getCurrentRpm() <= (targetRpm + rpmTolerance) ); //return Math.abs(getCurrentRpm() - targetRpm) <= rpmTolerance;
    }

    public double getRpmTolerance() {
        return rpmTolerance;
    }

    public void adjustRpmTolerance(double delta) {
        rpmTolerance = Math.max(0.0, rpmTolerance + delta);
    }

    public void adjustLauncherFeedforward(double delta) {
        hardware.flywheelPidfConfig.launcherF = Math.max(0.0, hardware.flywheelPidfConfig.launcherF + delta);
    }

    public void setLauncherFeedforward(double delta) {
        hardware.flywheelPidfConfig.launcherF = delta;
    }

    public void resetDriverTuningFromConstants() {
        rpmTolerance = Constants.FLYWHEEL_TOLERANCE_RPM;
        hardware.flywheelPidfConfig.reset();
    }

    /**
     * Call every loop to update the RPM based on the detected AprilTag.
     */
    public void update() {
        hardware.launcher.refreshLauncherPIDFFromConfig();
        Telemetry telemetry = hardware.getTelemetry();

        if (!flywheelEnabled) {
            hardware.setFrontRGBIndicator(RGBIndicator.LEDColors.OFF);
            publishPanelsFlywheelTelemetry(targetRpm, getCurrentRpm());
            return;
        }

        if (hardware.launcher == null) {
            telemetry.addLine("ERROR: launcher motor is NULL!");
            hardware.setFrontRGBIndicator(RGBIndicator.LEDColors.OFF);
            return;
        }

        double rpm = Constants.DEFAULT_RPM;

        LLResult result = hardware.getLatestLimelightResult();
        if (result != null && result.isValid()) {
            List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
            if (fiducials != null && !fiducials.isEmpty()) {
                LLResultTypes.FiducialResult fid = fiducials.get(0);
                Pose3D pose = fid.getRobotPoseTargetSpace();
                Position position = pose != null ? pose.getPosition() : null;

                if (position != null) {
                    Position metersPosition = position.toUnit(DistanceUnit.METER);
                    double xMeters = metersPosition.x;
                    double yMeters = metersPosition.y;
                    double zMeters = metersPosition.z;
                    // Use full 3D translation magnitude to avoid underestimating range.
                    double distanceMeters = Math.sqrt(xMeters * xMeters + yMeters * yMeters + zMeters * zMeters);
                    double distanceFeet = distanceMeters * 3.28084;

                    if (distanceFeet >= FAR_FAR_ZONE_DISTANCE_FT) {
                        //setLauncherFeedforward(29);
                        rpm = Constants.LAUNCH_ZONE_FAR_FAR_RPM;
                    } else if (distanceFeet < MID_ZONE_DISTANCE_FT) {
                        //setLauncherFeedforward(31);
                        rpm = Constants.LAUNCH_ZONE_MID_RPM;
                    } else {
                        //setLauncherFeedforward(31);
                        double clampedDistance = Range.clip(distanceFeet, MID_ZONE_DISTANCE_FT, FAR_ZONE_DISTANCE_FT);
                        double distanceRatio = (clampedDistance - MID_ZONE_DISTANCE_FT) / (FAR_ZONE_DISTANCE_FT - MID_ZONE_DISTANCE_FT);
                        rpm = Constants.LAUNCH_ZONE_MID_RPM
                                + distanceRatio * (Constants.LAUNCH_ZONE_FAR_RPM - Constants.LAUNCH_ZONE_MID_RPM);
                    }

                    telemetry.addData("Flywheel Distance (ft)", "%.2f", distanceFeet);
                }
            }
        }

        if (targetRpm == 0 || result == null || !result.isValid()) {
            setLauncherFeedforward(31.0);
            flywheelAdjustmentTimer.reset();
        } else if (flywheelAdjustmentTimer.seconds() >= Constants.FLYWHEEL_ADJUSTMENT_TIME) {
            if (getCurrentRpm() >= targetRpm+rpmTolerance) {
                adjustLauncherFeedforward(-Constants.FLYWHEEL_ADJUSTMENT_INCREMENT);
            } else if (getCurrentRpm() <= targetRpm-rpmTolerance) {
                adjustLauncherFeedforward(Constants.FLYWHEEL_ADJUSTMENT_INCREMENT);
            }
            flywheelAdjustmentTimer.reset();
        }

        rpm = Math.max(rpm, Constants.DEFAULT_RPM);
        setFlywheelRpm(rpm);

        updateFrontLedColor();

        publishPanelsFlywheelTelemetry(targetRpm, getCurrentRpm());

        if (measuringSpinup && isAtSpeed()) {
            double elapsedSeconds = spinupTimer.seconds();
            RobotLog.ii("FlywheelController", "Spin-up to %.0f RPM reached in %.2f s", spinupSetpointRpm, elapsedSeconds);
            measuringSpinup = false;
        }
    }

    private void stop() {
        targetRpm = 0.0;
        measuringSpinup = false;
        LauncherGroup launcherGroup = hardware.launcher;
        if (launcherGroup != null) {
            launcherGroup.group.setVelocity(0);
            launcherGroup.group.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        publishPanelsFlywheelTelemetry(targetRpm, getCurrentRpm());
    }

    private void setFlywheelRpm(double rpm) {
        if (rpm > 0 && targetRpm <= 0) {
            spinupSetpointRpm = rpm;
            spinupTimer.reset();
            measuringSpinup = true;
        }

        targetRpm = rpm;
        LauncherGroup launcherGroup = hardware.launcher;
        if (launcherGroup == null) {
            Telemetry telemetry = hardware.getTelemetry();
            telemetry.addLine("ERROR: launcher motor is NULL!");
            return;
        }

        double ticksPerSecond = rpmToTicksPerSecond(rpm);
        launcherGroup.group.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcherGroup.group.setVelocity(ticksPerSecond);
    }

    private double rpmToTicksPerSecond(double rpm) {
        return (rpm * TICKS_PER_REV) / 60.0;
    }

    public void publishPanelsFlywheelTelemetry(double target, double current) {
        TelemetryManager panelsTelemetry = hardware.getPanels();
        if (panelsTelemetry == null) return;

        panelsTelemetry.addLine("--- FLYWHEEL RPM ---");
        panelsTelemetry.debug("Flywheel RPM (target/current)", String.format(Locale.US, "%.0f / %.0f", target, current));
        panelsTelemetry.addLine("-------------------------------");
    }

    private void updateFrontLedColor() {
        if (hardware.rgbIndicatorGroup.rgbIndicators[RobotHardware.RGBIndicatorPosition.FRONT_LED] == null) return;

        if (!flywheelEnabled) {
            hardware.setFrontRGBIndicator(RGBIndicator.LEDColors.OFF);
            return;
        }

        if (targetRpm == Constants.DEFAULT_RPM) {
            hardware.setFrontRGBIndicator(RGBIndicator.LEDColors.VIOLET);
            return;
        }

        double currentRpm = Math.abs(getCurrentRpm());
        double minimumRpm = targetRpm - rpmTolerance;
        double maxRpm = targetRpm + rpmTolerance;

        if (isAtSpeed() ) {
            hardware.setFrontRGBIndicator(RGBIndicator.LEDColors.GREEN);
        } else if (currentRpm > maxRpm ) {
            hardware.setFrontRGBIndicator(RGBIndicator.LEDColors.AZURE);
        } else if (currentRpm >= minimumRpm * 0.75) {
            hardware.setFrontRGBIndicator(RGBIndicator.LEDColors.ORANGE);
        } else if (currentRpm >= minimumRpm * 0.5) {
            hardware.setFrontRGBIndicator(RGBIndicator.LEDColors.YELLOW);
        } else {
            hardware.setFrontRGBIndicator(RGBIndicator.LEDColors.RED);
        }
    }
}
