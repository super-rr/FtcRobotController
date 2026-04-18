package org.firstinspires.ftc.teamcode.hardware.subsystems;

import androidx.annotation.NonNull;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.configs.Constants;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.hardware.SharedState;

import java.util.List;
import java.util.Locale;
import java.util.function.BooleanSupplier;

public class ReadObelisk {
    public enum ArtifactColor {
        GREEN,
        PURPLE
    }

    public static class ObeliskPattern {
        private final ArtifactColor[] colors;

        public ObeliskPattern(ArtifactColor first, ArtifactColor second, ArtifactColor third) {
            this.colors = new ArtifactColor[]{first, second, third};
        }

        public ArtifactColor[] getColors() {
            return colors.clone();
        }

        @NonNull
        @Override
        public String toString() {
            return String.format(Locale.US, "%s, %s, %s", colors[0], colors[1], colors[2]);
        }
    }

    private static volatile ObeliskPattern cachedPattern;

    private final RobotHardware hardware;
    private final LinearOpMode linearOpMode;

    public ReadObelisk(RobotHardware hardware, LinearOpMode linearOpMode) {
        this.hardware = hardware;
        this.linearOpMode = linearOpMode;
    }

    public static ObeliskPattern getCachedPattern() {
        if (cachedPattern == null) {
            cachedPattern = SharedState.loadObeliskPattern();
        }
        return cachedPattern;
    }

    public ObeliskPattern decodeLatestAndCache() {
        hardware.refreshLimelightResult();
        ObeliskPattern pattern = decodePattern(hardware.getLatestLimelightResult());
        if (pattern != null) {
            cachedPattern = pattern;
            SharedState.saveObeliskPattern(pattern);
        }
        return pattern;
    }

    public ObeliskPattern scanForPattern() {
        return scanForPattern(() -> false);
    }

    public ObeliskPattern scanForPattern(BooleanSupplier shouldCancel) {
        Telemetry telemetry = hardware.getTelemetry();

        if (hardware.limelight == null) {
            telemetry.addLine("** ERROR: Limelight not initialized");
            telemetry.update();
            return cachedPattern;
        }

        if (hardware.turret == null) {
            telemetry.addLine("** ERROR: Turret motor not initialized");
            telemetry.update();
            return cachedPattern;
        }

        hardware.refreshAllianceFromSwitchState();
        boolean allianceColor = hardware.getAllianceColor() == RobotHardware.AllianceColor.RED;
        String sweepDirection = allianceColor ? "LEFT (negative)" : "RIGHT (positive)";

        hardware.limelight.pipelineSwitch(1);
        hardware.refreshLimelightResult();

        int targetTicks = allianceColor ? Constants.TURRET_OBELISK_LEFT_LIMIT : Constants.TURRET_OBELISK_RIGHT_LIMIT;

        ObeliskPattern detected = cachedPattern;
        boolean targetFound = detected != null;

        driveTurretTo(targetTicks);

        while (linearOpMode.opModeIsActive() && hardware.turret.isBusy() && !shouldCancel.getAsBoolean()) {
            hardware.refreshLimelightResult();
            detected = decodePattern(hardware.getLatestLimelightResult());
            targetFound = detected != null;
            if (targetFound) {
                cachedPattern = detected;
                SharedState.saveObeliskPattern(cachedPattern);
            }

            telemetry.addData("Alliance Color", allianceColor ? "RED" : "BLUE");
            telemetry.addData("Turret Sweep", sweepDirection);
            telemetry.addData("Turret Position", hardware.turret.getCurrentPosition());
            telemetry.addData("Target Found", targetFound);
            if (targetFound) {
                telemetry.addData("Pattern", cachedPattern);
            }
            telemetry.update();

            linearOpMode.idle();
            if (targetFound) {
                break;
            }
        }

        if (cachedPattern == null) {
            cachedPattern = decodePattern(hardware.getLatestLimelightResult());
            if (cachedPattern != null) {
                SharedState.saveObeliskPattern(cachedPattern);
                targetFound = true;
            }
        }

        driveTurretTo(Constants.TURRET_HOME);

        telemetry.addData("Alliance Color", allianceColor ? "RED" : "BLUE");
        telemetry.addData("Turret Sweep", "Returning to home");
        telemetry.addData("Turret Position", hardware.turret.getCurrentPosition());
        telemetry.addData("Target Found", targetFound);
        if (cachedPattern != null) {
            telemetry.addData("Pattern", cachedPattern.toString());
        } else {
            telemetry.addLine("No valid obelisk tag detected");
        }
        telemetry.update();

        return cachedPattern;
    }

    private void driveTurretTo(int targetTicks) {
        DcMotorEx turret = hardware.turret;
        turret.setTargetPosition(targetTicks);
        turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turret.setPower(0.35);
    }

    private ObeliskPattern decodePattern(LLResult result) {
        if (result == null || !result.isValid()) return null;

        List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
        if (fiducials == null || fiducials.isEmpty()) return null;

        int id = fiducials.get(0).getFiducialId();
        switch (id) {
            case 21:
                return new ObeliskPattern(ArtifactColor.GREEN, ArtifactColor.PURPLE, ArtifactColor.PURPLE);
            case 22:
                return new ObeliskPattern(ArtifactColor.PURPLE, ArtifactColor.GREEN, ArtifactColor.PURPLE);
            case 23:
                return new ObeliskPattern(ArtifactColor.PURPLE, ArtifactColor.PURPLE, ArtifactColor.GREEN);
            default:
                return null;
        }
    }
}
