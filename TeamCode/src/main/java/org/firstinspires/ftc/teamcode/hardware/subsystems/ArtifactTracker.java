package org.firstinspires.ftc.teamcode.hardware.subsystems;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.configs.Constants;
import org.firstinspires.ftc.teamcode.drivers.RGBIndicator;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;

import java.util.Locale;

public class ArtifactTracker {
    public enum SlotStatus {
        VACANT,
        GREEN,
        PURPLE
    }

    private final RobotHardware hardware;
    private final SlotStatus[] slotStatuses = new SlotStatus[]{SlotStatus.VACANT, SlotStatus.VACANT, SlotStatus.VACANT};
    private SlotStatus[] lastLoggedStatuses = null;

    public ArtifactTracker(RobotHardware hardware) {
        this.hardware = hardware;
    }

    public SlotStatus[] getSlotStatuses() {
        return slotStatuses.clone();
    }

    public SlotStatus getSlotStatus(int index) {
        if (index < 0 || index >= slotStatuses.length) {
            throw new IndexOutOfBoundsException("Slot index must be between 0 and 2");
        }
        return slotStatuses[index];
    }

    public void update() {
        RGBIndicator rearRGB1 = hardware.rgbIndicatorGroup.rgbIndicators[RobotHardware.RGBIndicatorPosition.REAR_RGB_1];
        RGBIndicator rearRGB2 = hardware.rgbIndicatorGroup.rgbIndicators[RobotHardware.RGBIndicatorPosition.REAR_RGB_2];
        RGBIndicator rearRGB3 = hardware.rgbIndicatorGroup.rgbIndicators[RobotHardware.RGBIndicatorPosition.REAR_RGB_3];

        SlotReading slot1 = evaluateSensor(hardware.color1, hardware.distance1, rearRGB1);
        SlotReading slot2 = evaluateSensor(hardware.color2, hardware.distance2, rearRGB2);
        SlotReading slot3 = evaluateSensor(hardware.color3, hardware.distance3, rearRGB3);

        slotStatuses[0] = slot1.status;
        slotStatuses[1] = slot2.status;
        slotStatuses[2] = slot3.status;

        maybeLogStatusChange();

        Telemetry telemetry = hardware.getTelemetry();
        telemetry.addLine("--- ARTIFACT TRACKER ---");
        telemetry.addData("Slot 1", formatSlotTelemetry(slot1));
        telemetry.addData("Slot 2", formatSlotTelemetry(slot2));
        telemetry.addData("Slot 3", formatSlotTelemetry(slot3));
        telemetry.addLine("-------------------------------------");
    }

    private SlotReading evaluateSensor(ColorSensor colorSensor, DistanceSensor distanceSensor, RGBIndicator indicator) {
        if (colorSensor == null || distanceSensor == null) {
            if (indicator != null) indicator.setColor(RGBIndicator.LEDColors.OFF);
            return SlotReading.missing();
        }

        double distance = 0;
        double red = 0;
        double green = 0;
        double blue = 0;

        boolean validSample = false;

        double dist = distanceSensor.getDistance(DistanceUnit.MM);
        if (!Double.isNaN(dist)) {
            distance = dist;
            red = colorSensor.red();
            green = colorSensor.green();
            blue = colorSensor.blue();
            validSample = true;
        }

        if (!validSample) {
            if (indicator != null) indicator.setColor(RGBIndicator.LEDColors.OFF);
            return new SlotReading(SlotStatus.VACANT, 0, 0, 0, Double.NaN);
        }

        double total = red + green + blue;
        SlotStatus status = SlotStatus.VACANT;

        if (distance <= Constants.COLOR_SENSOR_DETECTION_DISTANCE_MM && total > 0) {
            double redRatio = red / total;
            double greenRatio = green / total;
            double blueRatio = blue / total;

            // minimum brightness level for if it detects low light
            if (total >= 40) {
                if (blueRatio > greenRatio && greenRatio > redRatio && blue >= Constants.COLOR_SENSOR_PURPLE_RATIO * Math.max(green, red)) {
                    status = SlotStatus.PURPLE;
                } else if (greenRatio > blueRatio && greenRatio > redRatio && green >= Constants.COLOR_SENSOR_GREEN_BLUE_RATIO * blue && green >= Constants.COLOR_SENSOR_GREEN_RED_RATIO * red) {
                    status = SlotStatus.GREEN;
                }
            }
        }

        if (indicator != null) {
            switch (status) {
                case PURPLE:
                    indicator.setColor(RGBIndicator.LEDColors.VIOLET);
                    break;
                case GREEN:
                    indicator.setColor(RGBIndicator.LEDColors.GREEN);
                    break;
                default:
                    indicator.setColor(RGBIndicator.LEDColors.OFF);
                    break;
            }
        }

        return new SlotReading(status, red, green, blue, distance);
    }

    private String formatSlotTelemetry(SlotReading reading) {
        return String.format("%s | R: %s G: %s B: %s | D: %smm",
                reading.status,
                reading.redText,
                reading.greenText,
                reading.blueText,
                reading.distanceText);
    }

    private void maybeLogStatusChange() {
        boolean changed = lastLoggedStatuses == null
                || lastLoggedStatuses.length != slotStatuses.length;
        if (!changed) {
            for (int i = 0; i < slotStatuses.length; i++) {
                if (lastLoggedStatuses[i] != slotStatuses[i]) {
                    changed = true;
                    break;
                }
            }
        }

        if (changed) {
            RobotLog.ii("ArtifactTracker", "Detected Artifacts: %s, %s, %s",
                    slotStatuses[0], slotStatuses[1], slotStatuses[2]);
            lastLoggedStatuses = slotStatuses.clone();
        }
    }

    private static class SlotReading {
        final SlotStatus status;
        final double red;
        final double green;
        final double blue;
        final double distanceMm;
        final String redText;
        final String greenText;
        final String blueText;
        final String distanceText;

        SlotReading(SlotStatus status, double red, double green, double blue, double distanceMm) {
            this.status = status;
            this.red = red;
            this.green = green;
            this.blue = blue;
            this.distanceMm = distanceMm;
            this.redText = formatChannel(red);
            this.greenText = formatChannel(green);
            this.blueText = formatChannel(blue);
            this.distanceText = Double.isNaN(distanceMm) ? "--" : String.format(Locale.US, "%.1f", distanceMm);
        }

        static SlotReading missing() {
            return new SlotReading(SlotStatus.VACANT, Double.NaN, Double.NaN, Double.NaN, Double.NaN);
        }

        private static String formatChannel(double value) {
            return Double.isNaN(value) ? "--" : String.format(Locale.US, "%.0f", value);
        }
    }
}
