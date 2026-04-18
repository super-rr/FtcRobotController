package org.firstinspires.ftc.teamcode.groups;

import org.firstinspires.ftc.teamcode.drivers.RGBIndicator;

import java.util.function.Consumer;

public class RGBIndicatorGroup {
    public final RGBIndicator[] rgbIndicators;
    private boolean initialized = false;

    public RGBIndicatorGroup(RGBIndicator... rgbIndicators) {
        if (rgbIndicators.length > 0) {
            this.rgbIndicators = rgbIndicators;
            initialized = true;
        } else this.rgbIndicators = new RGBIndicator[]{}; // if its empty create the array but don't init.
    }

    public void from(int min, int max, Consumer<RGBIndicator> action) {
        for (int i = min; i <= max; i++) {
            RGBIndicator rgbIndicator = rgbIndicators[i];
            if (rgbIndicator != null) action.accept(rgbIndicator);
        }
    }

    public void from(int min, Consumer<RGBIndicator> action) {
        from(min, rgbIndicators.length, action);
    }

    public void from(Consumer<RGBIndicator> action) {
        from(0, rgbIndicators.length, action);
    }

    public void setColor(double pwmValue) {
        if (!initialized) return;
        for (RGBIndicator light : rgbIndicators) light.setColor(pwmValue);
    }

    public void turnOff() {
        for (RGBIndicator light : rgbIndicators) light.setColor(RGBIndicator.LEDColors.OFF);
    }
}
