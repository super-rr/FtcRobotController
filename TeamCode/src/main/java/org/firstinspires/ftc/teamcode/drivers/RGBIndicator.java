package org.firstinspires.ftc.teamcode.drivers;

import com.qualcomm.robotcore.hardware.Servo;

public class RGBIndicator {
    public static class LEDColors {
        public static final double OFF = 0.0;
        public static final double RED = 0.279;
        public static final double ORANGE = 0.333;
        public static final double YELLOW = 0.388;
        public static final double SAGE = 0.444;
        public static final double GREEN = 0.500;
        public static final double AZURE = 0.555;
        public static final double BLUE = 0.611;
        public static final double INDIGO = 0.666;
        public static final double VIOLET = 0.722;
        public static final double WHITE = 1.0;
    }

    private final Servo light;

    public RGBIndicator(Servo servo) {
        this.light = servo;
    }

    public void setColor(double pwmValue) {
        if (light != null) light.setPosition(pwmValue);
    }

    public void turnOff() {
        setColor(LEDColors.OFF);
    }
}
