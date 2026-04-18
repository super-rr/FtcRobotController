package org.firstinspires.ftc.teamcode.groups;

import com.qualcomm.robotcore.hardware.Servo;

import java.util.function.Consumer;

public class ServoGroup {
    public final Servo[] servos;
    private boolean initialized = false;

    public ServoGroup(Servo... servos) {
        if (servos.length > 0) {
            this.servos = servos;
            initialized = true;
        } else this.servos = new Servo[]{}; // if its empty create the array but don't init.
    }

    public void from(int min, int max, Consumer<Servo> action) {
        for (int i = min; i <= max; i++) {
            Servo servo = servos[i];
            if (servo != null) action.accept(servo);
        }
    }

    public void from(int min, Consumer<Servo> action) {
        from(min, servos.length, action);
    }

    public void from(Consumer<Servo> action) {
        from(0, servos.length, action);
    }

    /// From Servo.java

    public void setDirection(Servo.Direction direction) {
        if (!initialized) return;
        for (Servo servo : servos) servo.setDirection(direction);
    }
    public void setPosition(double position) {
        if (!initialized) return;
        for (Servo servo : servos) servo.setPosition(position);
    }

    public void scaleRange(double min, double max) {
        if (!initialized) return;
        for (Servo servo : servos) servo.scaleRange(min, max);
    }
}
