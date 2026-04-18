package org.firstinspires.ftc.teamcode.groups;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.function.Consumer;

public class DcMotorExGroup {
    public final DcMotorEx[] motors;
    private boolean initialized = false;

    public DcMotorExGroup(DcMotorEx... motors) {
        if (motors.length > 0) {
            this.motors = motors;
            initialized = true;
        } else this.motors = new DcMotorEx[]{}; // if its empty create the array but don't init.
    }

    public void from(int min, int max, Consumer<DcMotorEx> action) {
        for (int i = min; i <= max; i++) {
            DcMotorEx motor = motors[i];
            if (motor != null) action.accept(motor);
        }
    }

    public void from(int min, Consumer<DcMotorEx> action) {
        from(min, motors.length, action);
    }

    public void from(Consumer<DcMotorEx> action) {
        from(0, motors.length, action);
    }

    /// From DcMotorEx.java

    public void enable() {
        if (!initialized) return;
        for (DcMotorEx motor : motors) if (motor != null) motor.setMotorEnable();
    }

    public void disable() {
        if (!initialized) return;
        for (DcMotorEx motor : motors) if (motor != null) motor.setMotorDisable();
    }

    public boolean isAllEnabled() {
        if (!initialized) return false;
        for (DcMotorEx motor : motors)  if (motor == null || !motor.isMotorEnabled()) return false;
        return true;
    }

    public void setVelocity(double angularRate) {
        if (!initialized) return;
        for (DcMotorEx motor : motors) motor.setVelocity(angularRate);
    }

    public double getVelocity() {
        if (!initialized) return 0;
        double sum = 0;
        for (DcMotorEx motor : motors) sum += motor.getVelocity();
        return sum / motors.length;
    }

    public double getVelocity(AngleUnit unit) {
        if (!initialized) return 0;
        double sum = 0;
        for (DcMotorEx motor : motors) sum += motor.getVelocity(unit);
        return sum / motors.length;
    }

    public void setVelocity(double angularRate, AngleUnit unit) {
        if (!initialized) return;
        for (DcMotorEx motor : motors) motor.setVelocity(angularRate, unit);
    }

    public void setPIDFCoefficients(DcMotor.RunMode mode, PIDFCoefficients pidfCoefficients) {
        if (!initialized) return;
        for (DcMotorEx motor : motors) motor.setPIDFCoefficients(mode, pidfCoefficients);
    }

    public void setVelocityPIDFCoefficients(double p, double i, double d, double f) {
        if (!initialized) return;
        for (DcMotorEx motor : motors) motor.setVelocityPIDFCoefficients(p, i, d, f);
    }

    public void setPositionPIDFCoefficients(double p) {
        if (!initialized) return;
        for (DcMotorEx motor : motors) motor.setPositionPIDFCoefficients(p);
    }

    public void setTargetPositionTolerance(int tolerance) {
        if (!initialized) return;
        for (DcMotorEx motor : motors) motor.setTargetPositionTolerance(tolerance);
    }

    /// From DcMotor.java
    public void setMotorType(MotorConfigurationType motorType) {
        if (!initialized) return;
        for (DcMotorEx motor : motors) motor.setMotorType(motorType);
    }

    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior zeroPowerBehavior) {
        if (!initialized) return;
        for (DcMotorEx motor : motors) motor.setZeroPowerBehavior(zeroPowerBehavior);
    }

    public void setTargetPosition(int position) {
        if (!initialized) return;
        for (DcMotorEx motor : motors) motor.setTargetPosition(position);
    }

    public boolean isBusy() {
        if (!initialized) return false;
        for (DcMotorEx motor : motors) if (motor.isBusy()) return true;
        return false;
    }

    public void setMode(DcMotor.RunMode mode) {
        if (!initialized) return;
        for (DcMotorEx motor : motors) motor.setMode(mode);
    }

    /// From DcMotorSimple

    public void setDirection(DcMotorSimple.Direction direction) {
        if (!initialized) return;
        for (DcMotorEx motor : motors) motor.setDirection(direction);
    }

    public void setPower(double power) {
        if (!initialized) return;
        for (DcMotorEx motor : motors) motor.setPower(power);
    }

    public double getPower() {
        if (!initialized) return 0;
        double sum = 0;
        for (DcMotorEx motor : motors) sum += motor.getPower();
        return sum / motors.length;
    }
}
