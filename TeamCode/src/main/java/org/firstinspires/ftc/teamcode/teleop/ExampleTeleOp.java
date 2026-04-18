package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;

import java.util.Locale;

@TeleOp(name = "Example", group = "TeleOp")
public class ExampleTeleOp extends LinearOpMode {
    final RobotHardware hardware = new RobotHardware(this);

    private void onInit() {
        hardware.init();
        hardware.updateHeadingOffsetFromAllianceButton();
    }

    private void onLoop() {
        hardware.pinpoint.update();
        Pose2D pos = hardware.pinpoint.getPosition();
        String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getX(DistanceUnit.INCH), pos.getY(DistanceUnit.INCH), pos.getHeading(AngleUnit.DEGREES));

        telemetry.addLine("--- ROBOT DATA ---");
        telemetry.addData("Position", data);
        double velX = hardware.pinpoint.getVelX(DistanceUnit.MM);
        double velY = hardware.pinpoint.getVelY(DistanceUnit.MM);
        double headingVel = hardware.pinpoint.getHeadingVelocity(UnnormalizedAngleUnit.DEGREES);

        telemetry.addData("Wheel Power Dampening", hardware.getPowerDampener());
        telemetry.addData("Velocities (mm/s,deg/s)", "X: %.0f  Y: %.0f  H: %.1f", velX, velY, headingVel);
        telemetry.addLine("---------------------------");

        double botHeading = hardware.pinpoint.getHeading(AngleUnit.RADIANS);
        double adjustedHeading = hardware.applyHeadingOffset(botHeading);

        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x;

        hardware.fieldCentricDrive(x, y, rx, adjustedHeading);

        // Left Stick - Power dampen by half, if pressed again will set back to 1.
        if (gamepad1.leftStickButtonWasPressed()) hardware.setPowerDampener(0.5);


        telemetry.update();
    }

    @Override
    public void runOpMode() {
        onInit();
        waitForStart();
        resetRuntime();
        while (opModeIsActive()) onLoop();
    }
}
