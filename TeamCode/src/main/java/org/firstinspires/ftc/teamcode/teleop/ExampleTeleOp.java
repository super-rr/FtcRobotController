package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.teamcode.configs.Constants;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.hardware.subsystems.FlywheelController;
import org.firstinspires.ftc.teamcode.hardware.subsystems.ShootingController;
import org.firstinspires.ftc.teamcode.hardware.subsystems.SpindexerController;
import org.firstinspires.ftc.teamcode.hardware.subsystems.TurretTracker;

import java.util.Locale;

@TeleOp(name = "Example", group = "TeleOp")
public class ExampleTeleOp extends LinearOpMode {
    final RobotHardware hardware = new RobotHardware(this);
    TurretTracker turretTracker;
    FlywheelController flywheelController;
    SpindexerController spindexerController;
    ShootingController shootingController;

    private void onInit() {
        hardware.init();
        hardware.updateHeadingOffsetFromAllianceButton();

        turretTracker = new TurretTracker(hardware);
        flywheelController = new FlywheelController(hardware);
        spindexerController = new SpindexerController(hardware);
        shootingController = new ShootingController(hardware, flywheelController, spindexerController);

        spindexerController.init();
    }

    private void onLoop() {
        hardware.refreshLimelightResult();

        //Limelight Data
        LLResult result = hardware.getLatestLimelightResult();
        if (result != null) {
            if (result.isValid()) {
                Pose3D botPose = result.getBotpose();
                telemetry.addData("tx/ty", "tx: %.2f ty: %.2f", result.getTx(), result.getTy());
                telemetry.addData("Bot Pose", botPose.toString());
            }
        }

        //Odometry
        hardware.pinpoint.update(); //Update odometry
        Pose2D pos = hardware.pinpoint.getPosition();
        String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getX(DistanceUnit.INCH), pos.getY(DistanceUnit.INCH), pos.getHeading(AngleUnit.DEGREES));
        telemetry.addLine("--- hardware DATA ---");

        telemetry.addData("Position", data);
        double VelX = hardware.pinpoint.getVelX(DistanceUnit.MM);
        double VelY = hardware.pinpoint.getVelY(DistanceUnit.MM);
        double headingVel = hardware.pinpoint.getHeadingVelocity(UnnormalizedAngleUnit.DEGREES);

        telemetry.addData("Wheel Power Dampening", hardware.getPowerDampener());
        telemetry.addData("Velocities (mm/s,deg/s)", "X: %.0f  Y: %.0f  H: %.1f", VelX, VelY, headingVel);
        telemetry.addLine("---------------------------");

        hardware.updateHeadingOffsetFromAllianceButton();
        double botHeading = hardware.pinpoint.getHeading(AngleUnit.RADIANS);
        double adjustedHeading = hardware.applyHeadingOffset(botHeading);

        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x;

        hardware.fieldCentricDrive(x, y, rx, adjustedHeading);

        /// DPad Gamepad 1
        if (gamepad1.dpadUpWasPressed()) {
            flywheelController.adjustRpmTolerance(10.0);
        }

        if (gamepad1.dpadDownWasPressed()) {
            flywheelController.adjustRpmTolerance(-10.0);
        }

        if (gamepad1.dpadLeftWasPressed()) {
            flywheelController.adjustLauncherFeedforward(1.0);
        }

        if (gamepad1.dpadRightWasPressed()) {
            flywheelController.adjustLauncherFeedforward(-1.0);
        }

        if (gamepad1.leftStickButtonWasPressed()) {
            hardware.setPowerDampener(0.5);
        }

        /// GAMEPAD 2

        /// Gamepad 2 Intake
        boolean intakeIn = gamepad2.right_bumper; //Check if button is currently held
        boolean intakeOut = gamepad2.left_bumper; //Check if button is currently held

        if (intakeIn){
            if (!spindexerController.isEnabled() || !spindexerController.isSpindexerFull()) {
                hardware.runIntake(RobotHardware.IntakeDirection.IN);
            }
        } else if (intakeOut) {
            hardware.runIntake(RobotHardware.IntakeDirection.OUT);
        } else {
            hardware.runIntake(RobotHardware.IntakeDirection.STOP);
        }

        /// Gamepad 2 Sticks
        if (gamepad2.leftStickButtonWasPressed()) {
            spindexerController.advanceSpindexer();
        }
        if (gamepad2.rightStickButtonWasPressed()) {
            spindexerController.reverseSpindexer();
        }

        /// Gamepad 2 FlyWheel toggle
        if (gamepad2.backWasPressed()) {
            flywheelController.toggle();
        }

        /// Gamepad 2 DPad
        if (gamepad2.dpadUpWasPressed()) {
            spindexerController.toggleAuto();
        }

        /// Gamepad 2 Tracking
        boolean trackingActive = flywheelController.isEnabled() || gamepad2.start;
        if (trackingActive) {
            turretTracker.update();
            hardware.headlight.setPosition(Constants.headlightPower);
        } else {
            hardware.turret.setPower(0);
            hardware.headlight.setPosition(0.0);
        }

        /// Gamepad 2 Shoot Sequence
        if (gamepad2.aWasPressed() && shootingController.isIdle()
                && flywheelController.isEnabled() && flywheelController.getTargetRpm() > 0) {
            shootingController.startShootSequence();
        }

        /// Gamepad 1 & 2 Spindexer Manuals
        if (shootingController.isIdle()) {
            // Kicker Manual
            if (gamepad1.a) {
                hardware.kicker.setPosition(Constants.KICKER_UP);
            } else {
                hardware.kicker.setPosition(Constants.KICKER_DOWN);
            }

            // Spindexer Manual
            if (gamepad2.b) {
                spindexerController.setPosition(0);
            } else if (gamepad2.y) {
                spindexerController.setPosition(1);
            } else if (gamepad2.x) {
                spindexerController.setPosition(2);
            }
        }

        ///  Gamepad 1 Toggle Kicker Stand
        flywheelController.update();
        spindexerController.update();
        shootingController.update(false);

        telemetry.addLine("--- FLYWHEEL DATA ---");
        telemetry.addData("Flywheel Tolerance", "%.0f rpm", flywheelController.getRpmTolerance());
        telemetry.addData("Launcher F", "%.0f", hardware.flywheelPidfConfig.launcherF);
        telemetry.addLine("--------------------------------");
        hardware.flushPanels(telemetry);
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
