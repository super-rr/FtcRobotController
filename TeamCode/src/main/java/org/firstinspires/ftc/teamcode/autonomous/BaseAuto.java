package org.firstinspires.ftc.teamcode.autonomous;

import com.bylazar.field.FieldManager;
import com.bylazar.field.PanelsField;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.configs.PedroConstants;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.hardware.StateMachine;
import org.firstinspires.ftc.teamcode.hardware.subsystems.Drawer;
import org.firstinspires.ftc.teamcode.hardware.subsystems.FlywheelController;
import org.firstinspires.ftc.teamcode.hardware.subsystems.ShootingController;
import org.firstinspires.ftc.teamcode.hardware.subsystems.SpindexerController;
import org.firstinspires.ftc.teamcode.hardware.subsystems.TurretTracker;

public abstract class BaseAuto extends LinearOpMode {
    protected final RobotHardware hardware = new RobotHardware(this);
    private TelemetryManager panelsTelemetry;
    private FieldManager panelsField;

    @Override
    public final void runOpMode() {
        panelsTelemetry = hardware.getPanels();
        panelsField = PanelsField.INSTANCE.getField();
        panelsField.setOffsets(PanelsField.INSTANCE.getPresets().getPEDRO_PATHING());

        hardware.init();

        FlywheelController flywheelController = new FlywheelController(hardware);
        SpindexerController spindexerController = new SpindexerController(hardware);
        ShootingController shootingController = new ShootingController(hardware, flywheelController, spindexerController);
        TurretTracker turretTracker = new TurretTracker(hardware);
        Follower follower = PedroConstants.createFollower(hardwareMap);

        boolean isBlue = hardware.getAllianceColor() == RobotHardware.AllianceColor.BLUE;

        StateMachine stateMachine = new StateMachine(hardware, follower, flywheelController, shootingController, spindexerController, turretTracker, isBlue);

        Drawer drawer = new Drawer(panelsField, follower);

        spindexerController.init();
        stateMachine.setState(getHomeState(), true);
        drawer.draw();

        waitForStart();
        stateMachine.setState(getStartState());

        while (opModeIsActive()) {
            follower.update();
            stateMachine.update();
            flywheelController.update();
            spindexerController.update();
            drawer.draw();
            if (stateMachine.getState() == StateMachine.State.STOP) break;

            panelsTelemetry.debug("State", stateMachine.getState().name());
            panelsTelemetry.debug("Pose X", follower.getPose().getX());
            panelsTelemetry.debug("Pose Y", follower.getPose().getY());
            panelsTelemetry.debug("Heading", follower.getPose().getHeading());
            panelsTelemetry.update(telemetry);

            telemetry.addData("STATE", stateMachine.getState());
            telemetry.addData("X", "%.2f", follower.getPose().getX());
            telemetry.addData("Y", "%.2f", follower.getPose().getY());
            telemetry.addData("HEADING", "%.2f", Math.toDegrees(follower.getPose().getHeading()));
            telemetry.update();
        }
    }

    protected abstract StateMachine.State getHomeState();
    protected abstract StateMachine.State getStartState();
}