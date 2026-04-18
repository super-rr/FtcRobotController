package org.firstinspires.ftc.teamcode.hardware;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;

import org.firstinspires.ftc.teamcode.configs.AutoPaths;
import org.firstinspires.ftc.teamcode.configs.AutoPoses;
import org.firstinspires.ftc.teamcode.hardware.subsystems.FindGoal;
import org.firstinspires.ftc.teamcode.hardware.subsystems.FlywheelController;
import org.firstinspires.ftc.teamcode.hardware.subsystems.ShootingController;
import org.firstinspires.ftc.teamcode.hardware.subsystems.SpindexerController;
import org.firstinspires.ftc.teamcode.hardware.subsystems.TurretTracker;

public class StateMachine {
    public enum State {
        STOP,
        HOME,
        AUTO_HOME_NEAR,
        AUTO_NEAR,
        AUTO_HOME_FAR,
        AUTO_FAR,
        AUTO_LEAVE_NEAR,
        AUTO_LEAVE_FAR,
    }

    private enum NearStep {
        START_FLYWHEEL_AND_DRIVE,
        FIND_GOAL,
        SHOOT_FIRST,
        DRIVE_TO_SPIKE1_LINEUP,
        INTAKE_SPIKE1_PART1,
        INTAKE_SPIKE1_PART2,
        DRIVE_TO_SHOOT_SPIKE1,
        SHOOT_SECOND,
        LEAVE,
        DONE,
    }

    private enum FarStep {
        START_FLYWHEEL_AND_DRIVE,
        FIND_GOAL,
        SHOOT_FIRST,
        DRIVE_TO_SPIKE3_LINEUP,
        INTAKE_SPIKE3_PART1,
        SPINDEXER_ADVANCE,
        SPINDEXER_SETTLE,
        INTAKE_SPIKE3_PART2,
        DRIVE_TO_SHOOT_SPIKE3,
        SHOOT_SECOND,
        LEAVE,
        DONE,
    }

    private static final double SAFETY_CUTOFF_SECONDS = 28.75;

    private State currentState;
    private NearStep nearStep = NearStep.START_FLYWHEEL_AND_DRIVE;
    private FarStep farStep = FarStep.START_FLYWHEEL_AND_DRIVE;
    private boolean shootStarted = false;

    private final RobotHardware hardware;
    private final Follower follower;
    private final FlywheelController flywheelController;
    private final ShootingController shootingController;
    private final SpindexerController spindexerController;
    private final TurretTracker turretTracker;
    private final FindGoal findGoal;
    private final AutoPaths paths;

    private final Timer pathTimer = new Timer();
    private final Timer autoTimer = new Timer();

    public StateMachine(RobotHardware hardware, Follower follower, FlywheelController flywheelController, ShootingController shootingController, SpindexerController spindexerController, TurretTracker turretTracker, boolean isBlue) {
        this.hardware = hardware;
        this.follower = follower;
        this.flywheelController = flywheelController;
        this.shootingController = shootingController;
        this.spindexerController = spindexerController;
        this.turretTracker = turretTracker;
        this.findGoal = new FindGoal(hardware);
        this.paths = new AutoPaths(follower, isBlue);
        setState(State.HOME, true);
    }

    public void setState(State state, boolean doUpdate) {
        currentState = state;
        nearStep = NearStep.START_FLYWHEEL_AND_DRIVE;
        farStep = FarStep.START_FLYWHEEL_AND_DRIVE;
        shootStarted = false;
        pathTimer.resetTimer();
        if (doUpdate) update();
    }
    public void setState(State state) { setState(state, false); }
    public State getState() { return currentState; }

    public void update() {
        hardware.refreshLimelightResult();
        if (currentState == State.AUTO_NEAR || currentState == State.AUTO_FAR) {
            if (autoTimer.getElapsedTimeSeconds() >= SAFETY_CUTOFF_SECONDS) {
                emergencyStop();
                return;
            }
        }

        switch (currentState) {
            case HOME:
                spindexerController.setPosition(0);
                break;

            case AUTO_HOME_NEAR:
                setPose(AutoPoses.BLUE_NEAR_START, AutoPoses.RED_NEAR_START);
                break;

            case AUTO_HOME_FAR:
                setPose(AutoPoses.BLUE_FAR_START, AutoPoses.RED_FAR_START);
                break;

            case AUTO_NEAR:
                updateNear();
                break;

            case AUTO_FAR:
                updateFar();
                break;

            case AUTO_LEAVE_NEAR:
                follow(paths.nearShootToWall, true);
                setState(State.STOP);
                break;

            case AUTO_LEAVE_FAR:
                follow(paths.farShootLeave, true);
                setState(State.STOP);
                break;

            default:
                break;
        }
    }

    private void updateNear() {
        switch (nearStep) {
            case START_FLYWHEEL_AND_DRIVE:
                autoTimer.resetTimer();
                startFlywheel();
                follow(paths.nearStartToShoot, true);
                nearStep = NearStep.FIND_GOAL;
                break;

            case FIND_GOAL:
                turretTracker.update();
                flywheelController.update();
                if (findGoal.updateAndIsDone()) nearStep = NearStep.SHOOT_FIRST;
                break;

            case SHOOT_FIRST:
                flywheelController.update();
                if (pathDone()) {
                    if (shoot(paths.nearShootLeave, false)) {
                        nearStep = NearStep.DRIVE_TO_SPIKE1_LINEUP;
                    }
                }
                break;

            case DRIVE_TO_SPIKE1_LINEUP:
                if (pathDone()) {
                    follow(paths.nearShootToSpike1Lineup, true);
                    pathTimer.resetTimer();
                    nearStep = NearStep.INTAKE_SPIKE1_PART1;
                }
                break;

            case INTAKE_SPIKE1_PART1:
                if (pathDoneAfter(1.0)) {
                    hardware.runIntake(RobotHardware.IntakeDirection.IN);
                    follow(paths.nearSpike1Part1, true);
                    pathTimer.resetTimer();
                    nearStep = NearStep.INTAKE_SPIKE1_PART2;
                }
                break;

            case INTAKE_SPIKE1_PART2:
                if (pathDoneAfter(1.0)) {
                    follow(paths.nearSpike1Part2, true);
                    pathTimer.resetTimer();
                    nearStep = NearStep.DRIVE_TO_SHOOT_SPIKE1;
                }
                break;

            case DRIVE_TO_SHOOT_SPIKE1:
                if (pathDoneAfter(1.5)) {
                    hardware.runIntake(RobotHardware.IntakeDirection.STOP);
                    follow(paths.nearSpike1ToShoot, true);
                    nearStep = NearStep.SHOOT_SECOND;
                }
                break;

            case SHOOT_SECOND:
                flywheelController.update();
                turretTracker.update();
                if (pathDone()) {
                    if (shoot(paths.nearShootLeave, true)) {
                        stopFlywheel();
                        nearStep = NearStep.LEAVE;
                    }
                }
                break;

            case LEAVE:
                if (pathDone()) {
                    follow(paths.nearShootLeave, false);
                    nearStep = NearStep.DONE;
                }
                break;

            case DONE:
                if (pathDone()) setState(State.STOP);
                break;
        }
    }

    private void updateFar() {
        switch (farStep) {
            case START_FLYWHEEL_AND_DRIVE:
                autoTimer.resetTimer();
                startFlywheel();
                flywheelController.setLauncherFeedforward(29);
                follow(paths.farStartToShoot, true);
                farStep = FarStep.FIND_GOAL;
                break;

            case FIND_GOAL:
                turretTracker.update();
                flywheelController.update();
                if (findGoal.updateAndIsDone()) farStep = FarStep.SHOOT_FIRST;
                break;

            case SHOOT_FIRST:
                flywheelController.update();
                if (pathDone()) {
                    if (shoot(paths.farShootLeave, false)) {
                        hardware.runIntake(RobotHardware.IntakeDirection.IN);
                        follow(paths.farShootToSpike3Lineup, true);
                        farStep = FarStep.DRIVE_TO_SPIKE3_LINEUP;
                    }
                }
                break;

            case DRIVE_TO_SPIKE3_LINEUP:
                if (pathDone()) {
                    follow(paths.farSpike3Part1, true);
                    pathTimer.resetTimer();
                    farStep = FarStep.INTAKE_SPIKE3_PART1;
                }
                break;

            case INTAKE_SPIKE3_PART1:
                if (pathDoneAfter(2.0)) {
                    spindexerController.setPosition(2);
                    pathTimer.resetTimer();
                    farStep = FarStep.SPINDEXER_ADVANCE;
                }
                break;

            case SPINDEXER_ADVANCE:
                if (pathDoneAfter(1.25)) {
                    spindexerController.setPosition(1);
                    pathTimer.resetTimer();
                    farStep = FarStep.SPINDEXER_SETTLE;
                }
                break;

            case SPINDEXER_SETTLE:
                if (pathDoneAfter(1.375)) {
                    follow(paths.farSpike3Part2, true);
                    pathTimer.resetTimer();
                    farStep = FarStep.INTAKE_SPIKE3_PART2;
                }
                break;

            case INTAKE_SPIKE3_PART2:
                if (pathDoneAfter(1.45)) {
                    hardware.runIntake(RobotHardware.IntakeDirection.STOP);
                    follow(paths.farSpike3ToShoot, true);
                    farStep = FarStep.DRIVE_TO_SHOOT_SPIKE3;
                }
                break;

            case DRIVE_TO_SHOOT_SPIKE3:
                if (pathDone()) farStep = FarStep.SHOOT_SECOND;
                break;

            case SHOOT_SECOND:
                flywheelController.update();
                turretTracker.update();
                if (shoot(paths.farShootLeave, true)) {
                    follow(paths.farShootLeave, true);
                    stopFlywheel();
                    farStep = FarStep.LEAVE;
                }
                break;

            case LEAVE:
                farStep = FarStep.DONE;
                break;

            case DONE:
                if (pathDone()) setState(State.STOP);
                break;
        }
    }

    private boolean shoot(PathChain breakPath, boolean checkArtifacts) {
        if (breakPath != null && autoTimer.getElapsedTimeSeconds() >= SAFETY_CUTOFF_SECONDS) {
            follow(breakPath, true);
            return false;
        }

        turretTracker.update();
        flywheelController.update();

        if (!shootStarted
                && flywheelController.isEnabled()
                && flywheelController.getTargetRpm() > 0) {
            shootingController.startShootSequence();
            shootStarted = true;
        }

        boolean done = shootStarted && shootingController.updateAndIsComplete(checkArtifacts);
        if (done) shootStarted = false;
        return done;
    }

    private void startFlywheel() {
        if (!flywheelController.isEnabled()) flywheelController.toggle();
        flywheelController.update();
    }

    private void stopFlywheel() {
        if (flywheelController.isEnabled()) flywheelController.toggle();
    }

    private void emergencyStop() {
        hardware.runIntake(RobotHardware.IntakeDirection.STOP);
        spindexerController.setPosition(0);
        spindexerController.disableAuto();
        stopFlywheel();
        PathChain park = (currentState == State.AUTO_NEAR) ? paths.nearShootLeave : paths.farShootLeave;
        follow(park, true);
        setState(State.STOP);
    }

    private void follow(PathChain path, boolean hold) {
        if (path == null) return;
        follower.followPath(path, hold);
    }

    private void setPose(Pose blue, Pose red) {
        Pose pose = hardware.getAllianceColor() == RobotHardware.AllianceColor.BLUE ? blue : red;
        follower.setStartingPose(pose);
        follower.setPose(pose);
    }

    private boolean pathDone() {
        return !follower.isBusy();
    }

    private boolean pathDoneAfter(double seconds) {
        return pathDone() && pathTimer.getElapsedTimeSeconds() >= seconds;
    }
}
