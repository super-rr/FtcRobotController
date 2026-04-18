package org.firstinspires.ftc.teamcode.hardware;

import com.bylazar.configurables.PanelsConfigurables;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.configs.Constants;
import org.firstinspires.ftc.teamcode.configs.FlywheelPidfConfig;
import org.firstinspires.ftc.teamcode.configs.PedroConstants;
import org.firstinspires.ftc.teamcode.configs.TurretAimConfig;
import org.firstinspires.ftc.teamcode.drivers.RGBIndicator;
import org.firstinspires.ftc.teamcode.groups.DcMotorExGroup;
import org.firstinspires.ftc.teamcode.groups.RGBIndicatorGroup;
import org.firstinspires.ftc.teamcode.groups.LauncherGroup;

public class RobotHardware {
    /// Systems
    private final LinearOpMode linearOpMode;
    private TelemetryManager telemetryManager;

    /// Variables
    private double powerDampener = 1.0;
    private AllianceColor allianceColor = AllianceColor.UNKNOWN;
    private double headingOffsetRadians = 0.0;
    private LLResult latestLimelightResult;

    /// Constants
    public final FlywheelPidfConfig flywheelPidfConfig = new FlywheelPidfConfig();
    public final TurretAimConfig turretAimConfig = new TurretAimConfig();

    /// Enums
    // Group Enums
    public interface RGBIndicatorPosition {
        int MAIN_RGB_INDICATOR = 0;
        int FRONT_LED = 1;
        int REAR_RGB_1 = 2;
        int REAR_RGB_2 = 3;
        int REAR_RGB_3 = 4;
    }

    public interface WheelsPosition {
        int FRONT = 0;
        int BACK = 1;
    }

    // Main Enums

    public enum AllianceColor {
        UNKNOWN,
        RED,
        BLUE
    }

    public enum IntakeDirection {
        IN,
        OUT,
        STOP
    }

    /// Devices
    public DcMotorExGroup leftSide;
    public DcMotorExGroup rightSide;
    public RGBIndicatorGroup rgbIndicatorGroup;
    public DcMotorEx intake;
    public DcMotorEx turret;
    public LauncherGroup launcher;
    public Servo spindexer;

    public Servo kicker;
    public Servo headlight;
    public ColorSensor color1;
    public DistanceSensor distance1;
    public ColorSensor color2;
    public DistanceSensor distance2;
    public ColorSensor color3;
    public DistanceSensor distance3;

    public GoBildaPinpointDriver pinpoint;
    public Limelight3A limelight;
    private DigitalChannel allianceButton;

    public void init() {
        // Get left and right wheels
        leftSide = new DcMotorExGroup(
                getMotor("leftFront"), // index 0
                getMotor("leftBack")   // index 1
        );
        rightSide = new DcMotorExGroup(
                getMotor("rightFront"), // index 0
                getMotor("rightBack")   // index 1
        );

        rgbIndicatorGroup = new RGBIndicatorGroup(
                new RGBIndicator(getServo("rgbLight")), // index 0
                new RGBIndicator(getServo("frontLED")), // index 1
                new RGBIndicator(getServo("rearRGB1")), // index 2
                new RGBIndicator(getServo("rearRGB2")), // index 3
                new RGBIndicator(getServo("rearRGB3"))  // index 4
        );
        rgbIndicatorGroup.setColor(RGBIndicator.LEDColors.YELLOW);

        // Alliance button
        allianceButton = linearOpMode.hardwareMap.get(DigitalChannel.class, "allianceButton");
        allianceButton.setMode(DigitalChannel.Mode.INPUT);
        configureAllianceFromSwitch();

        // Pinpoint
        pinpoint = linearOpMode.hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        double forwardPodOffsetInches = PedroConstants.FORWARD_POD_Y;
        double strafePodOffsetInches = PedroConstants.STRAFE_POD_X;
        pinpoint.setOffsets(forwardPodOffsetInches, strafePodOffsetInches, DistanceUnit.INCH);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.REVERSED);
        pinpoint.resetPosAndIMU();

        // Enable brake mode
        leftSide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightSide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Make it run using the encoder
        leftSide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightSide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Flip the direction of the right-side to correct movement
        rightSide.setDirection(DcMotorSimple.Direction.REVERSE);

        // Intake
        intake = getMotor("intake");
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Launcher
        launcher = new LauncherGroup(this, flywheelPidfConfig, new DcMotorExGroup(
                getMotor("launcher"),
                getMotor("launcher2")
        ));

        // Turret
        turret = getMotor("turret");
        turret.setDirection(DcMotorSimple.Direction.REVERSE);
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turret.setTargetPositionTolerance(5);

        // Spindexer
        spindexer = getServo("spindexer");
        spindexer.setPosition(Constants.SPINDEXER_1);

        // Kicker
        kicker = getServo("kicker");
        kicker.setPosition(Constants.KICKER_DOWN);

        // Headlight
        headlight = getServo("headlight");
        headlight.setPosition(0.0);

        // Limelight
        limelight = linearOpMode.hardwareMap.get(Limelight3A.class, "limelight");
        selectAllianceLimelightPipeline();
        limelight.start();
        refreshLimelightResult();

        // Color sensors
        color1 = getColorSensor("color1");
        distance1 = getDistanceSensor("color1");
        color2 = getColorSensor("color2");
        distance2 = getDistanceSensor("color2");
        color3 = getColorSensor("color3");
        distance3 = getDistanceSensor("color3");

        // Config
        PanelsConfigurables.INSTANCE.refreshClass(flywheelPidfConfig);
        PanelsConfigurables.INSTANCE.refreshClass(turretAimConfig);
        launcher.refreshLauncherPIDFFromConfig();
        flushPanels();

        // Telemetry
        linearOpMode.telemetry.addData("Status", "Initialized");
        linearOpMode.telemetry.addData("X-Offset", pinpoint.getXOffset(DistanceUnit.MM));
        linearOpMode.telemetry.addData("Y-Offset", pinpoint.getYOffset(DistanceUnit.MM));
        linearOpMode.telemetry.addData("Device Version Number", pinpoint.getDeviceVersion());
        linearOpMode.telemetry.addData("Device Scalar", pinpoint.getYawScalar());
        linearOpMode.telemetry.update();
    }

    public RobotHardware(LinearOpMode linearOpMode) {
        this.linearOpMode = linearOpMode;
    }

    public Telemetry getTelemetry() {
        return linearOpMode.telemetry;
    }

    public TelemetryManager getPanels() {
        if (telemetryManager == null) telemetryManager = PanelsTelemetry.INSTANCE.getTelemetry();
        return telemetryManager;
    }

    public void flushPanels() {
        TelemetryManager panels = getPanels();
        panels.update(linearOpMode.telemetry);
    }

    public void flushPanels(Telemetry telemetry) {
        TelemetryManager panels = getPanels();
        panels.update(telemetry);
    }

    /// Alliance Button Methods - NOTE: **2026** only probably, should be updated according to the design
    public void configureAllianceFromSwitch() {
        refreshAllianceFromSwitchState();
        rgbIndicatorGroup.setColor(getAllianceLEDColor());
    }

    public void refreshAllianceFromSwitchState() {
        allianceColor = AllianceColor.UNKNOWN;
        if (allianceButton != null && allianceButton.getMode() == DigitalChannel.Mode.INPUT) {
            boolean rawSwitchState = allianceButton.getState();
            if (rawSwitchState) allianceColor = AllianceColor.BLUE;
            else allianceColor = AllianceColor.RED;
        }
    }

    public double getAllianceLEDColor() {
        switch (allianceColor) {
            case RED: return RGBIndicator.LEDColors.RED;
            case BLUE: return RGBIndicator.LEDColors.BLUE;
            default: return RGBIndicator.LEDColors.WHITE;
        }
    }

    public void selectAllianceLimelightPipeline() {
        if (limelight == null) {
            linearOpMode.telemetry.addLine("** ERROR: Limelight not initialized");
            return;
        }

        boolean isRed = allianceColor == AllianceColor.RED;

        // Default to blue if the switch is absent or read as low
        int pipeline = isRed ? 4 : 0;
        limelight.pipelineSwitch(pipeline);
        boolean rawSwitchState = allianceButton != null && allianceButton.getState();
        linearOpMode.telemetry.addData("Alliance switch state (raw)", rawSwitchState);
        linearOpMode.telemetry.addData("Alliance inferred", isRed ? "RED" : "BLUE");
        linearOpMode.telemetry.addData("Selected pipeline", pipeline);
    }

    public void updateHeadingOffsetFromAllianceButton() {
        refreshAllianceFromSwitchState();
        boolean isRed = allianceColor == AllianceColor.RED;
        headingOffsetRadians = isRed ? -Math.PI / 2.0 : Math.PI / 2.0;
    }

    public double getHeadingOffsetRadians() {
        return headingOffsetRadians;
    }

    public double applyHeadingOffset(double headingRadians) {
        return headingRadians + headingOffsetRadians;
    }

    public AllianceColor getAllianceColor() {
        return allianceColor;
    }

    /// Movement Methods
    public void fieldCentricDrive(double x, double y, double rx, double botHeading){
        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);
        rotX *= 1.1;

        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        double leftFrontPower = (rotY + rotX + rx) / denominator;
        double leftBackPower = (rotY - rotX + rx) / denominator;
        double rightFrontPower = (rotY - rotX - rx) / denominator;
        double rightBackPower = (rotY + rotX - rx) / denominator;

        setDrivePower(leftFrontPower, rightFrontPower, leftBackPower, rightBackPower);
    }

    public void setDrivePower(double leftFrontPower, double rightFrontPower, double leftBackPower, double rightBackPower) {
        DcMotorEx leftFront = leftSide.motors[0];
        DcMotorEx leftBack = leftSide.motors[1];
        DcMotorEx rightFront = rightSide.motors[0];
        DcMotorEx rightBack = rightSide.motors[1];

        leftFront.setPower(leftFrontPower * powerDampener);
        rightFront.setPower(rightFrontPower * powerDampener);
        leftBack.setPower(leftBackPower * powerDampener);
        rightBack.setPower(rightBackPower * powerDampener);
    }

    public void setPowerDampener(double pwr) {
        if (powerDampener == pwr) powerDampener = 1;
        else powerDampener = pwr;
    }

    public double getPowerDampener() {
        return powerDampener;
    }

    /// Limelight
    public void refreshLimelightResult() {
        if (limelight == null) {
            latestLimelightResult = null;
            linearOpMode.telemetry.addLine("** ERROR: Limelight not initialized");
            return;
        }
        latestLimelightResult = limelight.getLatestResult();
    }

    /// Intake
    public void runIntake(IntakeDirection direction) {
        switch (direction) {
            case OUT:
                intake.setPower(-Constants.INTAKE_POWER);
                break;
            case IN:
                intake.setPower(Constants.INTAKE_POWER);
                break;
            default:
                intake.setPower(0.0);
                break;
        }
    }

    /// Lights
    public void setRgbIndicator(int rgbIndicatorPosition, double pwmValue) {
        RGBIndicator rgbIndicator = rgbIndicatorGroup.rgbIndicators[rgbIndicatorPosition];
        if (rgbIndicator != null) rgbIndicator.setColor(pwmValue);
    }

    public void setMainRGBIndicator(double pwmValue) {
        setRgbIndicator(RGBIndicatorPosition.MAIN_RGB_INDICATOR, pwmValue);
    }

    public void setFrontRGBIndicator(double pwmValue) {
        setRgbIndicator(RGBIndicatorPosition.FRONT_LED, pwmValue);
    }

    public void setRearRGBIndicators(double pwmValue) {
        setRgbIndicator(RGBIndicatorPosition.REAR_RGB_1, pwmValue);
        setRgbIndicator(RGBIndicatorPosition.REAR_RGB_2, pwmValue);
        setRgbIndicator(RGBIndicatorPosition.REAR_RGB_3, pwmValue);
    }

    public LLResult getLatestLimelightResult() {
        return latestLimelightResult;
    }

    /// Helper Methods
    public DcMotorEx getMotor(String deviceName) {
        return linearOpMode.hardwareMap.get(DcMotorEx.class, deviceName);
    }

    public Servo getServo(String deviceName) {
        return linearOpMode.hardwareMap.get(Servo.class, deviceName);
    }

    public RevColorSensorV3 getColorSensor(String deviceName) {
        return linearOpMode.hardwareMap.get(RevColorSensorV3.class, deviceName);
    }

    public DistanceSensor getDistanceSensor(String deviceName) {
        return linearOpMode.hardwareMap.get(DistanceSensor.class, deviceName);
    }
}