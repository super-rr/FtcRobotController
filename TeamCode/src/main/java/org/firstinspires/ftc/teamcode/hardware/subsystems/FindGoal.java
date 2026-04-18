package org.firstinspires.ftc.teamcode.hardware.subsystems;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.configs.Constants;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;

import java.util.List;

public class FindGoal {
    private static final double SEARCH_POWER = 0.25;
    private static final double ALIGN_KP = 0.05;
    private static final double TARGET_TOLERANCE_PERCENT = 0.15;
    private static final double TARGET_TOLERANCE_DEGREES = 15.0;

    private final RobotHardware hardware;

    public FindGoal(RobotHardware hardware) {
        this.hardware = hardware;
    }

    public boolean updateAndIsDone() {
        DcMotorEx turret = hardware.turret;

        if (turret == null) return true;

        hardware.refreshLimelightResult();
        LLResult result = hardware.getLatestLimelightResult();

        boolean targetVisible = result != null && result.isValid();
        if (targetVisible) {
            List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
            targetVisible = fiducials != null && !fiducials.isEmpty();
        }

        double power;

        if (targetVisible) {
            double txPercent = result.getTx();
            double error;

            if (!Double.isNaN(txPercent)) {
                error = txPercent;
                if (Math.abs(error) <= TARGET_TOLERANCE_PERCENT) {
                    turret.setPower(0);
                    return true;
                }
            } else {
                error = result.getFiducialResults().get(0).getTargetXDegrees();
                if (Math.abs(error) <= TARGET_TOLERANCE_DEGREES) {
                    turret.setPower(0);
                    return true;
                }
            }

            power = ALIGN_KP * error;
        } else {
            boolean isBlue = hardware.getAllianceColor() == RobotHardware.AllianceColor.BLUE;
            power = isBlue ? -SEARCH_POWER : SEARCH_POWER;
        }

        power = Range.clip(power, -SEARCH_POWER, SEARCH_POWER);

        double pos = turret.getCurrentPosition();
        if ((pos <= Constants.TURRET_MIN && power < 0) || (pos >= Constants.TURRET_MAX && power > 0)) {
            turret.setPower(0);
            return false;
        }

        turret.setPower(power);
        return false;
    }
}
