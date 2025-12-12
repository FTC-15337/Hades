package org.firstinspires.ftc.teamcode.Mechanisms;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.List;

public class LimelightConfig {

    private Limelight3A limelight;


    public void init(HardwareMap hwMap) {
        limelight = hwMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();
    }

    public void autoInit(HardwareMap hwMap) {
        limelight = hwMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(8);
        limelight.start();
    }

    private LLResult getSafeResult() {
        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) {
            return null;
        }
        return result;
    }

    public boolean hasTarget() {
        LLResult result = getSafeResult();
        return result != null &&
                result.getFiducialResults() != null &&
                !result.getFiducialResults().isEmpty();
    }

    public int getId() {
        LLResult result = getSafeResult();
        if (result == null) return -1;

        List<LLResultTypes.FiducialResult> fid = result.getFiducialResults();
        if (fid == null || fid.isEmpty()) return -1;

        return fid.get(0).getFiducialId();
    }

    public double getTx() {
        LLResult result = getSafeResult();
        if (result == null) return 0;
        return result.getTx();
    }

    public void alignYaw(MecDrivebase drive) {

        if (!hasTarget()) {
            drive.drive(0, 0, 0);
            return;
        }

        double tx = getTx();

        double Kp = 0.03;
        double minPower = 0.10;
        double tolerance = 0.25;

        if (Math.abs(tx) > tolerance) {

            double rotate = tx * Kp;

            if (Math.abs(rotate) < minPower) {
                rotate = Math.copySign(minPower, rotate);
            }

            drive.drive(0, 0, rotate);
        }
        else {
            drive.drive(0, 0, 0);
        }
    }

    public void alignYawDirection(MecDrivebase drive, boolean forceLeft) {

        if (!hasTarget()) {
            drive.drive(0, 0, 0);
            return;
        }

        double tx = getTx();
        double Kp = 0.03;
        double minPower = 0.10;
        double tolerance = 1.0;

        double rotate = tx * Kp;

        if (Math.abs(rotate) < minPower) {
            rotate = Math.copySign(minPower, rotate);
        }

        rotate = forceLeft ? -Math.abs(rotate) : Math.abs(rotate);

        if (Math.abs(tx) > tolerance) {
            drive.drive(0, 0, rotate);
        }
        else {
            drive.drive(0, 0, 0);
        }
    }
}
