package org.firstinspires.ftc.teamcode.Mechanisms;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class SortingWithColor {
    private NormalizedColorSensor colorSensor;


    StorageConfig sorter = new StorageConfig();
    private DistanceSensor distance;

    public enum DetectedColor {
        PURPLE(1),
        GREEN(2),
        UNKNOWN(3);

        private final int code;

        DetectedColor(int code) {
            this.code = code;
        }

        public int getCode() {
            return code;
        }
    }

    public void init(HardwareMap hwMap) {
        colorSensor = hwMap.get(NormalizedColorSensor.class, "colorSensor");
        distance = hwMap.get(DistanceSensor.class , "distance");
    }


    public DetectedColor getDetectedColor(Telemetry telemetry) {

        NormalizedRGBA colors = colorSensor.getNormalizedColors();
        float r = colors.red;
        float g = colors.green;
        float b = colors.blue;

        float total = r + g + b;

        float redRatio = r / total;
        float greenRatio = g / total;
        float blueRatio = b / total;

        telemetry.addData("Red Ratio", redRatio);
        telemetry.addData("Green Ratio", greenRatio);
        telemetry.addData("Blue Ratio", blueRatio);
        if (GetDistance() < 12.0) {

            if (greenRatio > 0.4 && greenRatio < 0.45 && redRatio > 0.13 && redRatio < 0.20) {
                return DetectedColor.GREEN;
            } else {
                return DetectedColor.PURPLE;
            }
        }
        return DetectedColor.UNKNOWN;
    }

    public double GetDistance(){
        return distance.getDistance(DistanceUnit.CM);
    }

}
