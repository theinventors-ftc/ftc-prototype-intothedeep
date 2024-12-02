package org.firstinspires.ftc.teamcode.PrototypeRobot.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.inventors.ftc.robotbase.hardware.ColorSensor;

public class IntakeSubsystem extends SubsystemBase {
    private static double power = 0.9;

    private CRServoImplEx rightIntake, leftIntake;
    private NormalizedColorSensor colorSensor;
    private DigitalChannel limitSwitch;

    public enum COLOR {
        NONE,
        RED,
        BLUE,
        YELLOW
    }

    private COLOR color;

    private Telemetry telemetry;

    public IntakeSubsystem(HardwareMap hm, Telemetry telemetry) {
        this.telemetry = telemetry;

        rightIntake = hm.get(CRServoImplEx.class, "rightwheel");
        leftIntake = hm.get(CRServoImplEx.class, "leftwheel");
        colorSensor = hm.get(NormalizedColorSensor.class, "color");
        colorSensor.setGain(100);
        limitSwitch = hm.get(DigitalChannel.class, "switch");

        color = COLOR.NONE;
    }

    @Override
    public void periodic() {
//        getSampleColor();
//        isSample();
    }

    public void run() {
        rightIntake.setPower(-power);
        leftIntake.setPower(power);
    }

    public void reverse() {
        rightIntake.setPower(power);
        leftIntake.setPower(-power);
    }

    public void brake_reverse() {
        rightIntake.setPower(0.4);
        leftIntake.setPower(-0.4);
    }

    public void stop() {
        rightIntake.setPower(0);
        leftIntake.setPower(0);
    }

    private COLOR predict(double r, double g, double b) {
        if(b < 60 && r > 90 && g > 90) return COLOR.YELLOW;
        if(b > 70 && r < 50 && g < 60) return COLOR.BLUE;
        if(b < 40 && g < 65 && r > 65) return COLOR.RED;
        return COLOR.NONE;
    }

    public COLOR getSampleColor() {
        NormalizedRGBA raw_colors = colorSensor.getNormalizedColors();
        double[] colors = {Math.floor(raw_colors.red*100), Math.floor(raw_colors.green*100), Math.floor(raw_colors.blue*100), Math.floor(raw_colors.alpha*100)};

        telemetry.addData("Red: ", colors[0]);
        telemetry.addData("Green: ", colors[1]);
        telemetry.addData("Blue: ", colors[2]);
        telemetry.addData("Color Prediction: ", predict(colors[0], colors[1], colors[2]));

        return predict(colors[0], colors[1], colors[2]);
    }

    public boolean isSample() {
        telemetry.addData("Is Sample: ", limitSwitch.getState());
        return limitSwitch.getState();
    }
}
