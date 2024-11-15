package org.firstinspires.ftc.teamcode.TeleOPs;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.inventors.ftc.robotbase.hardware.MotorExEx;

// 0 - > 90
// 90 - > 635

@TeleOp(name = "ArmAngleTest", group = "Tests")
public class ArmAngleTest extends LinearOpMode {
    public MotorExEx arm;

    public double mapping(double x, double in_min, double in_max, double out_min, double out_max) {
        return (x-in_min) * (out_max-out_min) / (in_max - in_min) + out_min;
    }

    @Override
    public void runOpMode() throws InterruptedException {
        arm = new MotorExEx(hardwareMap, "tilt", 383.6, 312);
        arm.resetEncoder();
        arm.setRunMode(Motor.RunMode.RawPower);

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            telemetry.addData("Ticks: ", arm.getCurrentPosition());
            telemetry.addData(
                    "Angle: ",
                    mapping(arm.getCurrentPosition(), 90, 635, 0, 90)
            );
            telemetry.update();
        }
    }
}
