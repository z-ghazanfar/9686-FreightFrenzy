package org.firstinspires.ftc.teamcode;


import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "GetMotorRPM")
public class MotorTest extends LinearOpMode {

    Motor motar;

    @Override
    public void runOpMode() throws InterruptedException {
        motar = new Motor(hardwareMap, "testMotor");

        telemetry.addData("RPM", motar.getMaxRPM());
    }
}
