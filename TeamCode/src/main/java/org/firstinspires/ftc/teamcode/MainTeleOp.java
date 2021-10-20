package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.DriveCommmand;
import org.firstinspires.ftc.teamcode.commands.DuckySpinnerCommand;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DuckySpinnerSubsystem;

@TeleOp(name = "MainTeleOp-CB")
public class MainTeleOp extends CommandOpMode {

    private final double SLOW_MODE = 0.33;

    private Motor fL, bL, fR, bR;
    private MotorGroupTemp leftDrive, rightDrive;

    private Motor duckySpinner;

    private GamepadEx gPad1;
    private RevIMU imu;

    private DriveSubsystem driveSubsystem;
    private DuckySpinnerSubsystem duckySpinnerSubsystem;

    private DriveCommmand driveCommmand;
    private DuckySpinnerCommand duckySpinnerCommand;

    @Override
    public void initialize() {

        fL = new Motor(hardwareMap, "frontLeft");
        fR = new Motor(hardwareMap, "frontRight");
        bL = new Motor(hardwareMap, "backLeft");
        bR = new Motor(hardwareMap, "backRight");

        duckySpinner = new Motor(hardwareMap, "duckySpinner");

        leftDrive = new MotorGroupTemp(fL, bL);
        rightDrive = new MotorGroupTemp(fR, bR);

        gPad1 = new GamepadEx(gamepad1);
        imu = new RevIMU(hardwareMap);
        imu.init();

        driveSubsystem = new DriveSubsystem(leftDrive, rightDrive, imu);
        duckySpinnerSubsystem = new DuckySpinnerSubsystem(duckySpinner);
        duckySpinnerCommand = new DuckySpinnerCommand(duckySpinnerSubsystem);

        if(gPad1.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).get()) {
            driveCommmand = new DriveCommmand(driveSubsystem, gPad1::getLeftY, gPad1::getRightX);
        }

        gPad1.getGamepadButton(GamepadKeys.Button.B).whenHeld(duckySpinnerCommand);

        register(driveSubsystem);
        driveSubsystem.setDefaultCommand(driveCommmand);

    }
}
