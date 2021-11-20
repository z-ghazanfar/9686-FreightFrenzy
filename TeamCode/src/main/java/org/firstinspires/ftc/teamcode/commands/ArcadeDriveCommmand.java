package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class ArcadeDriveCommmand extends CommandBase {

    private DriveSubsystem driveSubsystem;
    private DoubleSupplier fSpeed, tSpeed;
    private DoubleSupplier kSlow;

    private GamepadEx gPad;

    public ArcadeDriveCommmand(DriveSubsystem dSystem, DoubleSupplier forwardSpeed, DoubleSupplier turnSpeed, DoubleSupplier slowMode) {
        driveSubsystem = dSystem;
        fSpeed = forwardSpeed;
        tSpeed = turnSpeed;

        kSlow = slowMode;
        // Declares subsystem dependencies
        addRequirements(driveSubsystem);
    }

    @Override
    public void execute() {
        // Normal Way
        driveSubsystem.arcadeSlowDrive(fSpeed.getAsDouble() * 0.75 * kSlow.getAsDouble(), tSpeed.getAsDouble() * kSlow.getAsDouble());
    }


}
