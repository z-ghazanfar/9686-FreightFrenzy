package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

import java.util.function.DoubleSupplier;

public class DriveCommmand extends CommandBase {

    private DriveSubsystem driveSubsystem;
    private DoubleSupplier fSpeed, tSpeed;
    private double slowCoefficient;

    public DriveCommmand(DriveSubsystem dSystem, DoubleSupplier forwardSpeed, DoubleSupplier turnSpeed) {
        driveSubsystem = dSystem;
        fSpeed = forwardSpeed;
        tSpeed = turnSpeed;

        // Declares subsystem dependencies
        addRequirements(driveSubsystem);
    }

    @Override
    public void execute() {
        driveSubsystem.drive(fSpeed.getAsDouble(), tSpeed.getAsDouble());
    }

}
