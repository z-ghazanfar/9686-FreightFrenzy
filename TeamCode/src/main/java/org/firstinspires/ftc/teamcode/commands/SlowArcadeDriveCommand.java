package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

import java.util.function.DoubleSupplier;

public class SlowArcadeDriveCommand extends CommandBase {

    private DriveSubsystem slowArcadeSubsystem;
    private DoubleSupplier fSpeed, tSpeed;

    private double SLOW_COEFF;

    public SlowArcadeDriveCommand(DriveSubsystem driveSubsystem, DoubleSupplier forwardSpeed, DoubleSupplier turnSpeed, double slowCoefficient) {
        slowArcadeSubsystem = driveSubsystem;

        fSpeed = forwardSpeed;
        tSpeed = turnSpeed;

        SLOW_COEFF = slowCoefficient;

        addRequirements(driveSubsystem);
    }

    @Override
    public void execute() {
        slowArcadeSubsystem.arcadeDrive(fSpeed.getAsDouble() * SLOW_COEFF, tSpeed.getAsDouble() * SLOW_COEFF);
    }


}
