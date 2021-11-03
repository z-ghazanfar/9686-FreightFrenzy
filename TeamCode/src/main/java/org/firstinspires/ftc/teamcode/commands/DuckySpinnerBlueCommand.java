package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.DuckySpinnerSubsystem;

public class DuckySpinnerBlueCommand extends CommandBase {

    private DuckySpinnerSubsystem dSSubsystem;

    public DuckySpinnerBlueCommand(DuckySpinnerSubsystem duckySpinnerSubsystem) {
        dSSubsystem = duckySpinnerSubsystem;
        addRequirements(dSSubsystem);
    }

    @Override
    public void execute() {
        dSSubsystem.start();
    }

    @Override
    public void end(boolean interrupted) {
        dSSubsystem.stop();
    }
}
