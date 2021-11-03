package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.DuckySpinnerSubsystem;

public class DuckySpinnerRedCommand extends CommandBase {

    private DuckySpinnerSubsystem dSSubsystem;

    public DuckySpinnerRedCommand(DuckySpinnerSubsystem duckySpinnerSubsystem) {
        dSSubsystem = duckySpinnerSubsystem;
        addRequirements(dSSubsystem);
    }

    @Override
    public void execute() {
        dSSubsystem.startReverse();
    }

    @Override
    public void end(boolean interrupted) {
        dSSubsystem.stop();
    }
}
