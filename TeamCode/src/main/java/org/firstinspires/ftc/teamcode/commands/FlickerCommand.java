package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

public class FlickerCommand extends CommandBase {

    IntakeSubsystem intakeSubsystem;

    public FlickerCommand(IntakeSubsystem iS) {
        intakeSubsystem = iS;
        addRequirements(intakeSubsystem);
    }

    @Override
    public void initialize() {
        intakeSubsystem.flickFlicker();
        intakeSubsystem.initFlicker();
    }
}
