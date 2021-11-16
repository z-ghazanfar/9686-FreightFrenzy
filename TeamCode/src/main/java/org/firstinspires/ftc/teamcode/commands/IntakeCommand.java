package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

public class IntakeCommand extends CommandBase {

    private IntakeSubsystem iS;

    public IntakeCommand(IntakeSubsystem intake){
        iS = intake;
        addRequirements(iS);
    }

    @Override
    public void execute() {
        iS.startIntake();
    }

    public void end(boolean interrupted) {
        iS.stopIntake();
    }

}
