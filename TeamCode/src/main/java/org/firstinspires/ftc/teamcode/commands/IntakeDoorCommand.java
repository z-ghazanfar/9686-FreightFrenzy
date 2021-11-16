package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

public class IntakeDoorCommand extends CommandBase {

    private IntakeSubsystem iS;

    public IntakeDoorCommand(IntakeSubsystem intake) {
        iS = intake;
        addRequirements(iS);
    }

    @Override
    public void execute() {
        iS.openDoor();
    }

    public void end(boolean interrupted) {
        iS.closeDoor();
    }

}
