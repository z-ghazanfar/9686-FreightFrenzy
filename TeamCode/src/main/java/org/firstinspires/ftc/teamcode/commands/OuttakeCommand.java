// package org.firstinspires.ftc.teamcode.commands;
//
// import com.arcrobotics.ftclib.command.CommandBase;
//
// import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
//
// public class OuttakeCommand extends CommandBase {
//
//     private IntakeSubsystem iS;
//
//     public OuttakeCommand(IntakeSubsystem intake){
//         iS = intake;
//         addRequirements(iS);
//     }
//
//     @Override
//     public void execute() {
//         iS.startOuttake();
//     }
//
//     public void end(boolean interrupted) {
//         iS.stopIntake();
//     }
//
// }
