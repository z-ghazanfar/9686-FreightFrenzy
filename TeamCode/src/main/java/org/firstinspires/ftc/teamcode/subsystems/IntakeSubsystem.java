package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;

public class IntakeSubsystem extends SubsystemBase {
    private Motor intake;
    private SimpleServo door;

    public IntakeSubsystem(Motor iM, SimpleServo doorServo){
        intake = iM;
        door = doorServo;
    }

    public void startIntake(){
        intake.set(0.75);
    }

    public void startReverseIntake(){
        intake.set(-0.75);
    }

    public void stopIntake(){
        intake.set(0);
    }

    public void openDoor(){
        door.setPosition(0.5);
    }

    public void closeDoor(){
        door.setPosition(0);
    }
}
