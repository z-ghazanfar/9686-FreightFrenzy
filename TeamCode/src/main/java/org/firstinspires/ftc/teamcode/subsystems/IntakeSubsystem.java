package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.util.ElapsedTime;

public class IntakeSubsystem extends SubsystemBase {
    private SimpleServo door, flicker;

    public IntakeSubsystem(SimpleServo doorServo, SimpleServo flickerServo){
        door = doorServo;
        flicker = flickerServo;
    }

    public void openDoor(){ door.setPosition(1);}

    public void closeDoor(){
        door.setPosition(0);
    }

    public void initFlicker(){
        flicker.setPosition(0);
    }

    public void flickFlicker() {
        flicker.setPosition(1);
    }

}
