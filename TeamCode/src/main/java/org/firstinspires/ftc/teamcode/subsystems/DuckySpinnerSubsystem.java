package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;

public class DuckySpinnerSubsystem extends SubsystemBase {

    private Motor dS;

    public DuckySpinnerSubsystem(Motor duckySpinner) {
        dS = duckySpinner;
    }

    public void start() { dS.set(0.5); }
    public void startReverse() { dS.set(-0.5); }
    public void stop() { dS.stopMotor(); }

}
