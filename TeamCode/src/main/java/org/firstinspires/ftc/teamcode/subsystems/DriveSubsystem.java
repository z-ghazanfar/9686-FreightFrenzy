package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.drivebase.DifferentialDrive;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.DifferentialDriveOdometry;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.DifferentialDriveWheelSpeeds;

import org.firstinspires.ftc.teamcode.DriveConstants;
import org.firstinspires.ftc.teamcode.MotorGroupTemp;

public class DriveSubsystem extends SubsystemBase {

    private MotorGroupTemp leftDrive, rightDrive; // Makes it so that left motors move in tandem and right motors move in tandem.
    private DifferentialDrive arcadeDrive;
    private DifferentialDriveOdometry differentialOdom;
    private RevIMU imu;

    // This constructor sets up declared variables with defaults.
    public DriveSubsystem(MotorGroupTemp left, MotorGroupTemp right, RevIMU revIMU) {
        leftDrive = left;
        rightDrive = right;

        // Sets distance per pulse to tuned value
        leftDrive.setDistancePerPulse(DriveConstants.DISTANCE_PER_PULSE);
        rightDrive.setDistancePerPulse(DriveConstants.DISTANCE_PER_PULSE);

        imu = revIMU;

        resetEncoders();
        arcadeDrive = new DifferentialDrive(leftDrive, rightDrive);
        differentialOdom = new DifferentialDriveOdometry(imu.getRotation2d());
    }

    // Subsystems have methods which can be run for various functionality, mainly used for convenience.

    // Self-explanatory
    public void resetEncoders() {
        leftDrive.resetEncoder();
        rightDrive.resetEncoder();
    }

    // Gets the Pose (x, y, direction)
    public Pose2d getPose() {
        return differentialOdom.getPoseMeters();
    }

    // Gets wheel speeds
    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        return new DifferentialDriveWheelSpeeds(leftDrive.getVelocity()*DriveConstants.DISTANCE_PER_PULSE,
            rightDrive.getVelocity()*DriveConstants.DISTANCE_PER_PULSE);
    }

    // Sets speed of drivetrain (autonomous)
    public void driveAuton(double leftSpeed, double rightSpeed) {
        leftDrive.set(leftSpeed);
        rightDrive.set(rightSpeed);
    }

    // Sets speed of arcade drive in teleop
    public void drive(double forwardSpeed, double turnSpeed) {
        arcadeDrive.arcadeDrive(-forwardSpeed, -turnSpeed);
    }

    public void slowDrive(double forwardSpeed, double turnSpeed) {
        arcadeDrive.arcadeDrive(-forwardSpeed, -turnSpeed, true);
    }

    // Repeatedly called whenever DriveSubsystem is in use (which is the entire time the bot is running)
    @Override
    public void periodic() {
        differentialOdom.update(imu.getRotation2d(), leftDrive.getPositions().get(0), rightDrive.getPositions().get(0));
    }

}
