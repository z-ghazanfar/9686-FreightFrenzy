package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.arcrobotics.ftclib.controller.wpilibcontroller.RamseteController;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.DifferentialDriveKinematics;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor.RunMode;

import org.firstinspires.ftc.teamcode.commands.RamseteCommandRe;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.trajs.*;

@Autonomous(name="NoPIDRed")
public class AutonNoPIDRed extends CommandOpMode {

    private Motor frontLeft, frontRight, backLeft, backRight;
    private MotorGroupTemp leftDrive, rightDrive;

    private RevIMU imu;

    private DifferentialDriveKinematics ddKinematics;
    private DriveSubsystem driveSubsystem;
    private RamseteCommandRe ramseteCommand;

    @Override
    public void initialize() {

        frontLeft = new Motor(hardwareMap, "frontLeft", Motor.GoBILDA.RPM_435);
        backLeft = new Motor(hardwareMap, "backLeft", Motor.GoBILDA.RPM_435);
        frontRight = new Motor(hardwareMap, "frontRight", Motor.GoBILDA.RPM_435);
        backRight = new Motor(hardwareMap, "backRight", Motor.GoBILDA.RPM_435);

        leftDrive = new MotorGroupTemp(frontLeft, backLeft);
        rightDrive = new MotorGroupTemp(frontRight, backRight);
        rightDrive.setInverted(true);

        frontLeft.motor.setMode(RunMode.STOP_AND_RESET_ENCODER);
        backLeft.motor.setMode(RunMode.STOP_AND_RESET_ENCODER);
        frontRight.motor.setMode(RunMode.STOP_AND_RESET_ENCODER);
        backRight.motor.setMode(RunMode.STOP_AND_RESET_ENCODER);

        frontLeft.motor.setMode(RunMode.RUN_WITHOUT_ENCODER);
        backLeft.motor.setMode(RunMode.RUN_WITHOUT_ENCODER);
        frontRight.motor.setMode(RunMode.RUN_WITHOUT_ENCODER);
        backRight.motor.setMode(RunMode.RUN_WITHOUT_ENCODER);

        imu = new RevIMU(hardwareMap);
        imu.init();
        ddKinematics = new DifferentialDriveKinematics(DriveConstants.TRACK_WIDTH);

        driveSubsystem = new DriveSubsystem(leftDrive, rightDrive, imu);
        driveSubsystem.getWheelSpeeds().normalize(1.5);
        ramseteCommand = new RamseteCommandRe(Trajectories.trajRed(), driveSubsystem::getPose,
                new RamseteController(DriveConstants.B, DriveConstants.ZETA),
                ddKinematics,
                driveSubsystem::driveAuton,
                telemetry);

        schedule(new WaitUntilCommand(this::isStarted).andThen(new ParallelCommandGroup(
                ramseteCommand,
                new RunCommand(() -> {
//                    telemetry.addData("CurPos", driveSubsystem.getPose());
//                    telemetry.addData("Left Encoders", leftDrive.getPositions());
//                    telemetry.addData("Right Encoders", rightDrive.getPositions());
                    telemetry.update();
                })
        )));
    }
}
