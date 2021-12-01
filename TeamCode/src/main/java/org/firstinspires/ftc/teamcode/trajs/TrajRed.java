package org.firstinspires.ftc.teamcode.trajs;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.trajectory.Trajectory;
import com.arcrobotics.ftclib.trajectory.TrajectoryConfig;
import com.arcrobotics.ftclib.trajectory.TrajectoryGenerator;
import com.sun.tools.javac.util.List;

import java.util.ArrayList;

public class TrajRed {
    public static Trajectory generateTrajectory() {

        Pose2d sideStart = new Pose2d(0.0, 0.0,
                Rotation2d.fromDegrees(0));
        Pose2d crossScale = new Pose2d(-0.2, 1,
                Rotation2d.fromDegrees(0));

        // Forward is positive x, Left is positive y
        ArrayList interiorWaypoints = new ArrayList<Translation2d>();
        interiorWaypoints.add(new Translation2d(-0.2, 0));

        TrajectoryConfig config = new TrajectoryConfig(0.5, 0.5);

        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
                sideStart,
                interiorWaypoints,
                crossScale,
                config);

        Trajectory trajectory1 = TrajectoryGenerator.generateTrajectory(
                new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
                List.of(
                        new Translation2d(0.5, 0)
                ),
                new Pose2d(0.5, 1, Rotation2d.fromDegrees(90)),
                config);

        return trajectory1;
    }
}