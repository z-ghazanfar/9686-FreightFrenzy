package org.firstinspires.ftc.teamcode.trajs;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.trajectory.Trajectory;
import com.arcrobotics.ftclib.trajectory.TrajectoryConfig;
import com.arcrobotics.ftclib.trajectory.TrajectoryGenerator;

import java.util.ArrayList;

public class Trajectories {

    public static Trajectory trajRed() {
        Pose2d start = new Pose2d(0.3, 0.0, new Rotation2d(0.0));
        Pose2d end = new Pose2d(0.0, -1.0, new Rotation2d(0.0));

        ArrayList interiorWaypoints = new ArrayList<Translation2d>();

        interiorWaypoints.add(new Translation2d(0.2, -1.0));
        interiorWaypoints.add(new Translation2d(0.1, -1.6));

        TrajectoryConfig config = new TrajectoryConfig(1.5, 1.5);

        return TrajectoryGenerator.generateTrajectory(start, interiorWaypoints, end, config);
    }

    public static Trajectory trajBlue() {
        Pose2d start = new Pose2d(0.3, 0.0, new Rotation2d(0.0));
        Pose2d end = new Pose2d(0.1, 1.0, new Rotation2d(0.0));

        ArrayList interiorWaypoints = new ArrayList<Translation2d>();

        interiorWaypoints.add(new Translation2d(0.3, 1.0));
        interiorWaypoints.add(new Translation2d(0.2 , 1.5));

        TrajectoryConfig config = new TrajectoryConfig(1.5, 1.5);

        return TrajectoryGenerator.generateTrajectory(start, interiorWaypoints, end, config);
    }

}
