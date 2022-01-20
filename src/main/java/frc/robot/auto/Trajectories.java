package frc.robot.auto;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import frc.robot.subsystems.Drivetrain;

/**
 * Contains all Trajectories used for various autonomous configurations.
 * 
 * @author Jatin Kohli
 * @author Shahzeb Lakhani
 * @author Ada Praun-Petrovic
 * @author Chirag Kaushik
 * @author Angela Jia
 * @author Anirudh Kotamraju
 * @since February 12, 2020
 */
public class Trajectories {
    public static TrajectoryConfig config = new TrajectoryConfig(Drivetrain.MP_MAX_DRIVE_VELOCITY,
            Drivetrain.MP_MAX_DRIVE_ACCELERATION).setKinematics(Drivetrain.getInstance().getKinematics());
    
    public static final Trajectory moveForward = TrajectoryGenerator.generateTrajectory(List.of(
        new Pose2d(0,0,Rotation2d.fromDegrees(90)),
        new Pose2d(0,2,Rotation2d.fromDegrees(90))
    ), config);

    public static final Trajectory moveBackward = TrajectoryGenerator.generateTrajectory(List.of(
        new Pose2d(0,0,Rotation2d.fromDegrees(270)),
        new Pose2d(0,-2,Rotation2d.fromDegrees(270))
    ), config);

        public static final Trajectory moveRight = TrajectoryGenerator.generateTrajectory(List.of(
        new Pose2d(0,0,Rotation2d.fromDegrees(0)),
        new Pose2d(2,0,Rotation2d.fromDegrees(0))
    ), config);

        public static final Trajectory moveLeft = TrajectoryGenerator.generateTrajectory(List.of(
        new Pose2d(0,0,Rotation2d.fromDegrees(180)),
        new Pose2d(-2,0,Rotation2d.fromDegrees(180))
    ), config);
}