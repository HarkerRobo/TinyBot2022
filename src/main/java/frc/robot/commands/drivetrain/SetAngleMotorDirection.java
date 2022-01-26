package frc.robot.commands.drivetrain;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Drivetrain;
import frc.robot.util.Vector;

public class SetAngleMotorDirection extends WaitCommand {
    private Pose2d pose;
    private static final double WAIT_TIME = 1;

    public SetAngleMotorDirection(Trajectory t) {
        super(WAIT_TIME);
        addRequirements(Drivetrain.getInstance());
        pose = t.getStates().get(0).poseMeters;
    }

    @Override
    public void initialize() {
        super.initialize();
    }

    @Override
    public void execute() {
        ChassisSpeeds chassis = ChassisSpeeds.fromFieldRelativeSpeeds(pose.getX()*0.00001, pose.getY()*0.00001, 0, Rotation2d.fromDegrees(-Drivetrain.getInstance().getPigeon().getFusedHeading()));
        Drivetrain.getInstance().setAngleAndDriveVelocity(Drivetrain.getInstance().getKinematics().toSwerveModuleStates(chassis));
    }
}
