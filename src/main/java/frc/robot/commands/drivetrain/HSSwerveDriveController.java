package frc.robot.commands.drivetrain;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import frc.robot.auto.Trajectories;
import frc.robot.subsystems.Drivetrain;
import frc.robot.util.Vector;

public class HSSwerveDriveController extends CommandBase {
    private static PIDController xController = new PIDController(Drivetrain.MP_X_KP, Drivetrain.MP_X_KI, Drivetrain.MP_X_KD);
    private static PIDController yController = new PIDController(Drivetrain.MP_Y_KP, Drivetrain.MP_Y_KI, Drivetrain.MP_Y_KD);
    private static ProfiledPIDController thetaController = new ProfiledPIDController(Drivetrain.MP_THETA_KP, Drivetrain.MP_THETA_KI, Drivetrain.MP_X_KD, new Constraints(2 * Math.PI, 3 * Math.PI));
    
    private Rotation2d initHeading;
    private SequentialCommandGroup group;
    List<Trajectory> trajectories;

  
    public HSSwerveDriveController(List<Trajectory> trajectories, Rotation2d initHeading) {
        this.trajectories = trajectories;
        group = new SequentialCommandGroup();
        for(Trajectory t: trajectories) {
            group.addCommands(new SwerveControllerCommand(t, Drivetrain.getInstance().getOdometry()::getPoseMeters, 
                Drivetrain.getInstance().getKinematics(), xController, yController, thetaController,() -> initHeading, 
                Drivetrain.getInstance()::setAngleAndDriveVelocity, Drivetrain.getInstance()));
            group.addCommands(new SetAngleMotorDirection(t));
        }
        this.initHeading = initHeading;
    }
    
    @Override
    public void initialize() {
        group.initialize();
        Drivetrain.getInstance().getPigeon().addFusedHeading(-63.9886 * (Drivetrain.getInstance().getPigeon().getFusedHeading()+initHeading.getDegrees()));
        Drivetrain.getInstance().getOdometry().resetPosition(new Pose2d(trajectories.get(0).getInitialPose().getTranslation(),Rotation2d.fromDegrees(Drivetrain.getInstance().getPigeon().getFusedHeading())),
            Rotation2d.fromDegrees(Drivetrain.getInstance().getPigeon().getFusedHeading()));
    }

    @Override
    public void execute() {
        group.execute();
        SmartDashboard.putNumber("mp x error", xController.getPositionError());
        SmartDashboard.putNumber("mp y error", yController.getPositionError());
        SmartDashboard.putNumber("mp theta error", thetaController.getPositionError());
    }

    @Override
    public void end(boolean interrupted){
        group.end(interrupted);
        SwerveManual.pigeonAngle = Drivetrain.getInstance().getPigeon().getFusedHeading();
        Drivetrain.getInstance().setPercentOutput(new Vector(0, 0));
    }
}