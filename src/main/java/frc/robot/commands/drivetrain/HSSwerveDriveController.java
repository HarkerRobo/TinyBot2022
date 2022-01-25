package frc.robot.commands.drivetrain;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import frc.robot.subsystems.Drivetrain;
import frc.robot.util.Vector;

public class HSSwerveDriveController extends SwerveControllerCommand {
    private static PIDController xController = new PIDController(Drivetrain.MP_X_KP, Drivetrain.MP_X_KI, Drivetrain.MP_X_KD);
    private static PIDController yController = new PIDController(Drivetrain.MP_Y_KP, Drivetrain.MP_Y_KI, Drivetrain.MP_Y_KD);
    private static ProfiledPIDController thetaController = new ProfiledPIDController(Drivetrain.MP_THETA_KP, Drivetrain.MP_THETA_KI, Drivetrain.MP_X_KD, new Constraints(2 * Math.PI, 3 * Math.PI));
    
    private Trajectory trajectory;
    private Rotation2d initHeading;

  
    public HSSwerveDriveController(Trajectory trajectory, Rotation2d heading, Rotation2d initHeading) {
        super(trajectory, Drivetrain.getInstance().getOdometry()::getPoseMeters, 
        Drivetrain.getInstance().getKinematics(), 
        xController, 
        yController, 
        thetaController,
        () -> heading,//.minus(Rotation2d.fromDegrees(90)),
        Drivetrain.getInstance()::setAngleAndDriveVelocity,
        Drivetrain.getInstance());
        this.trajectory=trajectory;
        this.initHeading = initHeading;
    }
    
    @Override
    public void initialize() {
        super.initialize();
        Drivetrain.getInstance().getPigeon().addFusedHeading(-63.9886 * (Drivetrain.getInstance().getPigeon().getFusedHeading()+initHeading.getDegrees()));
        Drivetrain.getInstance().getOdometry().resetPosition(new Pose2d(trajectory.getInitialPose().getTranslation(),Rotation2d.fromDegrees(Drivetrain.getInstance().getPigeon().getFusedHeading())),//-90)),
            Rotation2d.fromDegrees(Drivetrain.getInstance().getPigeon().getFusedHeading()));
    }

    @Override
    public void execute() {
        super.execute();
        SmartDashboard.putNumber("mp x error", xController.getPositionError());
        SmartDashboard.putNumber("mp y error", yController.getPositionError());
        SmartDashboard.putNumber("mp theta error", thetaController.getPositionError());
    }

    @Override
    public void end(boolean dhsak){
        super.end(dhsak);
        Drivetrain.getInstance().setPercentOutput(new Vector(0, 0));
    }
}