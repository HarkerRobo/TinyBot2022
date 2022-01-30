package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.auto.Trajectories;
import frc.robot.commands.drivetrain.HSSwerveDriveController;
import frc.robot.commands.drivetrain.SwerveManual;
// import frc.robot.commands.drivetrain.HSSwerveDriveController;
// import frc.robot.commands.intake.IntakeAutonControlForward;
// import frc.robot.commands.intake.MoveBallsToShooter;
// import frc.robot.commands.shooter.ShooterVelocityManual;
// import frc.robot.commands.spine.Jumble;
import frc.robot.subsystems.Drivetrain;
// import frc.robot.subsystems.Intake;
// import frc.robot.subsystems.Shooter;
import harkerrobolib.wrappers.HSGamepad;
import harkerrobolib.wrappers.XboxGamepad;


public class OI {
    private static OI oi;
    private HSGamepad driverGamepad;
    private HSGamepad operatorGamepad;
    public static final double DEADBAND = 0.1;
    public static final int DRIVER_PORT = 0;
    public static final int OPERATOR_PORT = 1;
    
    private OI() {
        driverGamepad = new XboxGamepad(DRIVER_PORT);
        operatorGamepad = new XboxGamepad(OPERATOR_PORT);

        initBindings();
    }

    private void initBindings() {

        driverGamepad.getButtonX().whenPressed(new InstantCommand(() -> {Drivetrain.getInstance().getPigeon().addFusedHeading(-63.9886 * Drivetrain.getInstance().getPigeon().getFusedHeading()); SwerveManual.pigeonAngle=0;}));

        driverGamepad.getButtonA().whenPressed(new InstantCommand(() -> {
            Drivetrain.getInstance().toggleDriveMode();
        }));

        driverGamepad.getButtonY().whenPressed(new InstantCommand(() -> {Robot.timer.reset();Robot.timer.start();}));

        driverGamepad.getButtonY().whenPressed(new SequentialCommandGroup(new HSSwerveDriveController(Trajectories.fiveBallAuto.get(0),Rotation2d.fromDegrees(0),true),
            new HSSwerveDriveController(Trajectories.fiveBallAuto.get(1),Rotation2d.fromDegrees(0)),new HSSwerveDriveController(Trajectories.fiveBallAuto.get(2),Rotation2d.fromDegrees(0)),
            new HSSwerveDriveController(Trajectories.fiveBallAuto.get(3),Rotation2d.fromDegrees(0))));

        // driverGamepad.getButtonStart().whenPressed(new InstantCommand(() -> {
        //     Drivetrain.getInstance().getPigeon().addFusedHeading(-63.9886 * 180);
        // }));

        // driverGamepad.getUpDPadButton().whenPressed(new HSSwerveDriveController(Trajectories.moveForward, Rotation2d.fromDegrees(0)));
        // driverGamepad.getDownDPadButton().whenPressed(new HSSwerveDriveController(Trajectories.moveBackward, Rotation2d.fromDegrees(0)));
        // driverGamepad.getLeftDPadButton().whenPressed(new HSSwerveDriveController(Trajectories.moveLeft, Rotation2d.fromDegrees(0)));
        // driverGamepad.getRightDPadButton().whenPressed(new HSSwerveDriveController(Trajectories.moveRight, Rotation2d.fromDegrees(0)));

        // driverGamepad.getButtonA().whenPressed(new HSSwerveDriveController(Trajectories.clockwisecircle, Rotation2d.fromDegrees(0)));
        // driverGamepad.getButtonB().whenPressed(new HSSwerveDriveController(Trajectories.counterclockwisecircle, Rotation2d.fromDegrees(0)));
    }
        // operatorGamepad.getButtonA().whenPressed(new InstantCommand(() -> {
        //     Intake.getInstance().invertSolenoid();
        // }, Intake.getInstance()));
        
        // driverGamepad.getButtonBumperRight().whilePressed(new ParallelCommandGroup(
        //     new MoveBallsToShooter(), new ShooterManual()
        // ));

        // driverGamepad.getRightDPadButton().whenPressed(new ParallelCommandGroup(
        //     Autons.autoPath1
        // ));
        // operatorGamepad.getButtonA().whenPressed(new InstantCommand(() -> Intake.getInstance().invertSolenoid(), Intake.getInstance()));
        // operatorGamepad.getButtonStart().whilePressed(new ShooterVelocityManual(78.5));


        
    
    public HSGamepad getDriverGamepad(){
        return driverGamepad;
    }
    public HSGamepad getOperatorGamepad(){
        return operatorGamepad;
    }

    public static OI getInstance(){
        if(oi==null)
            oi=new OI();
        return oi;
    }
}
