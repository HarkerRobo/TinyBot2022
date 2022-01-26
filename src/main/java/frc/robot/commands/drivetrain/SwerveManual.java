package frc.robot.commands.drivetrain;

import frc.robot.subsystems.Drivetrain;
import harkerrobolib.commands.IndefiniteCommand;
import harkerrobolib.util.MathUtil;
import frc.robot.util.Limelight;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.OI;

public class SwerveManual extends IndefiniteCommand {
    private static final double OUTPUT_MULTIPLIER= 1;
    private static final double kP=0.03;
    private static final double kI=0.0;//00002;
    private static final double kD=0.0;//02;
    private static final double I_ZONE = 0;
    private static final double angleKP=0.2;
    public static double pigeonAngle = Drivetrain.getInstance().getPigeon().getFusedHeading();
    private Debouncer debouncer = new Debouncer(0.3, DebounceType.kRising);

    private double lastPigeonHeading = 0;


    private PIDController pid;
    public SwerveManual() {
        addRequirements(Drivetrain.getInstance());
        pid = new PIDController(kP, kI, kD);
        pid.setIntegratorRange(-I_ZONE, I_ZONE);
    }

    @Override
    public void execute() {
        double angularVelocity = MathUtil.mapJoystickOutput(OI.getInstance().getDriverGamepad().getRightX(), OI.DEADBAND);
        double translationx = MathUtil.mapJoystickOutput(OI.getInstance().getDriverGamepad().getLeftX(), OI.DEADBAND);
        double translationy = MathUtil.mapJoystickOutput(OI.getInstance().getDriverGamepad().getLeftY(), OI.DEADBAND);
        double chasisMagnitude = Math.sqrt(Math.pow(translationx,2) + Math.pow(translationy,2));

        if(chasisMagnitude<(Drivetrain.MIN_OUTPUT)){
            translationx = 0;
            translationy = 0;
            if(Math.abs(angularVelocity)<(Drivetrain.MIN_OUTPUT)){
                // pigeonAngle = Drivetrain.getInstance().getPigeon().getFusedHeading() + 0.1 ;
                angularVelocity = 0.000001;
            }
        }
        // if(OI.getInstance().getDriverGamepad().getButtonBState() || OI.getInstance().getOperatorGamepad().getButtonBState()){
        //     Shooter.getInstance().setAutoHoodAngle();
        // }

        angularVelocity *= Drivetrain.MAX_ANGULAR_VEL;
        SmartDashboard.putNumber("limelight tx", Limelight.getTx());
        SmartDashboard.putNumber("limelight ang vel", angularVelocity);
        translationx *= Drivetrain.MAX_DRIVE_VEL * OUTPUT_MULTIPLIER;
        translationy *= Drivetrain.MAX_DRIVE_VEL * OUTPUT_MULTIPLIER;

       // System.out.println(translationx + " " + translationy);
        // double rotation =
        // MathUtil.mapJoystickOutput(OI.getInstance().getDriverGamepad().getRightX(),
        // OI.DEADBAND);
        angularVelocity*=0.6;
        if(debouncer.calculate(Math.abs(MathUtil.mapJoystickOutput(OI.getInstance().getDriverGamepad().getRightX(), OI.DEADBAND))<0.05)){
            angularVelocity=angleKP*(pigeonAngle - Drivetrain.getInstance().getPigeon().getFusedHeading());
            SmartDashboard.putBoolean("holding pigeon angle", true);
        }
        else {
            pigeonAngle = Drivetrain.getInstance().getPigeon().getFusedHeading();
            SmartDashboard.putBoolean("holding pigeon angle", false);
        }
            

        if(OI.getInstance().getDriverGamepad().getButtonBState()){
            translationx*=0.5;
            translationy*=0.5;
            angularVelocity*=0.5;
        }



        ChassisSpeeds chassis;
        if(Drivetrain.getInstance().isFieldCentric())
            chassis = ChassisSpeeds.fromFieldRelativeSpeeds(translationx, translationy, -angularVelocity, Rotation2d.fromDegrees(-Drivetrain.getInstance().getPigeon().getFusedHeading()));
        else
            chassis = ChassisSpeeds.fromFieldRelativeSpeeds(translationx, translationy, -angularVelocity, Rotation2d.fromDegrees(0));
        Drivetrain.getInstance().setAngleAndDriveVelocity(Drivetrain.getInstance().getKinematics().toSwerveModuleStates(chassis));
        lastPigeonHeading = Drivetrain.getInstance().getPigeon().getFusedHeading(); 
    }
}
