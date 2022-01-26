package frc.robot.util;

import frc.robot.RobotMap;
import frc.robot.subsystems.Drivetrain;
import harkerrobolib.wrappers.HSTalon;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.sensors.SensorVelocityMeasPeriod;

import harkerrobolib.util.Conversions.SpeedUnit;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import harkerrobolib.util.Conversions;
import harkerrobolib.wrappers.HSFalcon;

public class SwerveModule {
	public HSTalon rotation;
	public HSFalcon translation;

	private static final double VOLTAGE_COMP = 10;

	private static final double DRIVE_CURRENT_CONTINUOUS = 40;
    private static final double DRIVE_CURRENT_PEAK = 60;
    private static final double DRIVE_CURRENT_PEAK_DUR = 0.4;

    private static final double ANGLE_CURRENT_CONTINUOUS = 15;
    private static final double ANGLE_CURRENT_PEAK = 20;
    private static final double ANGLE_CURRENT_PEAK_DUR = 0.02;

	// private static final double TRANSLATION_P = 0.32126;
	// private static final double TRANSLATION_I = 0.0;
	// private static final double TRANSLATION_D = 0;
	// private static final double TRANSLATION_F = 0.04698;

	private static final double ANGLE_P = 0.8;
	private static final double ANGLE_I = 0;
	private static final double ANGLE_D = 15;
	private static final double ENCODER_TICKS = 4096.0;
	private static final double EPSILON_OUTPUT = 1e-4;
	private static final int DRIVE_TICKS_PER_REV = 2048;

	private static final double DRIVE_KS = 0.578;
	private static final double DRIVE_KV = 2.0473;
	private static final double DRIVE_KA = 0.13018;

	private static final double MAX_CONTROL_EFFORT = 10; // volts 
    private static final double MODEL_STANDARD_DEVIATION = 2;
    private static final double ENCODER_STANDARD_DEVIATION = 0.08;

	private SimpleVelocitySystem loop;

	private boolean ROTATION_SENSOR_PHASE;
	private boolean TRANSLATION_SENSOR_PHASE;

	
	private boolean ROTATION_INVERT;
	private boolean TRANSLATION_INVERT;


	public SwerveModule(boolean rotationSensorPhase, boolean translationSensorPhase, int rotationDriveId,
			int translationDriveId, boolean rotationInverted, boolean translationInverted) {
		ROTATION_SENSOR_PHASE = rotationSensorPhase;
		TRANSLATION_SENSOR_PHASE = translationSensorPhase;

		ROTATION_INVERT=rotationInverted;
		TRANSLATION_INVERT=translationInverted;

		rotation = new HSTalon(rotationDriveId);
		translation = new HSFalcon(translationDriveId);
		rotationMotorInit();
		translationMotorInit();
		loop = new SimpleVelocitySystem(DRIVE_KS, DRIVE_KV, DRIVE_KA,
			MAX_CONTROL_EFFORT, MODEL_STANDARD_DEVIATION, ENCODER_STANDARD_DEVIATION, RobotMap.LOOP_TIME);
	}

	public HSFalcon getTranslationMotor() {
		return translation;
	}

	public HSTalon getRotationMotor() {
		return rotation;
	}

	public double getRotationAngle(){
		return rotation.getSelectedSensorPosition()*(360.0/ENCODER_TICKS);
	}

	private void rotationMotorInit() {
		rotation.configFactoryDefault();
		rotation.setInverted(ROTATION_INVERT);
		rotation.setSensorPhase(ROTATION_SENSOR_PHASE);
		rotation.setNeutralMode(NeutralMode.Brake);
		rotation.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, ANGLE_CURRENT_CONTINUOUS, ANGLE_CURRENT_PEAK, ANGLE_CURRENT_PEAK_DUR));
		rotation.configVoltageCompSaturation(VOLTAGE_COMP);
		rotation.configForwardSoftLimitEnable(false);

		rotation.config_kP(RobotMap.SLOT_INDEX, ANGLE_P);
		rotation.config_kI(RobotMap.SLOT_INDEX, ANGLE_I);
		rotation.config_kD(RobotMap.SLOT_INDEX, ANGLE_D);

		rotation.selectProfileSlot(RobotMap.SLOT_INDEX, RobotMap.LOOP_INDEX);

		
		rotation.configVelocityMeasurementWindow(16);
		rotation.configVelocityMeasurementPeriod(SensorVelocityMeasPeriod.Period_25Ms);

		rotation.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, RobotMap.SLOT_INDEX);
	}

	private void translationMotorInit() {
		translation.configFactoryDefault();
		translation.setSensorPhase(TRANSLATION_SENSOR_PHASE);
		translation.setNeutralMode(NeutralMode.Brake);
		translation.setInverted(TRANSLATION_INVERT);

		translation.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, DRIVE_CURRENT_CONTINUOUS, DRIVE_CURRENT_PEAK, DRIVE_CURRENT_PEAK_DUR));

		// translation.config_kP(RobotMap.SLOT_INDEX, TRANSLATION_P);
		// translation.config_kI(RobotMap.SLOT_INDEX, TRANSLATION_I);
		// translation.config_kD(RobotMap.SLOT_INDEX, TRANSLATION_D);
		// translation.config_kF(RobotMap.SLOT_INDEX, TRANSLATION_F);

		translation.configVelocityMeasurementWindow(1);
		translation.configVelocityMeasurementPeriod(SensorVelocityMeasPeriod.Period_10Ms);

		translation.selectProfileSlot(RobotMap.SLOT_INDEX, RobotMap.LOOP_INDEX);

		translation.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, RobotMap.SLOT_INDEX);

	}

	public void setPercentOutput(Vector translationvec) {
		translation.set(ControlMode.PercentOutput, translationvec.getMagnitude());
		rotation.set(ControlMode.Position, translationvec.getAngle() * (4096 / 360));

	}

	public SwerveModuleState getState() {
		return new SwerveModuleState(
			getCurrentVelocity(), 
			Rotation2d.fromDegrees(rotation.getSelectedSensorPosition() * 360 / 4096)
		);
	}

	public void setSwerveManual(SwerveModuleState state){
		//state = optimize(state, getRotationAngle());
		double speed=state.speedMetersPerSecond;
		double angle = state.angle.getDegrees();
		double currentAngle=getRotationAngle();
		while(angle-currentAngle>180){
			angle -= 360;
			//delta = desiredState.angle.getDegrees() - currentAngle;
		}

		while(angle-currentAngle<-180){

			angle += 360;
			//delta = desiredState.angle.minus(currentAngle);

		}

		if(angle-currentAngle>90){
			angle -= 180;
			speed *= -1;
		}

		else if(angle-currentAngle<-90){
			angle += 180;
			speed *= -1;
		}

		setDriveOutput(speed, false);
		rotation.set(ControlMode.Position,angle * (4096 / 360));


	}

	/**
     * Sets the module's output to have the desired angle and output. 90 degree optimizations are performed if needed.
     * 
     * @param targetAngle The angle in degrees for the module's angle motor to have. Zero degrees is in the positive x direction.
     * @param output The output for the module's drive motor to have, with units corresponding to isPercentOutput.
     * @param isPercentOutput True if the output's units are a percentage of maximum output, false if the units are meters per second.
     * @param isMotionProfile True if the robot is currently in a motion profile and should not perform 90 degree optimizations, false if otherwise.
     */
    public void setAngleAndDriveVelocity(double targetAngle, double output, boolean isPercentOutput, boolean isMotionProfile) {
        boolean shouldReverse = !isMotionProfile && Math.abs(targetAngle - getRotationAngle()) > 90;
        
        if (shouldReverse) {
            setDriveOutput(-output, isPercentOutput);
            if (targetAngle - getAngleDegrees() > 90) {
                targetAngle -= 180;
            }
            else {
                targetAngle += 180;
            }
        } else {
            setDriveOutput(output, isPercentOutput);
        }
        
        int targetPos = (int)((targetAngle / 360) * 4096);

        if(output > EPSILON_OUTPUT || isMotionProfile) 
            rotation.set(ControlMode.Position, targetPos);
	}
	
	/**
     * Sets the drive output of the swerve module in either percent output or velocity in feet per second.
     * 
     * @param output The output of the swerve module, with units corresponding to isPercentOutput.
     * @param isPercentOutput True if the output's units are a percentage of maximum output, false if the units are meters per second.
     */
    public void setDriveOutput(double output, boolean isPercentOutput) {
        if(isPercentOutput) {
            translation.set(TalonFXControlMode.PercentOutput, output);
        } else {
			loop.set(output);
			loop.update(getCurrentVelocity());
            translation.set(TalonFXControlMode.PercentOutput, loop.getOutput());
        }
	}
	
	/**
     * Returns the current angle in degrees
     */
    public double getAngleDegrees() {
        return rotation.getSelectedSensorPosition() * 360.0 / SwerveModule.ENCODER_TICKS; //Convert encoder ticks to degrees
    }

	public double getCurrentVelocity(){
		return Conversions.convertSpeed(SpeedUnit.ENCODER_UNITS, translation.getSelectedSensorVelocity() / Drivetrain.FEET_TO_METER, SpeedUnit.FEET_PER_SECOND, Drivetrain.WHEEL_DIAMETER, DRIVE_TICKS_PER_REV) / Drivetrain.GEAR_RATIO;
	}

	public SimpleVelocitySystem getLoop() {
		return loop;
	}
}