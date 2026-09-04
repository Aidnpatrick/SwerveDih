package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public class SwerveModule {
  
  private final Constants.ModuleConfig config;
  
  private final TalonFX driveMotor;
  
  private final TalonFX turningMotor;
  
  private final CANcoder turningEncoder;
  
  private final ProfiledPIDController turningPid =
      new ProfiledPIDController(
          Constants.ModuleControlConstants.kTurningKpVoltsPerRadian,
          Constants.ModuleControlConstants.kTurningKiVoltsPerRadianSecond,
          Constants.ModuleControlConstants.kTurningKdVoltSecondsPerRadian,
          new TrapezoidProfile.Constraints(
              Constants.ModuleControlConstants.kTurningMaxVelocityRadiansPerSecond,
              Constants.ModuleControlConstants.kTurningMaxAccelerationRadiansPerSecondSquared));
  
  private final SimpleMotorFeedforward driveFeedforward =
      new SimpleMotorFeedforward(
          Constants.ModuleControlConstants.kDriveKsVolts,
          Constants.ModuleControlConstants.kDriveKvVoltSecondsPerMeter,
          Constants.ModuleControlConstants.kDriveKaVoltSecondsSquaredPerMeter);
  
  private final SimpleMotorFeedforward turningFeedforward =
      new SimpleMotorFeedforward(
          Constants.ModuleControlConstants.kTurningKsVolts,
          Constants.ModuleControlConstants.kTurningKvVoltSecondsPerRadian,
          Constants.ModuleControlConstants.kTurningKaVoltSecondsSquaredPerRadian);
 
  private final VoltageOut driveVoltageRequest = new VoltageOut(0.0);
  private final VoltageOut turningVoltageRequest = new VoltageOut(0.0);

 
  private Rotation2d heldAngle = Rotation2d.kZero;

  private Rotation2d desiredAngle = Rotation2d.kZero;
  private double desiredSpeedMetersPerSecond;
  private double previousDesiredSpeedMetersPerSecond;
  private double cosineScale;
  private double baseDriveVoltage;
  private double driveCorrectionVoltage;
  private double commandedDriveVoltage;
  private double turningPidVoltage;
  private double turningFeedforwardVoltage;
  private double commandedTurningVoltage;

  public SwerveModule(Constants.ModuleConfig config) {
    // Build Phoenix devices on the configured CAN bus (need to do this later)
    this.config = config;
    driveMotor = new TalonFX(config.driveMotorId, Constants.DriveConstants.kCanBusName);
    turningMotor = new TalonFX(config.steerMotorId, Constants.DriveConstants.kCanBusName);
    turningEncoder = new CANcoder(config.cancoderId, Constants.DriveConstants.kCanBusName);
    configureMotor(
        driveMotor,
        config.driveMotorInverted,
        Constants.ModuleControlConstants.kDriveSupplyCurrentLimitAmps,
        Constants.ModuleControlConstants.kDriveStatorCurrentLimitAmps);
    configureMotor(
        turningMotor,
        config.steerMotorInverted,
        Constants.ModuleControlConstants.kSteerSupplyCurrentLimitAmps,
        Constants.ModuleControlConstants.kSteerStatorCurrentLimitAmps);
    driveVoltageRequest.EnableFOC = false;
    turningVoltageRequest.EnableFOC = false;
    turningPid.enableContinuousInput(-Math.PI, Math.PI);
    Rotation2d currentAngle = getCurrentAngle();
    if (Double.isFinite(currentAngle.getRadians())) {
      heldAngle = currentAngle;
      desiredAngle = currentAngle;
      turningPid.reset(currentAngle.getRadians());
    }
  }

  private void configureMotor(
      TalonFX motor, boolean inverted, double supplyCurrentLimit, double statorCurrentLimit) {
    TalonFXConfiguration motorConfiguration = new TalonFXConfiguration();
    motorConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    motorConfiguration.MotorOutput.Inverted =
        inverted ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
    motorConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true;
    motorConfiguration.CurrentLimits.SupplyCurrentLimit = supplyCurrentLimit;
    motorConfiguration.CurrentLimits.StatorCurrentLimitEnable = true;
    motorConfiguration.CurrentLimits.StatorCurrentLimit = statorCurrentLimit;
    motor.getConfigurator().apply(motorConfiguration);
  }

  public Rotation2d getCurrentAngle() {
    // Read the absolute angle and apply the direction
    double rawRotations = turningEncoder.getAbsolutePosition().getValueAsDouble();
    if (config.absoluteEncoderInverted) {
      rawRotations = -rawRotations;
    }
    return Rotation2d.fromRotations(rawRotations - config.absoluteEncoderOffsetRotations);
  }

  public SwerveModuleState getState() {
    
    double velocityMetersPerSecond =
        hasValidDriveMeasurement() ? getDriveVelocityMetersPerSecond() : 0.0;
    return new SwerveModuleState(velocityMetersPerSecond, getCurrentAngle());
  }

  public SwerveModulePosition getPosition() {
    
    double distanceMeters = hasValidDriveMeasurement() ? getDriveDistanceMeters() : 0.0;
    return new SwerveModulePosition(distanceMeters, getCurrentAngle());
  }

  public boolean hasValidDriveMeasurement() {
    
    return Constants.ModulePhysicalConstants.hasVerifiedDriveConversion();
  }

  private double getDriveVelocityMetersPerSecond() {
    
    if (!Constants.ModulePhysicalConstants.hasVerifiedDriveConversion()) {
      return Double.NaN;
    }
    return driveMotor.getVelocity().getValueAsDouble()
        * Constants.ModulePhysicalConstants.metersPerDriveMotorRotation();
  }

  private double getDriveDistanceMeters() {
    
    if (!Constants.ModulePhysicalConstants.hasVerifiedDriveConversion()) {
      return Double.NaN;
    }
    return driveMotor.getPosition().getValueAsDouble()
        * Constants.ModulePhysicalConstants.metersPerDriveMotorRotation();
  }

  public void setDesiredState(SwerveModuleState requestedState) {
    
    Rotation2d currentAngle = getCurrentAngle();
    if (!Double.isFinite(currentAngle.getRadians())
        || !Double.isFinite(requestedState.speedMetersPerSecond)
        || !Double.isFinite(requestedState.angle.getRadians())) {
      stop();
      return;
    }

    SwerveModuleState desiredState =
        new SwerveModuleState(requestedState.speedMetersPerSecond, requestedState.angle);
    desiredState.optimize(currentAngle);
    desiredState.cosineScale(currentAngle);

    if (Math.abs(desiredState.speedMetersPerSecond)
        < Constants.DriveConstants.kLowSpeedHoldThresholdMetersPerSecond) {
      desiredSpeedMetersPerSecond = 0.0;
      desiredAngle = heldAngle;
      cosineScale = 0.0;
    } else {
      desiredSpeedMetersPerSecond = desiredState.speedMetersPerSecond;
      desiredAngle = desiredState.angle;
      heldAngle = desiredAngle;
      cosineScale = desiredAngle.minus(currentAngle).getCos();
    }

    baseDriveVoltage = calculateBaseDriveVoltage(desiredSpeedMetersPerSecond);
    driveCorrectionVoltage = calculateDriveCorrectionVoltage(desiredSpeedMetersPerSecond);
    applyDriveVoltage(baseDriveVoltage + driveCorrectionVoltage);

    turningPidVoltage = turningPid.calculate(currentAngle.getRadians(), desiredAngle.getRadians());
    turningFeedforwardVoltage = turningFeedforward.calculate(turningPid.getSetpoint().velocity);
    applyTurningVoltage(turningPidVoltage + turningFeedforwardVoltage);
    previousDesiredSpeedMetersPerSecond = desiredSpeedMetersPerSecond;
  }

  private double calculateBaseDriveVoltage(double desiredSpeedMetersPerSecond) {
    
    if (Constants.ModuleControlConstants.kUseDriveFeedforward) {
      return driveFeedforward.calculateWithVelocities(
          previousDesiredSpeedMetersPerSecond, desiredSpeedMetersPerSecond);
    }
    double normalizedSpeed =
        desiredSpeedMetersPerSecond
            / Constants.DriveConstants.kCommissioningMaxModuleSpeedMetersPerSecond;
    return MathUtil.clamp(normalizedSpeed, -1.0, 1.0)
        * Constants.DriveConstants.kCommissioningMaxDriveVoltage;
  }

  private double calculateDriveCorrectionVoltage(double desiredSpeedMetersPerSecond) {
    
    if (!Constants.ModuleControlConstants.kEnableDriveVelocityCorrection
        || !Constants.ModulePhysicalConstants.hasVerifiedDriveConversion()) {
      return 0.0;
    }
    double measuredSpeedMetersPerSecond = getDriveVelocityMetersPerSecond();
    if (!Double.isFinite(measuredSpeedMetersPerSecond)) {
      return 0.0;
    }
    double correctionVoltage =
        Constants.ModuleControlConstants.kDriveKpVoltsPerMeterPerSecond
            * (desiredSpeedMetersPerSecond - measuredSpeedMetersPerSecond);
    return MathUtil.clamp(
        correctionVoltage,
        -Constants.ModuleControlConstants.kMaxDriveFeedbackVoltage,
        Constants.ModuleControlConstants.kMaxDriveFeedbackVoltage);
  }

  private void applyDriveVoltage(double requestedVoltage) {
    // dont allow za NaN and keep final drive voltage inside the cap limits
    double finiteVoltage = Double.isFinite(requestedVoltage) ? requestedVoltage : 0.0;
    commandedDriveVoltage =
        MathUtil.clamp(
            finiteVoltage,
            -Constants.DriveConstants.kCommissioningMaxDriveVoltage,
            Constants.DriveConstants.kCommissioningMaxDriveVoltage);
    driveMotor.setControl(driveVoltageRequest.withOutput(commandedDriveVoltage));
  }

  private void applyTurningVoltage(double requestedVoltage) {
    
    double finiteVoltage = Double.isFinite(requestedVoltage) ? requestedVoltage : 0.0;
    commandedTurningVoltage =
        MathUtil.clamp(
            finiteVoltage,
            -Constants.DriveConstants.kMaxSteeringVoltage,
            Constants.DriveConstants.kMaxSteeringVoltage);
    turningMotor.setControl(turningVoltageRequest.withOutput(commandedTurningVoltage));
  }

  public void stop() {
    // Reset steering planning
    Rotation2d currentAngle = getCurrentAngle();
    if (Double.isFinite(currentAngle.getRadians())) {
      heldAngle = currentAngle;
      desiredAngle = currentAngle;
      turningPid.reset(currentAngle.getRadians());
    }
    desiredSpeedMetersPerSecond = 0.0;
    previousDesiredSpeedMetersPerSecond = 0.0;
    cosineScale = 0.0;
    baseDriveVoltage = 0.0;
    driveCorrectionVoltage = 0.0;
    turningPidVoltage = 0.0;
    turningFeedforwardVoltage = 0.0;
    applyDriveVoltage(0.0);
    applyTurningVoltage(0.0);
  }

  public void publishTelemetry() {
    // read the angle once for dashboard update
    Rotation2d currentAngle = getCurrentAngle();
    SmartDashboard.putBoolean(
        config.name + "/Drive Conversion Configured",
        Constants.ModulePhysicalConstants.hasVerifiedDriveConversion());
    SmartDashboard.putNumber(config.name + "/Desired Velocity Mps", desiredSpeedMetersPerSecond);
    SmartDashboard.putNumber(
        config.name + "/Measured Velocity Mps",
        hasValidDriveMeasurement() ? getDriveVelocityMetersPerSecond() : 0.0);
    SmartDashboard.putNumber(config.name + "/Measured Angle Degrees", currentAngle.getDegrees());
    SmartDashboard.putNumber(config.name + "/Desired Angle Degrees", desiredAngle.getDegrees());
    SmartDashboard.putNumber(
        config.name + "/Steering Error Degrees",
        Math.toDegrees(desiredAngle.minus(currentAngle).getRadians()));
    SmartDashboard.putNumber(config.name + "/Cosine Scale", cosineScale);
    SmartDashboard.putNumber(config.name + "/Base Drive Voltage", baseDriveVoltage);
    SmartDashboard.putNumber(config.name + "/Drive P Correction Voltage", driveCorrectionVoltage);
    SmartDashboard.putNumber(config.name + "/Commanded Drive Voltage", commandedDriveVoltage);
    SmartDashboard.putNumber(config.name + "/Commanded Turning Voltage", commandedTurningVoltage);
    SmartDashboard.putNumber(
        config.name + "/Drive Supply Current Amps", driveMotor.getSupplyCurrent().getValueAsDouble());
    SmartDashboard.putNumber(
        config.name + "/Drive Stator Current Amps", driveMotor.getStatorCurrent().getValueAsDouble());
  }
}
