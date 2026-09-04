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
          Constants.TurningControlConstants.kTurningKpVoltsPerRadian,
          Constants.TurningControlConstants.kTurningKiVoltsPerRadianSecond,
          Constants.TurningControlConstants.kTurningKdVoltSecondsPerRadian,
          new TrapezoidProfile.Constraints(
              Constants.TurningControlConstants.kTurningMaxVelocityRadiansPerSecond,
              Constants.TurningControlConstants.kTurningMaxAccelerationRadiansPerSecondSquared));
  private final SimpleMotorFeedforward turningFeedforward =
      new SimpleMotorFeedforward(
          Constants.TurningControlConstants.kTurningKsVolts,
          Constants.TurningControlConstants.kTurningKvVoltSecondsPerRadian,
          Constants.TurningControlConstants.kTurningKaVoltSecondsSquaredPerRadian);
  private final VoltageOut driveVoltageRequest = new VoltageOut(0.0);
  private final VoltageOut turningVoltageRequest = new VoltageOut(0.0);

  private Rotation2d heldAngle = Rotation2d.kZero;
  private Rotation2d desiredAngle = Rotation2d.kZero;
  private double desiredSpeedMetersPerSecond;
  private double measuredSpeedMetersPerSecond;
  private double normalizedDirectDriveRequest;
  private double baseDirectDriveVoltage;
  private double driveCorrectionVoltage;
  private double commandedDriveVoltage;
  private double turningPidVoltage;
  private double turningFeedforwardVoltage;
  private double commandedTurningVoltage;

  public SwerveModule(Constants.ModuleConfig config) {
    this.config = config;
    driveMotor = new TalonFX(config.driveMotorId, Constants.DriveConstants.kCanBusName);
    turningMotor = new TalonFX(config.steerMotorId, Constants.DriveConstants.kCanBusName);
    turningEncoder = new CANcoder(config.cancoderId, Constants.DriveConstants.kCanBusName);
    configureMotor(
        driveMotor,
        config.driveMotorInverted,
        Constants.ElectricalConstants.kDriveSupplyCurrentLimitAmps,
        Constants.ElectricalConstants.kDriveStatorCurrentLimitAmps);
    configureMotor(
        turningMotor,
        config.steerMotorInverted,
        Constants.ElectricalConstants.kSteerSupplyCurrentLimitAmps,
        Constants.ElectricalConstants.kSteerStatorCurrentLimitAmps);
    driveVoltageRequest.EnableFOC = false;
    turningVoltageRequest.EnableFOC = false;
    turningPid.enableContinuousInput(-Math.PI, Math.PI);
    double currentAngleRadians = getCurrentAngleRadians();
    if (Double.isFinite(currentAngleRadians)) {
      heldAngle = Rotation2d.fromRadians(currentAngleRadians);
      desiredAngle = heldAngle;
      turningPid.reset(currentAngleRadians);
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

  private double getCurrentAngleRadians() {
    double rawRotations = turningEncoder.getAbsolutePosition().getValueAsDouble();
    if (!Double.isFinite(rawRotations)) {
      return Double.NaN;
    }
    if (config.absoluteEncoderInverted) {
      rawRotations = -rawRotations;
    }
    return Rotation2d.fromRotations(rawRotations - config.absoluteEncoderOffsetRotations).getRadians();
  }

  public Rotation2d getCurrentAngle() {
    double currentAngleRadians = getCurrentAngleRadians();
    return Rotation2d.fromRadians(Double.isFinite(currentAngleRadians) ? currentAngleRadians : 0.0);
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(
        hasValidDriveMeasurement() ? getDriveVelocityMetersPerSecond() : 0.0, getCurrentAngle());
  }

  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
        hasValidDriveMeasurement() ? getDriveDistanceMeters() : 0.0, getCurrentAngle());
  }

  public boolean hasValidDriveMeasurement() {
    return Constants.ModulePhysicalConstants.hasVerifiedDriveConversion();
  }

  public boolean hasValidTurningMeasurement() {
    return Double.isFinite(getCurrentAngleRadians());
  }

  private double getDriveVelocityMetersPerSecond() {
    if (!hasValidDriveMeasurement()) {
      return 0.0;
    }
    double motorRotationsPerSecond = driveMotor.getVelocity().getValueAsDouble();
    if (!Double.isFinite(motorRotationsPerSecond)) {
      return 0.0;
    }
    return (config.driveMotorInverted ? -1.0 : 1.0)
        * motorRotationsPerSecond
        * Constants.ModulePhysicalConstants.metersPerDriveMotorRotation();
  }

  private double getDriveDistanceMeters() {
    if (!hasValidDriveMeasurement()) {
      return 0.0;
    }
    double motorRotations = driveMotor.getPosition().getValueAsDouble();
    if (!Double.isFinite(motorRotations)) {
      return 0.0;
    }
    return (config.driveMotorInverted ? -1.0 : 1.0)
        * motorRotations
        * Constants.ModulePhysicalConstants.metersPerDriveMotorRotation();
  }

  public void setDesiredState(SwerveModuleState requestedState) {
    double currentAngleRadians = getCurrentAngleRadians();
    if (!Double.isFinite(currentAngleRadians)
        || !Double.isFinite(requestedState.speedMetersPerSecond)
        || !Double.isFinite(requestedState.angle.getRadians())) {
      stop();
      return;
    }

    Rotation2d currentAngle = Rotation2d.fromRadians(currentAngleRadians);
    SwerveModuleState desiredState =
        new SwerveModuleState(requestedState.speedMetersPerSecond, requestedState.angle);
    desiredState.optimize(currentAngle);
    desiredState.cosineScale(currentAngle);

    if (Math.abs(desiredState.speedMetersPerSecond)
        < Constants.DriveConstants.kLowSpeedHoldThresholdMetersPerSecond) {
      desiredSpeedMetersPerSecond = 0.0;
      desiredAngle = heldAngle;
    } else {
      desiredSpeedMetersPerSecond = desiredState.speedMetersPerSecond;
      desiredAngle = desiredState.angle;
      heldAngle = desiredAngle;
    }

    calculateDriveVoltage();
    applyDriveVoltage(baseDirectDriveVoltage + driveCorrectionVoltage);
    turningPidVoltage = turningPid.calculate(currentAngleRadians, desiredAngle.getRadians());
    turningFeedforwardVoltage = turningFeedforward.calculate(turningPid.getSetpoint().velocity);
    applyTurningVoltage(turningPidVoltage + turningFeedforwardVoltage);
  }

  private void calculateDriveVoltage() {
    measuredSpeedMetersPerSecond = 0.0;
    normalizedDirectDriveRequest = 0.0;
    baseDirectDriveVoltage = calculateBaseDirectDriveVoltage();
    driveCorrectionVoltage = calculateDriveCorrectionVoltage();
  }

  private double calculateBaseDirectDriveVoltage() {
    double maximumModuleSpeed = Constants.DriveConstants.kMaxModuleSpeedMetersPerSecond;
    if (!Double.isFinite(maximumModuleSpeed) || maximumModuleSpeed <= 0.0) {
      return 0.0;
    }
    normalizedDirectDriveRequest =
        MathUtil.clamp(desiredSpeedMetersPerSecond / maximumModuleSpeed, -1.0, 1.0);
    return normalizedDirectDriveRequest * Constants.DriveConstants.kMaxDriveVoltage;
  }

  private double calculateDriveCorrectionVoltage() {
    if (!hasValidDriveMeasurement()) {
      return 0.0;
    }

    measuredSpeedMetersPerSecond = getDriveVelocityMetersPerSecond();
    if (!Constants.DriveControlConstants.kEnableDriveVelocityCorrection
        || desiredSpeedMetersPerSecond == 0.0) {
      return 0.0;
    }

    double correctionVoltage =
        Constants.DriveControlConstants.kDriveKpVoltsPerMeterPerSecond
            * (desiredSpeedMetersPerSecond - measuredSpeedMetersPerSecond);
    if (!Double.isFinite(correctionVoltage)) {
      return 0.0;
    }
    return MathUtil.clamp(
        correctionVoltage,
        -Constants.DriveControlConstants.kMaxDriveCorrectionVoltage,
        Constants.DriveControlConstants.kMaxDriveCorrectionVoltage);
  }

  private void applyDriveVoltage(double requestedVoltage) {
    double finiteVoltage = Double.isFinite(requestedVoltage) ? requestedVoltage : 0.0;
    commandedDriveVoltage =
        MathUtil.clamp(
            finiteVoltage,
            -Constants.DriveConstants.kMaxDriveVoltage,
            Constants.DriveConstants.kMaxDriveVoltage);
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
    double currentAngleRadians = getCurrentAngleRadians();
    if (Double.isFinite(currentAngleRadians)) {
      heldAngle = Rotation2d.fromRadians(currentAngleRadians);
      desiredAngle = heldAngle;
      turningPid.reset(currentAngleRadians);
    }
    desiredSpeedMetersPerSecond = 0.0;
    measuredSpeedMetersPerSecond = 0.0;
    normalizedDirectDriveRequest = 0.0;
    baseDirectDriveVoltage = 0.0;
    driveCorrectionVoltage = 0.0;
    turningPidVoltage = 0.0;
    turningFeedforwardVoltage = 0.0;
    applyDriveVoltage(0.0);
    applyTurningVoltage(0.0);
  }

  public void publishTelemetry() {
    Rotation2d currentAngle = getCurrentAngle();
    SmartDashboard.putBoolean(config.name + "/Drive Measurement Valid", hasValidDriveMeasurement());
    SmartDashboard.putBoolean(config.name + "/Turning Measurement Valid", hasValidTurningMeasurement());
    SmartDashboard.putNumber(config.name + "/Desired Velocity Mps", desiredSpeedMetersPerSecond);
    SmartDashboard.putNumber(config.name + "/Measured Velocity Mps", measuredSpeedMetersPerSecond);
    SmartDashboard.putNumber(
        config.name + "/Velocity Error Mps", desiredSpeedMetersPerSecond - measuredSpeedMetersPerSecond);
    SmartDashboard.putNumber(
        config.name + "/Normalized Direct Drive Request", normalizedDirectDriveRequest);
    SmartDashboard.putNumber(config.name + "/Base Direct Drive Voltage", baseDirectDriveVoltage);
    SmartDashboard.putNumber(config.name + "/Drive P Correction Voltage", driveCorrectionVoltage);
    SmartDashboard.putNumber(config.name + "/Commanded Drive Voltage", commandedDriveVoltage);
    SmartDashboard.putNumber(config.name + "/Desired Angle Degrees", desiredAngle.getDegrees());
    SmartDashboard.putNumber(config.name + "/Measured Angle Degrees", currentAngle.getDegrees());
    SmartDashboard.putNumber(
        config.name + "/Steering Error Degrees",
        Math.toDegrees(desiredAngle.minus(currentAngle).getRadians()));
    SmartDashboard.putNumber(config.name + "/Commanded Turning Voltage", commandedTurningVoltage);
    SmartDashboard.putNumber(
        config.name + "/Drive Supply Current Amps", driveMotor.getSupplyCurrent().getValueAsDouble());
    SmartDashboard.putNumber(
        config.name + "/Drive Stator Current Amps", driveMotor.getStatorCurrent().getValueAsDouble());
  }
}
