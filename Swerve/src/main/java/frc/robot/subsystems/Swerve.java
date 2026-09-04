package frc.robot.subsystems;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Swerve extends SubsystemBase {
  private static final int kTelemetryPeriodLoops = 5;

  private final SwerveModule frontLeft = new SwerveModule(Constants.ModuleConstants.kFrontLeft);
  private final SwerveModule frontRight = new SwerveModule(Constants.ModuleConstants.kFrontRight);
  private final SwerveModule backLeft = new SwerveModule(Constants.ModuleConstants.kBackLeft);
  private final SwerveModule backRight = new SwerveModule(Constants.ModuleConstants.kBackRight);

  private double commandedXSpeedMetersPerSecond;
  private double commandedYSpeedMetersPerSecond;
  private double commandedOmegaRadiansPerSecond;
  private int telemetryLoopCounter;

  public void drive(ChassisSpeeds chassisSpeeds) {
    if (!Double.isFinite(chassisSpeeds.vxMetersPerSecond)
        || !Double.isFinite(chassisSpeeds.vyMetersPerSecond)
        || !Double.isFinite(chassisSpeeds.omegaRadiansPerSecond)) {
      stop();
      return;
    }

    commandedXSpeedMetersPerSecond = chassisSpeeds.vxMetersPerSecond;
    commandedYSpeedMetersPerSecond = chassisSpeeds.vyMetersPerSecond;
    commandedOmegaRadiansPerSecond = chassisSpeeds.omegaRadiansPerSecond;

    SwerveModuleState[] moduleStates =
        Constants.DriveConstants.kSwerveKinematics.toSwerveModuleStates(chassisSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(
        moduleStates, Constants.DriveConstants.kMaxModuleSpeedMetersPerSecond);

    frontLeft.setDesiredState(moduleStates[0]);
    frontRight.setDesiredState(moduleStates[1]);
    backLeft.setDesiredState(moduleStates[2]);
    backRight.setDesiredState(moduleStates[3]);
  }

  public void stop() {
    commandedXSpeedMetersPerSecond = 0.0;
    commandedYSpeedMetersPerSecond = 0.0;
    commandedOmegaRadiansPerSecond = 0.0;
    frontLeft.stop();
    frontRight.stop();
    backLeft.stop();
    backRight.stop();
  }

  public SwerveModuleState[] getModuleStates() {
    return new SwerveModuleState[] {
      frontLeft.getState(), frontRight.getState(), backLeft.getState(), backRight.getState()
    };
  }

  public SwerveModulePosition[] getModulePositions() {
    return new SwerveModulePosition[] {
      frontLeft.getPosition(), frontRight.getPosition(), backLeft.getPosition(), backRight.getPosition()
    };
  }

  public boolean hasValidModulePositions() {
    return frontLeft.hasValidDriveMeasurement()
        && frontRight.hasValidDriveMeasurement()
        && backLeft.hasValidDriveMeasurement()
        && backRight.hasValidDriveMeasurement();
  }

  @Override
  public void periodic() {
    if (++telemetryLoopCounter % kTelemetryPeriodLoops != 0) {
      return;
    }

    SmartDashboard.putNumber("Swerve/Commanded X Speed Mps", commandedXSpeedMetersPerSecond);
    SmartDashboard.putNumber("Swerve/Commanded Y Speed Mps", commandedYSpeedMetersPerSecond);
    SmartDashboard.putNumber("Swerve/Commanded Omega Rad Per Sec", commandedOmegaRadiansPerSecond);
    SmartDashboard.putNumber("Swerve/Battery Voltage", RobotController.getBatteryVoltage());
    frontLeft.publishTelemetry();
    frontRight.publishTelemetry();
    backLeft.publishTelemetry();
    backRight.publishTelemetry();
  }
}
