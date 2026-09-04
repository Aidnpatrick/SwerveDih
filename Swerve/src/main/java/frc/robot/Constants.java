package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

public final class Constants {
  private Constants() {}

  public static final class OperatorConstants {
    public static final int kControllerPort = 0;
    public static final double kDeadband = 0.05;

    private OperatorConstants() {}
  }

  public static final class DriveConstants {
    public static final String kCanBusName = "rio";
    public static final double kTrackWidthMeters = 0.550;
    public static final double kWheelBaseMeters = 0.555;
    public static final double kMaxModuleSpeedMetersPerSecond = 2.0;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kLowSpeedHoldThresholdMetersPerSecond = 0.05;
    public static final double kMaxDriveVoltage = 4.0;
    public static final double kMaxSteeringVoltage = 4.0;
    public static final double kTranslationSlewRateMetersPerSecondSquared = 3.0;
    public static final double kRotationSlewRateRadiansPerSecondSquared = 2.0 * Math.PI;
    public static final SwerveDriveKinematics kSwerveKinematics =
        new SwerveDriveKinematics(
            new Translation2d(kWheelBaseMeters / 2.0, kTrackWidthMeters / 2.0),
            new Translation2d(kWheelBaseMeters / 2.0, -kTrackWidthMeters / 2.0),
            new Translation2d(-kWheelBaseMeters / 2.0, kTrackWidthMeters / 2.0),
            new Translation2d(-kWheelBaseMeters / 2.0, -kTrackWidthMeters / 2.0));

    private DriveConstants() {}
  }

  public static final class DriveControlConstants {
    public static final double kDriveKpVoltsPerMeterPerSecond = 0.0;
    public static final double kDriveKiVoltsPerMeter = 0.0;
    public static final double kDriveKdVoltSecondsPerMeter = 0.0;
    public static final double kDriveKsVolts = 0.0;
    public static final double kDriveKvVoltSecondsPerMeter = 0.0;
    public static final double kDriveKaVoltSecondsSquaredPerMeter = 0.0;

    private DriveControlConstants() {}
  }

  public static final class TurningControlConstants {
    public static final double kTurningKpVoltsPerRadian = 0.5;
    public static final double kTurningKiVoltsPerRadianSecond = 0.0;
    public static final double kTurningKdVoltSecondsPerRadian = 0.0;
    public static final double kTurningKsVolts = 0.0;
    public static final double kTurningKvVoltSecondsPerRadian = 0.0;
    public static final double kTurningKaVoltSecondsSquaredPerRadian = 0.0;
    public static final double kTurningMaxVelocityRadiansPerSecond = Math.PI;
    public static final double kTurningMaxAccelerationRadiansPerSecondSquared = 2.0 * Math.PI;

    private TurningControlConstants() {}
  }

  public static final class ElectricalConstants {
    public static final double kDriveSupplyCurrentLimitAmps = 30.0;
    public static final double kDriveStatorCurrentLimitAmps = 60.0;
    public static final double kSteerSupplyCurrentLimitAmps = 15.0;
    public static final double kSteerStatorCurrentLimitAmps = 25.0;

    private ElectricalConstants() {}
  }

  public static final class ModulePhysicalConstants {
    public static final double kWheelDiameterMeters = Double.NaN;
    public static final double kDriveMotorRotationsPerWheelRotation = Double.NaN;

    public static boolean hasVerifiedDriveConversion() {
      return Double.isFinite(kWheelDiameterMeters)
          && kWheelDiameterMeters > 0.0
          && Double.isFinite(kDriveMotorRotationsPerWheelRotation)
          && kDriveMotorRotationsPerWheelRotation > 0.0;
    }

    public static double metersPerDriveMotorRotation() {
      if (!hasVerifiedDriveConversion()) {
        return 0.0;
      }
      return Math.PI * kWheelDiameterMeters / kDriveMotorRotationsPerWheelRotation;
    }

    private ModulePhysicalConstants() {}
  }

  public static final class ModuleConstants {
    public static final ModuleConfig kFrontLeft =
        new ModuleConfig("FrontLeft", 4, 5, 3, 0.0, false, false, false);
    public static final ModuleConfig kFrontRight =
        new ModuleConfig("FrontRight", 2, 3, 2, 0.0, false, false, false);
    public static final ModuleConfig kBackLeft =
        new ModuleConfig("BackLeft", 6, 7, 4, 0.0, false, false, false);
    public static final ModuleConfig kBackRight =
        new ModuleConfig("BackRight", 8, 1, 1, 0.0, false, false, false);

    private ModuleConstants() {}
  }

  public static final class ModuleConfig {
    public final String name;
    public final int driveMotorId;
    public final int steerMotorId;
    public final int cancoderId;
    public final double absoluteEncoderOffsetRotations;
    public final boolean driveMotorInverted;
    public final boolean steerMotorInverted;
    public final boolean absoluteEncoderInverted;

    private ModuleConfig(
        String name,
        int driveMotorId,
        int steerMotorId,
        int cancoderId,
        double absoluteEncoderOffsetRotations,
        boolean driveMotorInverted,
        boolean steerMotorInverted,
        boolean absoluteEncoderInverted) {
      this.name = name;
      this.driveMotorId = driveMotorId;
      this.steerMotorId = steerMotorId;
      this.cancoderId = cancoderId;
      this.absoluteEncoderOffsetRotations = absoluteEncoderOffsetRotations;
      this.driveMotorInverted = driveMotorInverted;
      this.steerMotorInverted = steerMotorInverted;
      this.absoluteEncoderInverted = absoluteEncoderInverted;
    }
  }
}
