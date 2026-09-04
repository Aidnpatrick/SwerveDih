package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

// Central home for robot settings and hardware mappings.
public final class Constants {
  private Constants() {}

  public static final class OperatorConstants {
    // USB port for za xbox controller
    public static final int kControllerPort = 0;
    // Ignore small controller noise (recommended by docs)
    public static final double kDeadband = 0.05;

    private OperatorConstants() {}
  }

  // btw NAN means not a number, cool ig


  public static final class DriveConstants {
    // Active CAN bus name (not sure if we have systemcore yet)
    public static final String kCanBusName = "SystemCore";
    
    // distance between modules (left and right centers)
    public static final double kTrackWidthMeters = 0.550;
    
    // Front and back distance between module centers
    public static final double kWheelBaseMeters = 0.555;
    
    // Safe top wheel speed used during early testing.
    public static final double kCommissioningMaxModuleSpeedMetersPerSecond = 2.0;
    
    // Maximum commanded robot rotate rate
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    
    // Hold the last angle below the wheel speed
    public static final double kLowSpeedHoldThresholdMetersPerSecond = 0.05;
    
    // voltage cap (needs to be fixed)
    public static final double kCommissioningMaxDriveVoltage = 4.0;
    
    // steering voltage cap
    public static final double kMaxSteeringVoltage = 4.0;
    
    // Maximum driver commanded translation acceleration speed (not made by me)
    public static final double kTranslationSlewRateMetersPerSecondSquared = 3.0;
    
    // Maximum rotation acceleration
    public static final double kRotationSlewRateRadiansPerSecondSquared = 2.0 * Math.PI;
    
    // Converts robot chassis motion into four module state
    public static final SwerveDriveKinematics kSwerveKinematics =
        new SwerveDriveKinematics(
            new Translation2d(kWheelBaseMeters / 2.0, kTrackWidthMeters / 2.0),
            new Translation2d(kWheelBaseMeters / 2.0, -kTrackWidthMeters / 2.0),
            new Translation2d(-kWheelBaseMeters / 2.0, kTrackWidthMeters / 2.0),
            new Translation2d(-kWheelBaseMeters / 2.0, -kTrackWidthMeters / 2.0));

    private DriveConstants() {}
  }

  public static final class ModuleControlConstants {
    // True after drive feedforward is defined
    public static final boolean kUseDriveFeedforward = false;
    
    // Switch to true after wheel speed conversion and P tuning are verified (p tuning is just how aggressive the controller reacts ot error)
    public static final boolean kEnableDriveVelocityCorrection = false;
    
    // teensy weensy P correction
    public static final double kDriveKpVoltsPerMeterPerSecond = 0.0;
    
    // Limits how much drive feedback can change the base voltag
    public static final double kMaxDriveFeedbackVoltage = 0.5;
    
    
    public static final double kDriveKsVolts = 0.0;
    
    // Drive feedforward velocity voltage.
    public static final double kDriveKvVoltSecondsPerMeter = 0.0;
    
    // Drive feedforward acceleration voltage.
    public static final double kDriveKaVoltSecondsSquaredPerMeter = 0.0;
    
    
    public static final double kTurningKpVoltsPerRadian = 0.5;
    
    // Steering integral gain starts disabled for pid
    public static final double kTurningKiVoltsPerRadianSecond = 0.0;
    
    // Steering derivative gain starts disabled just like integral term
    public static final double kTurningKdVoltSecondsPerRadian = 0.0;
    
    // Steering feedforward for friction (yes gemeni recommended)
    public static final double kTurningKsVolts = 0.0;
    
    // Steering feedforward velocity voltage
    public static final double kTurningKvVoltSecondsPerRadian = 0.0;
    
    // Steering feedforward acceleration voltage, yes this is diff than velocity voltage
    public static final double kTurningKaVoltSecondsSquaredPerRadian = 0.0;
    
    // Fastest planned steering movement cap
    public static final double kTurningMaxVelocityRadiansPerSecond = Math.PI;
    
    // Fastest planned steering acceleration (need to verify later becuz gemini recommended this)
    public static final double kTurningMaxAccelerationRadiansPerSecondSquared = 2.0 * Math.PI;
    
    // Temporary drive supply current limit gotta limit the power we allocate to diff parts to stop brownies
    public static final double kDriveSupplyCurrentLimitAmps = 30.0;
    
    public static final double kDriveStatorCurrentLimitAmps = 60.0;
    
    // Limit on current supply to prevent brownout
    public static final double kSteerSupplyCurrentLimitAmps = 15.0;
    
    // more limits on the steer stator
    public static final double kSteerStatorCurrentLimitAmps = 25.0;

    private ModuleControlConstants() {}
  }

  public static final class ModulePhysicalConstants {
    //I DO NOT KNOW THE WHEEL DIAMETER
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
        return Double.NaN;
      }
      return Math.PI * kWheelDiameterMeters / kDriveMotorRotationsPerWheelRotation;
    }

    private ModulePhysicalConstants() {}
  }

  public static final class ModuleConstants {
    // CAN IDs, encoder vals, for za modules
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
    // TalonFX CAN ID
    public final int driveMotorId;
    // TalonFX CAN ID for steers
    public final int steerMotorId;
    // CANcoder CAN ID for absolute steering angle
    public final int cancoderId;
    // Raw CANcoder rotations when the wheel points forward
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
