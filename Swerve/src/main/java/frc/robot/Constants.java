package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

public class Constants {

    public static final double trackWidth = 0.550;
    public static final double wheelBase = 0.555;

    public static final SwerveDriveKinematics swerveKinematics =
        new SwerveDriveKinematics(
            new Translation2d(wheelBase / 2.0,  trackWidth / 2.0),
            new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0,  trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0)
        );

    public static final double xPercent = 0.2;
    public static final double yPercent = 0.2;
    public static final double zPercent = 0.2;
    public static final double zOffset = (zPercent)/(Math.sqrt((trackWidth/2)*(trackWidth/2)*(wheelBase/2)*(wheelBase/2)));
    
    public static final double DEADBAND = 0.05;

    public static final double kp = 0.5;

    public static final int frontLSteerMotorID = 5;
    public static final int frontLDriveMotorID = 4;
    public static final int frontLCANcoderID = 3;

    public static final int frontRSteerMotorID = 3;
    public static final int frontRDriveMotorID = 2;
    public static final int frontRCANcoderID = 2;

    public static final int backLSteerMotorID = 7;
    public static final int backLDriveMotorID = 6;
    public static final int backLCANcoderID = 4;

    public static final int backRSteerMotorID = 1;
    public static final int backRDriveMotorID = 8;
    public static final int backRCANcoderID = 1;
}
