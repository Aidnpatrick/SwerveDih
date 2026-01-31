import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

public class Swerve extends SubsystemBase {

    private SwerveModule frontL;
    private SwerveModule frontR;
    private SwerveModule backL;
    private SwerveModule backR;

    public Swerve() {
        frontL = new SwerveModule(
            Constants.frontLDriveMotorID,
            Constants.frontLSteerMotorID,
            Constants.frontLCANcoderID
        );

        frontR = new SwerveModule(
            Constants.frontRDriveMotorID,
            Constants.frontRSteerMotorID,
            Constants.frontRCANcoderID
        );

        backL = new SwerveModule(
            Constants.backLDriveMotorID,
            Constants.backLSteerMotorID,
            Constants.backLCANcoderID
        );

        backR = new SwerveModule(
            Constants.backRDriveMotorID,
            Constants.backRSteerMotorID,
            Constants.backRCANcoderID
        );
    }

    public void drive(ChassisSpeeds chassisSpeed) {
        SwerveModuleState[] moduleStates =
            Constants.swerveKinematics.toSwerveModuleStates(chassisSpeed);
        
        SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, 1)
        
        frontL.setSwerveState(moduleStates[0]);
        frontR.setSwerveState(moduleStates[1]);
        backL.setSwerveState(moduleStates[2]);
        backR.setSwerveState(moduleStates[3]);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("FrontL", frontL.angleEncoder.getAbsolutePosition().getValueAsDouble());
        SmartDashboard.putNumber("FrontR", frontR.angleEncoder.getAbsolutePosition().getValueAsDouble());
        SmartDashboard.putNumber("BackL", backL.angleEncoder.getAbsolutePosition().getValueAsDouble());
        SmartDashboard.putNumber("BackR", backR.angleEncoder.getAbsolutePosition().getValueAsDouble());

    }

}
