package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants;

public class SwerveModule {
    private TalonFX driveMotor;
    private TalonFX steerMotor;
    public CANcoder angleEncoder;

    private PIDController turnPIDController = new PIDController(Constants.kp,0,0);
    private double encoderOffset;

    public SwerveModule(int drivingMotorID, int steerMotorID, int angleEncoderID) {
        driveMotor = new TalonFX(drivingMotorID);
        steerMotor = new TalonFX(steerMotorID);
        angleEncoder = new CANcoder(angleEncoderID);

        driveMotor.setNeutralMode(NeutralModeValue.Brake);
        steerMotor.setNeutralMode(NeutralModeValue.Brake);

    }

    public double getCurrentAngle() {
        double rotations = angleEncoder.getAbsolutePosition().getValueAsDouble();
        if(rotations < -0.5)
            rotations +=1;
        else if(rotations > 0.5) 
            rotations -= 1;

        return rotations * 2 * Math.PI;
    }

    public void setSwerveState(SwerveModuleState desiredState) {
        driveMotor.set(desiredState.speedMetersPerSecond);
        steerMotor.set(-turnPIDController.calculate(getCurrentAngle(), desiredState.angle.getRadians()));

    }
}