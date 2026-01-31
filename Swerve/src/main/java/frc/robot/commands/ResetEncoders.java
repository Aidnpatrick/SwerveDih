package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.SwerveModule;

public class ResetEncoders {
    public SwerveModule m_Swerve;
    public ResetEncoders(SwerveModule m_Swerve) {
        this.m_Swerve = m_Swerve;
        addRequiements(m_Swerve);
    }
    @Override
    public void initialize() {
        m_Swerve.resetHeading();
    }
}
