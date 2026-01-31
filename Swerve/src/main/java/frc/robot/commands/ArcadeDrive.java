package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;


public class ArcadeDrive extends Command {
    private Swerve m_serve;
    private CommandXboxController controller;
    
    public ArcadeDrive(Swerve m_swerve, CommandXboxController controller) {
        this.m_serve = m_swerve;
        this.controller = controller;
        addRequiements(m_swerve);
        
    }
    public void initialize() {

    }
    public void execute() {
        double x = -controller.getRawAxis(1);
        double y = -controller.getRawAxis(0);
        double zRot = -controller.getRawAxis(4);

        if(Math.abs(x) < Constants.DEADBAND) x = 0;
        if(Math.abs(y) < Constants.DEADBAND) y = 0;
        if(Math.abs(zRot) < Constants.DEADBAND) zRot = 0;

        x*= Constants.xPercent;
        y *= Constants.yPercent;
        zRot *= Constants.zPercent;
        if(controller.rightBumper().getAsBoolean()) {
            m_swerve.drive(new ChassisSpeeds(x,y,zRot));
        }
        else {
            m_swerve.drive(ChassisSpeeds.fromFieldRelativeSpeeds(x,y,zRot,m_swerve.getHeading));
        }
    }

    public void end(boolean interrupted) {

    }

    public boolean isFinished() {
        return false;
    }
}
