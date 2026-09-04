package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

public class ArcadeDrive extends Command {
  private final Swerve swerve;
  private final CommandXboxController controller;
  private final SlewRateLimiter xLimiter =
      new SlewRateLimiter(Constants.DriveConstants.kTranslationSlewRateMetersPerSecondSquared);
  private final SlewRateLimiter yLimiter =
      new SlewRateLimiter(Constants.DriveConstants.kTranslationSlewRateMetersPerSecondSquared);
  private final SlewRateLimiter rotationLimiter =
      new SlewRateLimiter(Constants.DriveConstants.kRotationSlewRateRadiansPerSecondSquared);

  public ArcadeDrive(Swerve swerve, CommandXboxController controller) {
    this.swerve = swerve;
    this.controller = controller;
    addRequirements(swerve);
  }

  @Override
  public void initialize() {
    xLimiter.reset(0.0);
    yLimiter.reset(0.0);
    rotationLimiter.reset(0.0);
  }

  @Override
  public void execute() {
    double xInput =
        MathUtil.applyDeadband(-controller.getRawAxis(1), Constants.OperatorConstants.kDeadband);
    double yInput =
        MathUtil.applyDeadband(-controller.getRawAxis(0), Constants.OperatorConstants.kDeadband);
    double rotationInput =
        MathUtil.applyDeadband(-controller.getRawAxis(4), Constants.OperatorConstants.kDeadband);
    double xSpeed =
        xLimiter.calculate(
            xInput * Constants.DriveConstants.kMaxModuleSpeedMetersPerSecond);
    double ySpeed =
        yLimiter.calculate(
            yInput * Constants.DriveConstants.kMaxModuleSpeedMetersPerSecond);
    double rotationSpeed =
        rotationLimiter.calculate(
            rotationInput * Constants.DriveConstants.kMaxAngularSpeedRadiansPerSecond);
    swerve.drive(new ChassisSpeeds(xSpeed, ySpeed, rotationSpeed));
  }

  @Override
  public void end(boolean interrupted) {
    swerve.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
