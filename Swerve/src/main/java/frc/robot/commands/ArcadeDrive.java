package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

public class ArcadeDrive extends Command {
  // Drivetrain controlled by this command
  private final Swerve swerve;
  // Xbox controller that supplies driver input
  private final CommandXboxController controller;
  // Smooth forward
  private final SlewRateLimiter xLimiter =
      new SlewRateLimiter(Constants.DriveConstants.kTranslationSlewRateMetersPerSecondSquared);
  // Smooth left and right command changes
  private final SlewRateLimiter yLimiter =
      new SlewRateLimiter(Constants.DriveConstants.kTranslationSlewRateMetersPerSecondSquared);
  // Smooth rotation command change
  private final SlewRateLimiter rotationLimiter =
      new SlewRateLimiter(Constants.DriveConstants.kRotationSlewRateRadiansPerSecondSquared);

  public ArcadeDrive(Swerve swerve, CommandXboxController controller) {
    this.swerve = swerve;
    this.controller = controller;
    addRequirements(swerve);
  }

  @Override
  public void initialize() {
    // Begin each drive command from zero speed demanded by greedy humans
    xLimiter.reset(0.0);
    yLimiter.reset(0.0);
    rotationLimiter.reset(0.0);
  }

  @Override
  public void execute() {
    // forward&backward, strafe, and rotation sticks.
    double xInput = MathUtil.applyDeadband(-controller.getRawAxis(1), Constants.OperatorConstants.kDeadband);
    double yInput = MathUtil.applyDeadband(-controller.getRawAxis(0), Constants.OperatorConstants.kDeadband);
    double rotationInput =
        MathUtil.applyDeadband(-controller.getRawAxis(4), Constants.OperatorConstants.kDeadband);
    // Convert controller inputs into robot relative speeds (this prob wont work which is why this branch is experimental)
    double xSpeed =
        xLimiter.calculate(
            xInput * Constants.DriveConstants.kCommissioningMaxModuleSpeedMetersPerSecond);
    double ySpeed =
        yLimiter.calculate(
            yInput * Constants.DriveConstants.kCommissioningMaxModuleSpeedMetersPerSecond);
    double rotationSpeed =
        rotationLimiter.calculate(
            rotationInput * Constants.DriveConstants.kMaxAngularSpeedRadiansPerSecond);
    // Send the chassis request to WPILib swerve kinematics which does ts for me
    swerve.drive(new ChassisSpeeds(xSpeed, ySpeed, rotationSpeed));
  }

  @Override
  public void end(boolean interrupted) {
    // close the old drive voltage before we blow up
    swerve.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
