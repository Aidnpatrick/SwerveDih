package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.ArcadeDrive;
import frc.robot.subsystems.Swerve;

public class RobotContainer {
  private final Swerve swerve = new Swerve();
  private final CommandXboxController controller =
      new CommandXboxController(Constants.OperatorConstants.kControllerPort);

  public RobotContainer() {
    swerve.setDefaultCommand(new ArcadeDrive(swerve, controller));
  }

  public void stopDrive() {
    swerve.stop();
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
