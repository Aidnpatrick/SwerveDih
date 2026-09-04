package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.ArcadeDrive;
import frc.robot.subsystems.Swerve;

public class RobotContainer {
  // Main four-module drivetrain subsystem.
  private final Swerve swerve = new Swerve();
  // Driver input device.
  private final CommandXboxController controller =
      new CommandXboxController(Constants.OperatorConstants.kControllerPort);

  public RobotContainer() {
    // Drive whenever no other command owns the drivetrain.
    swerve.setDefaultCommand(new ArcadeDrive(swerve, controller));
  }

  public void stopDrive() {
    // Used by robot mode transitions.
    swerve.stop();
  }

  public Command getAutonomousCommand() {
    // Autonomous routines have not been added yet.
    return Commands.print("No autonomous command configured");
  }
}
