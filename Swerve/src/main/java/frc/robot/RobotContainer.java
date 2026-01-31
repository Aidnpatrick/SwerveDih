import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.ArcadeDrive;
import frc.robot.subsystems.Swerve;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.ResetEncoders;


public class RobotContainer {
  private Swerve m_swerve = new Swerve();

  private CommandController controller = new CommandXboxController(Constants.controllerPortNumber);

  public RobotContainer() {
    m_swerve.setDefaultCommand(new ArcadeDrive(m_swerve, controller));
    configureBindings();

  }
  private void configureBindings(){

  }
  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");

  }
}