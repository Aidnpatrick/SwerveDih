package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
  // Command selected for autonomous mode.
  private Command autonomousCommand;
  // Owns subsystems, controllers, and bindings.
  private final RobotContainer robotContainer;

  public Robot() {
    robotContainer = new RobotContainer();
  }

  @Override
  public void robotPeriodic() {
    // Run commands and subsystem periodic methods.
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {
    // Clear motor output whenever the robot is disabled.
    robotContainer.stopDrive();
  }

  @Override
  public void autonomousInit() {
    // Request the selected autonomous command.
    autonomousCommand = robotContainer.getAutonomousCommand();
    if (autonomousCommand != null) {
      autonomousCommand.schedule();
    }
  }

  @Override
  public void teleopInit() {
    // Autonomous must not continue into driver control.
    if (autonomousCommand != null) {
      autonomousCommand.cancel();
    }
  }

  @Override
  public void testInit() {
    // Start test mode with no running commands.
    CommandScheduler.getInstance().cancelAll();
  }
}
