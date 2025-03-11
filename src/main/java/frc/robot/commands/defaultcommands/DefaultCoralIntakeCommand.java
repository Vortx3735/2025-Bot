package frc.robot.commands.defaultcommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralIntake;

public class DefaultCoralIntakeCommand extends Command {
  private final CoralIntake m_CoralIntake;

  double position;

  /**
   * Default Coral Intake Command
   *
   * @param subsystem The subsystem used by this command.
   */
  public DefaultCoralIntakeCommand(CoralIntake subsystem) {
    m_CoralIntake = subsystem;
    addRequirements(m_CoralIntake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_CoralIntake.stopIntake();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
