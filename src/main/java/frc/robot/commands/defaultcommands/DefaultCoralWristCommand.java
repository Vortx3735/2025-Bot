package frc.robot.commands.defaultcommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralWrist;

public class DefaultCoralWristCommand extends Command {
  private final CoralWrist m_CoralWrist;

  double position;

  /**
   * Default Coral Intake Command
   *
   * @param subsystem The subsystem used by this command.
   */
  public DefaultCoralWristCommand(CoralWrist subsystem) {
    m_CoralWrist = subsystem;
    addRequirements(m_CoralWrist);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    position = m_CoralWrist.getWristPosition();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_CoralWrist.hold(position);
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
