package frc.robot.commands.defaultcommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaeWrist;

public class DefaultAlgaeWristCommand extends Command {
  private final AlgaeWrist m_AlgaeWrist;

  double position;

  /**
   * Creates a new DefaultAlgaeWristCommand
   *
   * @param subsystem The subsystem used by this command.
   */
  public DefaultAlgaeWristCommand(AlgaeWrist subsystem) {
    m_AlgaeWrist = subsystem;
    addRequirements(m_AlgaeWrist);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    position = m_AlgaeWrist.getWristPosition();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_AlgaeWrist.hold(position);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_AlgaeWrist.stowWrist();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
