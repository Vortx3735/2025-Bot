package frc.robot.commands.defaultcommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaeIntake;

public class DefaultAlgaeIntakeCommand extends Command {
  private final AlgaeIntake m_AlgaeIntake;

  /**
   * Creates a new DefaultAlgaeIntakeCommand
   *
   * @param subsystem The subsystem used by this command.
   */
  public DefaultAlgaeIntakeCommand(AlgaeIntake subsystem) {
    m_AlgaeIntake = subsystem;
    addRequirements(m_AlgaeIntake);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_AlgaeIntake.stopIntake();
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
