package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.CoralIntake;
import frc.robot.subsystems.CoralWrist;

public class CoralHPCommand extends Command {
  private final CoralIntake m_CoralIntake;
  private final CoralWrist m_CoralWrist;
  double pos;
  double speed;

  /**
   * Default Coral Intake Command
   *
   * @param subsystem The subsystem used by this command.
   */
  public CoralHPCommand(double position, double intakeSpeed) {
    m_CoralIntake = RobotContainer.coralIntake;
    m_CoralWrist = RobotContainer.coralWrist;
    addRequirements(m_CoralIntake, m_CoralWrist);
    pos = position;
    speed = intakeSpeed;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println(pos);
    m_CoralIntake.intake(speed);
    m_CoralWrist.hold(pos);
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
