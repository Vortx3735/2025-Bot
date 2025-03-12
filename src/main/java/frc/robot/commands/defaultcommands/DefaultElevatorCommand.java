package frc.robot.commands.defaultcommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;

public class DefaultElevatorCommand extends Command {
  private final Elevator m_elevator;
  private double holdPos;

  public DefaultElevatorCommand(Elevator elevator) {
    m_elevator = elevator;
    addRequirements(m_elevator);
    m_elevator.stopElevator();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    holdPos = m_elevator.getPosition();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_elevator.hold(holdPos);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    holdPos = 1;
    m_elevator.stopElevator();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
