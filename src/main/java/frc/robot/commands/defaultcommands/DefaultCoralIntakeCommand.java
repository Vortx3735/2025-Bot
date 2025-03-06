package frc.robot.commands.defaultcommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralIntake;

public class DefaultCoralIntakeCommand extends Command {
  private final CoralIntake m_CoralIntake;

  /**
   * Default Coral Intake Command
   *
   * @param subsystem The subsystem used by this command.
   */
  public DefaultCoralIntakeCommand(CoralIntake subsystem) {
    m_CoralIntake = subsystem;
    addRequirements(m_CoralIntake);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // // m_CoralIntake.hold();

    // if (!m_CoralIntake.getCoralIntakeBeam().getAsBoolean()) {
    //   m_CoralIntake.stopIntake();
    //  // Stop intake motor
    // } else {
    //   m_CoralIntake.moveWristToHP(); // Move to Human Player position
    //   m_CoralIntake.intake(); // Run intake motor
    // }

    // if (!m_CoralIntake.hasCoral()) {
    //  // Stop intake motor
    //  m_CoralIntake.moveWristToHP();
    // }

    // m_CoralIntake.stopIntake();

    m_CoralIntake.moveWristToPosition(m_CoralIntake.getWristPosition());

    // if (m_CoralIntake.getWristPosition() > -0.36) {
    //   m_CoralIntake.stopWrist();
    // } else {
    //   m_CoralIntake.moveWristToPosition(m_CoralIntake.getWristPosition());
    // }

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
