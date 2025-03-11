package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;

public class CommandFactory {
  public static Command scoreL2Command() {
    CommandScheduler.getInstance().cancelAll();
    return Commands.sequence(
            RobotContainer.elevator.moveElevatorToL2().asProxy(),
            Commands.parallel(
                RobotContainer.coralIntake.moveWristToL2().asProxy(),
                RobotContainer.coralIntake.outtakeCommand().asProxy()),
            idleCommand().asProxy())
        .withName("L2 Command Group");
  }

  public static Command scoreL3Command() {
    CommandScheduler.getInstance().cancelAll();
    return Commands.sequence(
            RobotContainer.elevator.moveElevatorToL3().asProxy(),
            RobotContainer.coralIntake.moveWristToL3().asProxy().withTimeout(2),
            RobotContainer.coralIntake.outtakeCommand().asProxy(),
            idleCommand().asProxy())
        .withName("L3 Command Group");
  }

  public static Command scoreL4Command() {
    CommandScheduler.getInstance().cancelAll();
    return Commands.sequence(
            RobotContainer.elevator.moveElevatorToL4().asProxy(),
            RobotContainer.coralIntake.moveWristToL4().asProxy().withTimeout(2),
            RobotContainer.coralIntake.outtakeCommand().asProxy(),
            idleCommand().asProxy())
        .withName("L4 Command Group");
  }

  public static Command hpCommand() {
    CommandScheduler.getInstance().cancelAll();
    // AutoAlignCommand autoAlignCommand =
    //     new AutoAlignCommand(RobotContainer.drivetrain, RobotContainer.hpCamera);
    return Commands.parallel(
            // autoAlignCommand.asProxy(),
            RobotContainer.elevator.moveElevatorToHP().asProxy(),
            RobotContainer.coralIntake.coralHPCommand().asProxy())
        .withName("HP Command Group");
  }

  private static Command idleCommand() {
    return Commands.parallel(
            RobotContainer.elevator.moveElevatorToHP().asProxy(),
            RobotContainer.coralIntake.moveWristToHP().asProxy())
        .withName("Idle Command Group");
  }
}
