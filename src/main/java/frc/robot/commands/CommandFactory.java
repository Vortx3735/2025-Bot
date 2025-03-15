package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;

public class CommandFactory {
  public static Command movetoL2Command() {
    CommandScheduler.getInstance().cancelAll();
    return Commands.sequence(
            RobotContainer.elevator.moveElevatorToL2().asProxy(),
            RobotContainer.coralWrist.moveWristToL2().asProxy())
        .withName("Move to L2 Command Group");
  }

  public static Command movetoL3Command() {
    CommandScheduler.getInstance().cancelAll();
    return Commands.sequence(
            RobotContainer.elevator.moveElevatorToL3().asProxy(),
            RobotContainer.coralWrist.moveWristToL3().asProxy())
        .withName("Move to L3 Command Group");
  }

  public static Command outtakeCommand() {
    CommandScheduler.getInstance().cancelAll();
    return Commands.sequence(
            RobotContainer.coralIntake.outtakeCommand().asProxy(), idleCommand().asProxy())
        .withName("Outtake Command Group");
  }

  public static Command scoreL2Command() {
    CommandScheduler.getInstance().cancelAll();
    return Commands.sequence(movetoL2Command(), outtakeCommand())
        .withName("Score L2 Command Group");
  }

  public static Command scoreL3Command() {
    CommandScheduler.getInstance().cancelAll();
    return Commands.sequence(movetoL2Command(), outtakeCommand(), idleCommand())
        .withName("Score L3 Command Group");
  }

  public static Command movetoL4Command() {
    CommandScheduler.getInstance().cancelAll();
    return Commands.sequence(
            RobotContainer.elevator.moveElevatorToL4().asProxy(),
            RobotContainer.coralWrist.moveWristToL4().asProxy())
        .withName("Move to L3 Command Group");
  }

  public static Command scoreL4Command() {
    CommandScheduler.getInstance().cancelAll();
    return Commands.sequence(
            RobotContainer.elevator.moveElevatorToL4().asProxy(),
            RobotContainer.coralWrist.moveWristToL4().asProxy(),
            Commands.race(
                new WaitCommand(2),
                RobotContainer.coralIntake.outtakeCommand().asProxy(),
                RobotContainer.coralWrist.moveWristUpSlow().asProxy()),
            idleCommand().asProxy())
        .withName("Score L4 Command Group");
  }

  public static Command hpCommand() {
    CommandScheduler.getInstance().cancelAll();
    // AutoAlignCommand autoAlignCommand =
    //     new AutoAlignCommand(RobotContainer.drivetrain, RobotContainer.hpCamera);
    return Commands.parallel(
            // autoAlignCommand.asProxy(),
            RobotContainer.elevator.moveElevatorToHP().asProxy(),
            RobotContainer.coralWrist.moveWristToHP().asProxy())
        .andThen(RobotContainer.coralIntake.intakeCommand())
        .withName("HP Command Group");
  }

  public static Command idleCommand() {
    return Commands.parallel(
            RobotContainer.elevator.moveElevatorToHP().asProxy(),
            RobotContainer.coralWrist.moveWristToHP().asProxy())
        .withName("Idle Command Group");
  }
}
