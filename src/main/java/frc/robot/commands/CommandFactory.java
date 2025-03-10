package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;

public class CommandFactory {
  public static Command scoreL2Command() {
    AutoAlignCommand autoAlignCommand =
        new AutoAlignCommand(RobotContainer.drivetrain, RobotContainer.reefCamera);
    return Commands.sequence(
            autoAlignCommand.asProxy(),
            RobotContainer.elevator.moveElevatorToL2().asProxy(),
            RobotContainer.coralIntake.moveWristToL2().asProxy(),
            RobotContainer.coralIntake.outtakeCommand().asProxy(),
            idleCommand().asProxy())
        .withName("L2 Command Group");
  }

  public static Command scoreL3Command() {
    AutoAlignCommand autoAlignCommand =
        new AutoAlignCommand(RobotContainer.drivetrain, RobotContainer.reefCamera);
    return Commands.sequence(
            autoAlignCommand.asProxy(),
            RobotContainer.elevator.moveElevatorToL3().asProxy(),
            RobotContainer.coralIntake.moveWristToL3().asProxy(),
            RobotContainer.coralIntake.outtakeCommand().asProxy(),
            idleCommand().asProxy())
        .withName("L3 Command Group");
  }

  public static Command scoreL4Command() {
    AutoAlignCommand autoAlignCommand =
        new AutoAlignCommand(RobotContainer.drivetrain, RobotContainer.reefCamera);
    return Commands.sequence(
            autoAlignCommand.asProxy(),
            RobotContainer.elevator.moveElevatorToL4().asProxy(),
            RobotContainer.coralIntake.moveWristToL4().asProxy(),
            RobotContainer.coralIntake.outtakeCommand().asProxy(),
            idleCommand().asProxy())
        .withName("L4 Command Group");
  }

  public static Command hpCommand() {
    // AutoAlignCommand autoAlignCommand =
    //     new AutoAlignCommand(RobotContainer.drivetrain, RobotContainer.hpCamera);
    return Commands.sequence(
            // autoAlignCommand.asProxy(),
            RobotContainer.elevator.moveElevatorToHP().asProxy(),
            RobotContainer.coralIntake.moveWristToHP().asProxy(),
            RobotContainer.coralIntake.intakeCommand().asProxy())
        .withName("HP Command Group");
  }

  public static Command idleCommand() {
    return Commands.parallel(
            RobotContainer.elevator.moveElevatorToHP().asProxy(),
            RobotContainer.coralIntake.moveWristToHP().asProxy())
        .withName("Idle Command Group");
  }
}
