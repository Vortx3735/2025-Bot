package frc.robot.commands;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeCoralOnFly;

public class CommandFactory {
  public static Command movetoL2Command() {
    CommandScheduler.getInstance().cancelAll();
    return Commands.parallel(
            RobotContainer.elevator.moveElevatorToL2().asProxy(),
            RobotContainer.coralWrist.moveWristToL2().asProxy())
        .withName("Move to L2 Command Group");
  }

  public static Command movetoL3Command() {
    CommandScheduler.getInstance().cancelAll();
    return Commands.parallel(
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
    return Commands.parallel(
            RobotContainer.elevator.moveElevatorToL4().asProxy(),
            RobotContainer.coralWrist.moveWristToL4().asProxy())
        .withName("Move to L3 Command Group");
  }

  public static Command scoreL4Command() {
    CommandScheduler.getInstance().cancelAll();
    return Commands.sequence(
            movetoL4Command(),
            new WaitCommand(0.2),
            Commands.race(
                new WaitCommand(0.5), RobotContainer.coralIntake.outtakeCommand().asProxy()),
            idleCommand().asProxy())
        .withName("Score L4 Command Group");
  }

  public static Command ScoreL4CommandSim(SwerveDriveSimulation driveSimulation) {
    CommandScheduler.getInstance().cancelAll();
    return Commands.sequence(
            RobotContainer.elevator.moveElevatorToL4().asProxy(),
            RobotContainer.coralWrist.moveWristToL4().asProxy(),
            Commands.race(
                new WaitCommand(2),
                new InstantCommand(
                        () ->
                            SimulatedArena.getInstance()
                                .addGamePieceProjectile(
                                    new ReefscapeCoralOnFly(
                                        driveSimulation
                                            .getSimulatedDriveTrainPose()
                                            .getTranslation(),
                                        new Translation2d(0.6, 0.2),
                                        driveSimulation
                                            .getDriveTrainSimulatedChassisSpeedsFieldRelative(),
                                        driveSimulation.getSimulatedDriveTrainPose().getRotation(),
                                        Meters.of(2),
                                        MetersPerSecond.of(1.5),
                                        Degrees.of(-80))))
                    .andThen(new WaitCommand(2)),
                RobotContainer.coralWrist.moveWristUpSlow().asProxy()),
            idleCommand().asProxy())
        .withName("Score L4 Command Group Sim");
  }

  public static Command hpCommand() {
    CommandScheduler.getInstance().cancelAll();
    // AutoAlignCommand autoAlignCommand =
    // new AutoAlignCommand(RobotContainer.drivetrain, RobotContainer.hpCamera);
    return Commands.parallel(
            // autoAlignCommand.asProxy(),
            RobotContainer.elevator.moveElevatorToHP(),
            RobotContainer.coralWrist.moveWristToHP(),
            RobotContainer.coralIntake.intakeCommand())
        .withName("HP Command Group");
  }

  public static Command idleCommand() {
    return Commands.parallel(
            RobotContainer.elevator.moveElevatorToHP().asProxy(),
            RobotContainer.coralWrist.moveWristToHP().asProxy())
        .withName("Idle Command Group");
  }
}
