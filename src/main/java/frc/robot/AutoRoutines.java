package frc.robot;

import choreo.Choreo;
import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.commands.AutoAlignCommand;
import frc.robot.commands.CommandFactory;
import java.util.Optional;

public class AutoRoutines {
  private final AutoFactory m_factory;
  private final Optional<Trajectory<SwerveSample>> testReef = Choreo.loadTrajectory("TestReef");

  public AutoRoutines(AutoFactory factory) {
    m_factory = factory;
  }

  public AutoRoutine testAuto1() {
    final AutoRoutine routine = m_factory.newRoutine("Test Auto 1");
    final AutoTrajectory testTraj = routine.trajectory("TestForward");

    routine.active().onTrue(testTraj.resetOdometry().andThen(testTraj.cmd()));
    return routine;
  }

  public AutoRoutine testAuto3() {
    final AutoRoutine routine = m_factory.newRoutine("Test Auto 3");
    final AutoTrajectory testTraj = routine.trajectory("TestReef");

    routine
        .active()
        .onTrue(
            Commands.sequence(
                testTraj.resetOdometry(),
                testTraj.cmd(),
                Commands.parallel(
                    new RunCommand(
                        () -> RobotContainer.coralWrist.moveWristToPosition(-0.38),
                        RobotContainer.coralWrist),
                    new RunCommand(
                        () -> RobotContainer.elevator.moveElevatorToPosition(0.22),
                        RobotContainer.elevator),
                    RobotContainer.coralIntake.intakeCommand(-.05))));
    return routine;
  }

  public AutoRoutine testAuto4() {
    final AutoRoutine routine = m_factory.newRoutine("Test Auto 4");
    final AutoTrajectory testTraj = routine.trajectory("TestReef");

    routine
        .active()
        .onTrue(
            Commands.sequence(
                testTraj.resetOdometry(),
                testTraj.cmd(),
                // RobotContainer.elevator.moveElevatorToL2Auto(),
                new RunCommand(
                    () -> RobotContainer.coralWrist.moveWristToL2(), RobotContainer.coralWrist)));
    return routine;
  }

  public AutoRoutine moveCenter() {
    final AutoRoutine routine = m_factory.newRoutine("moveCenter");
    final AutoTrajectory testTraj = routine.trajectory("CenterReef");

    routine
        .active()
        .onTrue(
            Commands.sequence(
                testTraj.resetOdometry(),
                testTraj.cmd(),
                // RobotContainer.elevator.moveElevatorToL2Auto(),
                new RunCommand(
                    () -> RobotContainer.coralWrist.moveWristToL2(), RobotContainer.coralWrist)));
    return routine;
  }

  public AutoRoutine centerRoutine() {
    final AutoRoutine routine = m_factory.newRoutine("centerRoutine");
    final AutoTrajectory testTraj = routine.trajectory("CenterReef");
    routine
        .active()
        .onTrue(
            Commands.sequence(
                testTraj.resetOdometry(),
                testTraj.cmd(),
                Commands.parallel(
                    new RunCommand(
                        () -> RobotContainer.coralWrist.moveWristToPosition(-0.42),
                        RobotContainer.coralWrist),
                    new RunCommand(
                        () -> RobotContainer.elevator.moveElevatorToPosition(0.1),
                        RobotContainer.elevator)),
                RobotContainer.coralIntake.intakeCommand(-.1)));
    return routine;
  }

  public AutoRoutine mAutoRoutine() {
    final AutoRoutine routine = m_factory.newRoutine("Main Auton");
    final AutoTrajectory testTraj = routine.trajectory("MainAuton");

    routine.active().onTrue(testTraj.resetOdometry().andThen(testTraj.cmd()));
    return routine;
  }

  public AutoRoutine visionAutoRoutine() {
    final AutoRoutine routine = m_factory.newRoutine("Vision Auton");
    final AutoTrajectory StartToReef = routine.trajectory("TestReef");
    final AutoTrajectory reefToHP = routine.trajectory("ReefToHP");
    final AutoTrajectory hpToReef = routine.trajectory("HPToReef");

    AutoAlignCommand autoAlignCommand =
        new AutoAlignCommand(RobotContainer.drivetrain, RobotContainer.reefCamera);

    routine
        .active()
        .onTrue(
            Commands.sequence(
                    StartToReef.resetOdometry().asProxy(),
                    Commands.parallel(
                            Commands.sequence(
                                    RobotContainer.coralWrist.moveWristToHP().asProxy(),
                                    RobotContainer.coralIntake.intakeCommand().asProxy())
                                .withName("Move Wrist and Intake Coral"),
                            StartToReef.cmd().asProxy())
                        .withName("Move and Intake Coral"),
                    autoAlignCommand.asProxy(),
                    CommandFactory.scoreL4Command().asProxy(),
                    reefToHP.cmd().asProxy(),
                    RobotContainer.coralIntake.intakeCommand().asProxy(),
                    hpToReef.cmd().asProxy(),
                    autoAlignCommand.asProxy(),
                    CommandFactory.scoreL4Command().asProxy())
                .withName("Vision Auton"));
    return routine;
  }
}
