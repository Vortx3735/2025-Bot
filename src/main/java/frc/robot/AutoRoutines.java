package frc.robot;

import choreo.Choreo;
import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import choreo.trajectory.*;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
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
                        () -> RobotContainer.coralIntake.moveWristToPosition(-0.38),
                        RobotContainer.coralIntake),
                    new RunCommand(
                        () -> RobotContainer.elevator.moveElevatorToPosition(0.22),
                        RobotContainer.elevator),
                    new RunCommand(
                        () -> RobotContainer.coralIntake.intakeAtSpeed(-0.05),
                        RobotContainer.coralIntake))));
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
                    () -> RobotContainer.coralIntake.L2Auto(), RobotContainer.coralIntake)));
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
                    () -> RobotContainer.coralIntake.L2Auto(), RobotContainer.coralIntake)));
    return routine;
  }

  // public AutoRoutine testAuto2() {
  // final AutoRoutine routine = m_factory.newRoutine("Test Auto 2");
  // // final AutoTrajectory testTraj = routine.trajectory("TestReef");

  // routine.active().onTrue(testReef.resetOdometry().andThen(testTraj.cmd()));
  // routine.active().onTrue(
  // new ParallelCommandGroup(
  // testTraj.resetOdometry(),
  // new FollowTrajectoryCommand(testTraj, ),
  // new RunCommand(() -> coralIntake.moveWristToL2(), coralIntake),
  // new RunCommand(() -> elevator.moveElevatorToL2(), elevator),
  // new RunCommand(() -> algaeIntake.stowWrist(), algaeIntake)
  // )
  // );
  // return routine;
  // return routine;
  // }
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
                        () -> RobotContainer.coralIntake.moveWristToPosition(-0.42),
                        RobotContainer.coralIntake),
                    new RunCommand(
                        () -> RobotContainer.elevator.moveElevatorToPosition(0.1),
                        RobotContainer.elevator)),
                new RunCommand(
                    () -> RobotContainer.coralIntake.intakeAtSpeed(-0.1),
                    RobotContainer.coralIntake)));
    return routine;
  }

  public AutoRoutine mAutoRoutine() {
    final AutoRoutine routine = m_factory.newRoutine("Maine Auton");
    final AutoTrajectory testTraj = routine.trajectory("MainAuton");

    routine.active().onTrue(testTraj.resetOdometry().andThen(testTraj.cmd()));
    return routine;
  }

  public AutoRoutine visionAutoRoutine() {
    final AutoRoutine routine = m_factory.newRoutine("Vision Auton");
    final AutoTrajectory StartToReef = routine.trajectory("VisionAuton");

    routine.active().onTrue(Commands.sequence(StartToReef.resetOdometry(), StartToReef.cmd()));
    return routine;
  }
}
