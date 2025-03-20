package frc.robot;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.CommandFactory;
import frc.robot.subsystems.drive.Drive;

public class AutoRoutines {
  private final AutoFactory m_factory;
  private final Drive m_drive;

  public AutoRoutines(AutoFactory factory, Drive drive) {
    m_factory = factory;
    m_drive = drive;
  }

  //   public Command autoAlignL4() {
  //     return new AutoAlignCommand(m_drive, 0.38).withTimeout(5);
  //   }

  //   public Command autoAlignHP() {
  //     return new AutoAlignHpCommand(m_drive, 0.07).withTimeout(2);
  //   }

  public AutoRoutine oneL4Left() {
    final AutoRoutine routine = m_factory.newRoutine("One L4 Left Auton");
    final AutoTrajectory startToReef = routine.trajectory("LeftStart");

    routine
        .active()
        .onTrue(
            Commands.sequence(
                startToReef.resetOdometry().asProxy(),
                Commands.parallel(
                        Commands.sequence(
                                RobotContainer.coralWrist.moveWristToHP().asProxy(),
                                RobotContainer.coralIntake.intakeCommand().asProxy())
                            .withName("Move Wrist and Intake Coral"),
                        startToReef.cmd().asProxy())
                    .withName("Move and Intake Coral"),
                // autoAlignL4().asProxy(),
                CommandFactory.scoreL4Command()));
    return routine;
  }

  public AutoRoutine twoL4Left() {
    final AutoRoutine routine = m_factory.newRoutine("Two L4 Left Auton");
    final AutoTrajectory startToReef = routine.trajectory("LeftStart");
    final AutoTrajectory reefToHP = routine.trajectory("LeftReeftoHP");
    final AutoTrajectory hpToReef = routine.trajectory("LeftHPtoReef");

    routine
        .active()
        .onTrue(
            Commands.sequence(
                startToReef.resetOdometry().asProxy(),
                Commands.parallel(
                        Commands.sequence(
                                RobotContainer.coralWrist.moveWristToHP().asProxy(),
                                RobotContainer.coralIntake.intakeCommand().asProxy())
                            .withName("Move Wrist and Intake Coral"),
                        startToReef.cmd().asProxy())
                    .withName("Move and Intake Coral"),
                // autoAlignL4(),
                CommandFactory.scoreL4Command().asProxy(),
                reefToHP.cmd().asProxy(),
                Commands.parallel(
                    // autoAlignHP().asProxy(),
                    // RobotContainer.coralIntake.intakeCommand().asProxy()),
                    hpToReef.cmd().asProxy(),
                    // autoAlignL4(),
                    CommandFactory.scoreL4Command().asProxy())));
    return routine;
  }

  public AutoRoutine oneL4Right() {
    final AutoRoutine routine = m_factory.newRoutine("One L4 Right Auton");
    final AutoTrajectory startToReef = routine.trajectory("RightStart");

    routine
        .active()
        .onTrue(
            Commands.sequence(
                startToReef.resetOdometry().asProxy(),
                Commands.parallel(
                        Commands.sequence(
                                RobotContainer.coralWrist.moveWristToHP().asProxy(),
                                RobotContainer.coralIntake.intakeCommand().asProxy())
                            .withName("Move Wrist and Intake Coral"),
                        startToReef.cmd().asProxy())
                    .withName("Move and Intake Coral"),
                // autoAlignL4().asProxy(),
                CommandFactory.scoreL4Command()));
    return routine;
  }

  public AutoRoutine twoL4Right() {
    final AutoRoutine routine = m_factory.newRoutine("Two L4 Right Auton");
    final AutoTrajectory startToReef = routine.trajectory("RightStart");
    final AutoTrajectory reefToHP = routine.trajectory("RightReeftoHP");
    final AutoTrajectory hpToReef = routine.trajectory("RightHPtoReef");

    routine
        .active()
        .onTrue(
            Commands.sequence(
                startToReef.resetOdometry().asProxy(),
                Commands.parallel(
                        Commands.sequence(
                                RobotContainer.coralWrist.moveWristToHP().asProxy(),
                                RobotContainer.coralIntake.intakeCommand().asProxy())
                            .withName("Move Wrist and Intake Coral"),
                        startToReef.cmd().asProxy())
                    .withName("Move and Intake Coral"),
                // autoAlignL4().asProxy(),
                CommandFactory.scoreL4Command().asProxy(),
                reefToHP.cmd().asProxy(),
                // autoAlignHP().asProxy(),
                RobotContainer.coralIntake.intakeCommand().asProxy(),
                hpToReef.cmd().asProxy(),
                // autoAlignL4().asProxy(),
                CommandFactory.scoreL4Command().asProxy()));
    return routine;
  }

  public AutoRoutine oneL4Center() {
    final AutoRoutine routine = m_factory.newRoutine("One L4 Center Auton");
    final AutoTrajectory startToReef = routine.trajectory("CenterStart");

    routine
        .active()
        .onTrue(
            Commands.sequence(
                startToReef.resetOdometry().asProxy(),
                Commands.parallel(
                        Commands.sequence(
                                RobotContainer.coralWrist.moveWristToHP().asProxy(),
                                RobotContainer.coralIntake.intakeCommand().asProxy())
                            .withName("Move Wrist and Intake Coral"),
                        startToReef.cmd().asProxy())
                    .withName("Move and Intake Coral"),
                // autoAlignL4().asProxy(),
                CommandFactory.scoreL4Command()));
    return routine;
  }

  public AutoRoutine alignAndScore() {
    final AutoRoutine routine = m_factory.newRoutine("One L4 Center Auton");
    routine
        .active()
        .onTrue(
            Commands.sequence(
                RobotContainer.coralWrist.moveWristToHP().asProxy(),
                RobotContainer.coralIntake.intakeCommand().asProxy(),
                // autoAlignL4(),
                CommandFactory.scoreL4Command()));
    return routine;
  }
}
