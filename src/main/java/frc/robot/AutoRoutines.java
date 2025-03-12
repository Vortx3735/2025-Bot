package frc.robot;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.AutoAlignCommand;
import frc.robot.commands.CommandFactory;
import frc.robot.commands.NewAutoAlignCommand;

public class AutoRoutines {
  private final AutoFactory m_factory;

  public AutoRoutines(AutoFactory factory) {
    m_factory = factory;
  }

  public AutoRoutine oneL4() {
    final AutoRoutine routine = m_factory.newRoutine("Vision Auton");
    final AutoTrajectory StartToReef = routine.trajectory("TestReef");
    final AutoTrajectory reefToHP = routine.trajectory("ReefToHP");
    final AutoTrajectory hpToReef = routine.trajectory("HPToReef");

    NewAutoAlignCommand autoAlignCommand =
        new NewAutoAlignCommand(RobotContainer.drivetrain, RobotContainer.reefCamera, 0.42);
    // AutoAlignCommand autoAlignCommand =
    //     new AutoAlignCommand(RobotContainer.drivetrain, RobotContainer.reefCamera);

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
                    autoAlignCommand.withTimeout(2).asProxy().withName("Auto Align Command"),
                    CommandFactory.scoreL4Command().asProxy())

                // reefToHP.cmd().asProxy(),
                // RobotContainer.coralIntake.intakeCommand().asProxy(),
                // hpToReef.cmd().asProxy(),
                // autoAlignCommand.asProxy(),
                // CommandFactory.scoreL4Command().asProxy())
                .withName("One L4 Auton Sequence"));
    return routine;
  }

  public AutoRoutine twoL4() {
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
                    autoAlignCommand.withTimeout(2).asProxy().withName("Auto Align Command"),
                    CommandFactory.scoreL4Command().asProxy(),
                    reefToHP.cmd().asProxy(),
                    RobotContainer.coralIntake.intakeCommand().asProxy(),
                    hpToReef.cmd().asProxy(),
                    autoAlignCommand.asProxy(),
                    CommandFactory.scoreL4Command().asProxy())
                .withName("Two L4 Auton Sequence"));
    return routine;
  }
}
