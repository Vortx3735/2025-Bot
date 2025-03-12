package frc.robot.commands;

import choreo.trajectory.*;
import choreo.util.*;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import java.util.Optional;

public class FollowTrajectoryCommand extends Command {
  private final Optional<Trajectory<SwerveSample>> trajectory;
  public final CommandSwerveDrivetrain drivetrain = RobotContainer.drivetrain;
  private final Timer timer = new Timer();

  public FollowTrajectoryCommand(Optional<Trajectory<SwerveSample>> trajectory) {
    this.trajectory = trajectory;
    addRequirements(drivetrain);
  }

  private boolean isRedAlliance() {
    return DriverStation.getAlliance().orElse(Alliance.Blue).equals(Alliance.Red);
  }

  @Override
  public void initialize() {
    // Reset odometry to the starting pose of the trajectory
    // drivetrain.resetPose(trajectory.getInitialPose());
    if (trajectory.isPresent()) {
      // Get the initial pose of the trajectory
      Optional<Pose2d> initialPose = trajectory.get().getInitialPose(isRedAlliance());

      if (initialPose.isPresent()) {
        // Reset odometry to the start of the trajectory
        drivetrain.resetPose(initialPose.get());
      }
    }

    // Reset and start the timer when the autonomous period begins
    timer.restart();
  }

  @Override
  public void execute() {
    // Follow the trajectory
    if (trajectory.isPresent()) {
      // Sample the trajectory at the current time into the autonomous period
      Optional<SwerveSample> sample = trajectory.get().sampleAt(timer.get(), isRedAlliance());

      if (sample.isPresent()) {
        drivetrain.followPath(sample.get());
      }
    }
  }
}
