package frc.robot.commands.autos;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.InchesPerSecond;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.subsystems.drive.Drive;

public class PositionPIDCommand extends Command {

  public Drive mSwerve;
  public final Pose2d goalPose;
  private PPHolonomicDriveController mDriveController =
      Constants.DriveTrainConstants.kDriveController;

  private final Trigger endTrigger;
  private final Trigger endTriggerDebounced;

  private final BooleanPublisher endTriggerLogger =
      NetworkTableInstance.getDefault()
          .getTable("logging")
          .getBooleanTopic("PositionPIDEndTrigger")
          .publish();

  private PositionPIDCommand(Drive mSwerve, Pose2d goalPose) {
    this.mSwerve = mSwerve;
    this.goalPose = goalPose;

    endTrigger =
        new Trigger(
            () -> {
              Pose2d diff = mSwerve.getPose().relativeTo(goalPose);

              boolean rotation =
                  MathUtil.isNear(
                      0.0,
                      diff.getRotation().getDegrees(),
                      Rotation2d.fromDegrees(2.0).getDegrees(),
                      0.0,
                      1.0);

              boolean position = diff.getTranslation().getNorm() < Inches.of(0.4).in(Meters);

              boolean speed =
                  mSwerve.getChassisSpeeds().vxMetersPerSecond
                      < InchesPerSecond.of(0.25).in(MetersPerSecond);

              System.out.println(
                  "end trigger conditions R: " + rotation + "\tP: " + position + "\tS: " + speed);

              return rotation && position && speed;
            });

    endTriggerDebounced =
        new Trigger(endTrigger::getAsBoolean).debounce(Seconds.of(0.04).in(Seconds));
  }

  public static Command generateCommand(Drive swerve, Pose2d goalPose, Time timeout) {
    return new PositionPIDCommand(swerve, goalPose)
        .withTimeout(timeout)
        .finallyDo(
            () -> {
              swerve.runVelocity(new ChassisSpeeds(0, 0, 0));
              swerve.stopWithX();
            });
  }

  @Override
  public void initialize() {
    endTriggerLogger.accept(endTrigger.getAsBoolean());
  }

  @Override
  public void execute() {
    PathPlannerTrajectoryState goalState = new PathPlannerTrajectoryState();
    goalState.pose = goalPose;

    endTriggerLogger.accept(endTrigger.getAsBoolean());

    mSwerve.runVelocity(
        mDriveController.calculateRobotRelativeSpeeds(mSwerve.getPose(), goalState));
  }

  @Override
  public void end(boolean interrupted) {
    endTriggerLogger.accept(endTrigger.getAsBoolean());
  }

  @Override
  public boolean isFinished() {
    return endTriggerDebounced.getAsBoolean();
  }
}
