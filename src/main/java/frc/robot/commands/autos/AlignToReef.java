package frc.robot.commands.autos;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.IdealStartingState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.AutoAlignCommand;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.AprilTagRegion;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Set;

public class AlignToReef {
  private final Drive mSwerve;
  public static ArrayList<Pose2d> blueReefTagPoses = new ArrayList<>();
  public static ArrayList<Pose2d> redReefTagPoses = new ArrayList<>();
  public static ArrayList<Pose2d> allReefTagPoses = new ArrayList<>();
  public static final PathConstraints kPathConstraints =
      new PathConstraints(6, 6, 1 / 2 * Math.PI, 1 * Math.PI); // The constraints for this path.
  public static final Time kAlignmentAdjustmentTimeout = Seconds.of(3);

  public AlignToReef(Drive mSwerve, AprilTagFieldLayout field) {
    this.mSwerve = mSwerve;
    Arrays.stream(AprilTagRegion.kReef.blue())
        .forEach(
            (i) -> {
              field
                  .getTagPose(i)
                  .ifPresent(
                      (p) -> {
                        blueReefTagPoses.add(
                            new Pose2d(
                                p.getMeasureX(), p.getMeasureY(), p.getRotation().toRotation2d()));
                      });
            });

    Arrays.stream(AprilTagRegion.kReef.red())
        .forEach(
            (i) -> {
              field
                  .getTagPose(i)
                  .ifPresent(
                      (p) -> {
                        redReefTagPoses.add(
                            new Pose2d(
                                p.getMeasureX(), p.getMeasureY(), p.getRotation().toRotation2d()));
                      });
            });

    Arrays.stream(AprilTagRegion.kReef.both())
        .forEach(
            (i) -> {
              field
                  .getTagPose(i)
                  .ifPresent(
                      (p) -> {
                        allReefTagPoses.add(
                            new Pose2d(
                                p.getMeasureX(), p.getMeasureY(), p.getRotation().toRotation2d()));
                      });
            });
  }

  public Command generateCommand() {
    return Commands.defer(
        () -> {
          Pose2d waypoint = getWaypointFromTag(getClosestReefAprilTag(mSwerve.getPose()));
          return getPathFromWaypoint(waypoint, getClosestReefAprilTag(mSwerve.getPose()));
        },
        Set.of());
  }

  private Command getPathFromWaypoint(Pose2d waypoint, Pose2d tagPose) {
    Translation2d offsetVector =
        new Translation2d(
            0.7 * Math.cos(waypoint.getRotation().getRadians() + Math.PI),
            0.7 * Math.sin(waypoint.getRotation().getRadians() + Math.PI));

    // Create the actual target pose with the offset
    Pose2d targetPose =
        new Pose2d(
            waypoint.getTranslation().plus(offsetVector),
            waypoint.getRotation() // Keep the same rotation to face the reef
            );
    List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(mSwerve.getPose(), targetPose);

    if (waypoints.get(0).anchor().getDistance(waypoints.get(1).anchor()) < 0.2) {
      return Commands.sequence(
          Commands.print("start position PID loop"),
          PositionPIDCommand.generateCommand(mSwerve, waypoint, kAlignmentAdjustmentTimeout),
          Commands.print("end position PID loop"));
    }

    PathPlannerPath path =
        new PathPlannerPath(
            waypoints,
            kPathConstraints,
            new IdealStartingState(
                getVelocityMagnitude(mSwerve.getChassisSpeeds()), mSwerve.getRotation()),
            new GoalEndState(0.0, waypoint.getRotation()));

    path.preventFlipping = true;

    return AutoBuilder.followPath(path)
        .andThen(
            Commands.print("start position PID loop"),
            AutoAlignCommand.generateCommand(mSwerve, targetPose, tagPose),
            Commands.print("end position PID loop"));
  }

  /**
   * @param cs field relative chassis speeds
   * @return
   */
  private Rotation2d getPathVelocityHeading(ChassisSpeeds cs, Pose2d target) {
    if (getVelocityMagnitude(cs).in(MetersPerSecond) < 0.25) {
      var diff = target.minus(mSwerve.getPose()).getTranslation();
      return (diff.getNorm() < 0.01)
          ? target.getRotation()
          : diff.getAngle(); // .rotateBy(Rotation2d.k180deg);
    }
    return new Rotation2d(cs.vxMetersPerSecond, cs.vyMetersPerSecond);
  }

  private LinearVelocity getVelocityMagnitude(ChassisSpeeds cs) {
    return MetersPerSecond.of(
        new Translation2d(cs.vxMetersPerSecond, cs.vyMetersPerSecond).getNorm());
  }

  /**
   * @return Pathplanner waypoint with direction of travel away from the associated reef side
   */
  private Pose2d getWaypointFromTag(Pose2d tag) {
    System.out.println("tag rotation" + tag.getRotation().getDegrees());
    return new Pose2d(tag.getTranslation(), tag.getRotation().plus(new Rotation2d(Math.PI)));
  }

  /**
   * get closest reef april tag pose to given position
   *
   * @param pose field relative position
   * @return
   */
  public static Pose2d getClosestReefAprilTag(Pose2d pose) {
    var alliance = DriverStation.getAlliance();

    ArrayList<Pose2d> reefPoseList;
    if (alliance.isEmpty()) {
      reefPoseList = allReefTagPoses;
    } else {
      reefPoseList = alliance.get() == Alliance.Blue ? blueReefTagPoses : redReefTagPoses;
    }

    return pose.nearest(reefPoseList);
  }
}
