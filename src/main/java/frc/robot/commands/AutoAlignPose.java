package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.IdealStartingState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.Vision;
import java.util.Arrays;
import java.util.List;
import java.util.Optional;

public class AutoAlignPose extends Command {
  private final Drive drivetrain;
  private final Vision vision;

  // Target distance from tag
  private static final double TAG_OFFSET_METERS = 0.38;

  // Path constraints
  private static final double MAX_VELOCITY = 3.0 * 0.3;
  private static final double MAX_ACCELERATION = 2.0;
  private static final double MAX_ANGULAR_VELOCITY = 2 * Math.PI;
  private static final double MAX_ANGULAR_ACCELERATION = 4 * Math.PI;

  // Define reef AprilTag ranges
  private static final List<Integer> REEF_TAG_IDS =
      Arrays.asList(6, 7, 8, 9, 10, 11, 17, 18, 19, 20, 21, 22);

  private final AprilTagFieldLayout fieldLayout;
  private Command pathCommand = null;
  private Pose2d targetPose = null;
  private Pose2d beginningRobotPose;
  private int currentTargetId = -1;

  // PID constants for fine adjustment
  private static final double kP_X = 3;
  private static final double kP_Y = 3;
  private static final double kP_Rotation = 2;

  // Position tolerance
  private static final double POSITION_TOLERANCE = 0.02;
  private static final double ROTATION_TOLERANCE = 0.05;

  public AutoAlignPose(Drive drivetrain, Vision vision) {
    this.drivetrain = drivetrain;
    this.vision = vision;

    // Load the field layout
    fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);

    addRequirements(drivetrain, vision);
  }

  @Override
  public void initialize() {
    // Find closest reef tag
    updateTargetFromVision();
    beginningRobotPose = drivetrain.getPose();

    // Create path command if target found
    if (targetPose != null) {
      createPathCommand();
    }
  }

  private void updateTargetFromVision() {
    // Get current robot pose
    Pose2d currentRobotPose = drivetrain.getPose();
    double bestDistance = Double.MAX_VALUE;
    int bestTagId = 18;

    // Get all detected tags
    int[] detectedTags = vision.getDetectedTags(0);

    for (int tagId : detectedTags) {
      // Only consider reef tags
      if (REEF_TAG_IDS.contains(tagId)) {
        // Get the tag's position from field layout
        Optional<Pose3d> tagPose = fieldLayout.getTagPose(tagId);
        if (tagPose.isPresent()) {
          // Calculate distance from robot to this tag
          double distance =
              currentRobotPose
                  .getTranslation()
                  .getDistance(new Translation2d(tagPose.get().getX(), tagPose.get().getY()));
          // Choose the closest tag
          if (distance < bestDistance) {
            bestDistance = distance;
            bestTagId = tagId;
          }
        }
      }
    }

    // If we found a tag, calculate target pose
    if (bestTagId != -1) {
      calculateTargetPose(bestTagId);
      currentTargetId = bestTagId;
    }
  }

  private void calculateTargetPose(int tagId) {
    try {
      // Get the pose of the April Tag in field coordinates
      Optional<Pose3d> tagPoseOptional = fieldLayout.getTagPose(tagId);

      if (!tagPoseOptional.isPresent()) {
        System.err.println("Error: Tag ID " + 18 + " not found in field layout");
        targetPose = null;
        return;
      }

      // Get the tag pose
      Pose3d tagPose3d = tagPoseOptional.get();
      Pose2d tagPose = tagPose3d.toPose2d();

      // Print tag information for debugging
      System.out.println("Tag ID: " + 18 + " at pose: " + tagPose);

      // Calculate approach distance (how far from the tag to stop)
      double approachDistance = 1.0; // 1 meter from tag

      // Calculate target position - in FRONT of the tag
      Translation2d targetTranslation =
          tagPose
              .getTranslation()
              .plus(
                  new Translation2d(
                      approachDistance * Math.cos(tagPose.getRotation().getRadians() + Math.PI),
                      approachDistance * Math.sin(tagPose.getRotation().getRadians() + Math.PI)));

      // Calculate target rotation - facing the tag
      Rotation2d targetRotation = tagPose.getRotation().plus(new Rotation2d(Math.PI));

      // Create the target pose
      targetPose = new Pose2d(targetTranslation, targetRotation);

      // Print final target for debugging
      System.out.println("Calculated target pose: " + targetPose);
      System.out.println("Robot will stop " + approachDistance + "m in front of tag " + tagId);

      SmartDashboard.putNumber("Pathplanner/TargetPoseX", targetPose.getX());
      SmartDashboard.putNumber("Pathplanner/TargetPoseY", targetPose.getY());
      SmartDashboard.putNumber("Pathplanner/TargetPoseRot", targetPose.getRotation().getDegrees());
      SmartDashboard.putNumber(
          "Pathplanner/AprilTagPoseX", fieldLayout.getTagPose(18).get().getX());
      SmartDashboard.putNumber(
          "Pathplanner/AprilTagPoseY", fieldLayout.getTagPose(18).get().getY());
      SmartDashboard.putNumber(
          "Pathplanner/AprilTagPoseRot", fieldLayout.getTagPose(18).get().getRotation().getAngle());
    } catch (Exception e) {
      System.err.println("Error calculating target pose: " + e.getMessage());
      e.printStackTrace();
      targetPose = null;
    }
  }

  private void createPathCommand() {
    try {
      // Get current robot pose and velocity
      Pose2d currentPose = drivetrain.getPose();
      ChassisSpeeds robotVelocity = drivetrain.getChassisSpeeds();

      // Calculate velocity magnitude
      double velocityMagnitude =
          Math.hypot(robotVelocity.vxMetersPerSecond, robotVelocity.vyMetersPerSecond);

      // Determine heading for path based on velocity or direction to target
      Rotation2d pathHeading;
      if (velocityMagnitude > 0.25) {
        // Use robot's current velocity direction if moving
        pathHeading =
            new Rotation2d(robotVelocity.vxMetersPerSecond, robotVelocity.vyMetersPerSecond);
      } else {
        // Otherwise use direction to target
        Translation2d diff = targetPose.getTranslation().minus(currentPose.getTranslation());
        pathHeading = diff.getNorm() < 0.01 ? targetPose.getRotation() : diff.getAngle();
      }

      // Create path constraints
      PathConstraints constraints =
          new PathConstraints(
              MAX_VELOCITY, MAX_ACCELERATION,
              MAX_ANGULAR_VELOCITY, MAX_ANGULAR_ACCELERATION);

      // Generate waypoints
      List<Waypoint> waypoints =
          PathPlannerPath.waypointsFromPoses(
              // Start with current pose but use calculated path heading
              new Pose2d(currentPose.getTranslation(), pathHeading),
              // End at target pose but with path heading pointing towards it
              new Pose2d(targetPose.getTranslation(), pathHeading));

      // Check if we're already close to target
      if (waypoints.get(0).anchor().getDistance(waypoints.get(1).anchor()) < 0.05) {
        // Skip path following and use direct PID control
        System.out.println("Already close to target, using direct PID control");
        // pathCommand = createPIDPositionCommand();
      } else {
        // Create the path
        PathPlannerPath path =
            new PathPlannerPath(
                waypoints,
                constraints,
                new IdealStartingState(
                    drivetrain.getChassisSpeeds().vxMetersPerSecond,
                    drivetrain.getPose().getRotation()),
                new GoalEndState(0.0, targetPose.getRotation()) // End with target rotation
                );

        // Prevent the path from being flipped
        path.preventFlipping = true;

        // Create the path following command with PID fine adjustment at the end
        pathCommand =
            AutoBuilder.followPath(path)
                .andThen(Commands.print("Starting position PID loop"))
                // .andThen(createPIDPositionCommand())
                .andThen(Commands.print("Finished position PID loop"));
      }
    } catch (Exception e) {
      // Log any errors
      DriverStation.reportError("Error creating path: " + e.getMessage(), e.getStackTrace());
      pathCommand = null;
    }
  }

  // // Create a PID-based position command for final alignment
  // private Command createPIDPositionCommand() {
  //   // return Commands.run(
  //   //         () -> {
  //   //           // Get current pose
  //   //           Pose2d currentPose = drivetrain.getPose();

  //   //           // Calculate errors relative to target
  //   //           double xError = targetPose.getX() - currentPose.getX();
  //   //           double yError = targetPose.getY() - currentPose.getY();
  //   //           double rotError =
  //   //               targetPose.getRotation().minus(currentPose.getRotation()).getRadians();

  //   //           // Simple PID control
  //   //           double xSpeed = kP_X * xError;
  //   //           double ySpeed = kP_Y * yError;
  //   //           double rotSpeed = kP_Rotation * rotError;

  //   //           // Clamp outputs
  //   //           xSpeed = MathUtil.clamp(xSpeed, -1.0, 1.0);
  //   //           ySpeed = MathUtil.clamp(ySpeed, -1.0, 1.0);
  //   //           rotSpeed = MathUtil.clamp(rotSpeed, -2.0, 2.0);

  //   //           // Drive using field-relative speeds
  //   //           drivetrain.runVelocity(
  //   //               ChassisSpeeds.fromFieldRelativeSpeeds(
  //   //                   xSpeed, ySpeed, rotSpeed, currentPose.getRotation()));

  //   //           // Debug info
  //   //           SmartDashboard.putNumber("PID/XError", xError);
  //   //           SmartDashboard.putNumber("PID/YError", yError);
  //   //           SmartDashboard.putNumber("PID/RotError", rotError);
  //   //         },
  //   //         drivetrain)
  //   //     .until(
  //   //         () -> {
  //   //           // Check if we've reached target position
  //   //           Pose2d currentPose = drivetrain.getPose();
  //   //           double posError =
  //   //               currentPose.getTranslation().getDistance(targetPose.getTranslation());
  //   //           double rotError =
  //   //
  // Math.abs(currentPose.getRotation().minus(targetPose.getRotation()).getRadians());

  //   //           return posError < POSITION_TOLERANCE && rotError < ROTATION_TOLERANCE;
  //   //         });
  // }

  @Override
  public void execute() {
    if (pathCommand == null && targetPose == null) {
      // Try to find a target if we don't have one
      updateTargetFromVision();
      if (targetPose != null) {
        createPathCommand();
        if (pathCommand != null) {
          pathCommand.initialize();
        }
      }
    } else if (pathCommand == null && targetPose != null) {
      // We have a target but no path - try creating it again
      createPathCommand();
      if (pathCommand != null) {
        pathCommand.initialize();
      }
    } else if (pathCommand != null) {
      // Execute the path command
      pathCommand.schedule();
    }
  }

  @Override
  public boolean isFinished() {
    if (pathCommand == null) {
      return targetPose == null; // Finish if we couldn't find a target
    }

    return pathCommand.isFinished();
  }

  @Override
  public void end(boolean interrupted) {
    if (pathCommand != null) {
      pathCommand.end(interrupted);
    }
  }
}
