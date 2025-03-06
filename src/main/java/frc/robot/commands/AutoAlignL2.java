package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import java.util.Optional;
import java.util.Set;
import java.util.function.BooleanSupplier;
import org.photonvision.PhotonCamera;

public class AutoAlignL2 extends Command {
  private final CommandSwerveDrivetrain drivetrain;
  private final PhotonCamera camera;
  private final AprilTagFieldLayout fieldLayout;

  private static boolean isAligned = false;

  private static final double TARGET_DISTANCE_METERS = 0.1651;
  private static final double kP_Yaw = 0.2;
  private static final double kP_Drive = 0.3; // tune this kp so robot dont tip when elevator up
  private static final double kP_Strafe = 0.3;
  private static final double ALIGNMENT_TOLERANCE = 0.05; // meters

  public AutoAlignL2(CommandSwerveDrivetrain drivetrain, PhotonCamera camera) {
    this.drivetrain = drivetrain;
    this.camera = camera;
    this.fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
  }

  public BooleanSupplier isAligned() {
    return () -> isAligned;
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    // update pose with multitag *DONT FORGET TO ENABLE MULTITAG IN PHOTONVISION
    drivetrain.updatePoseWithVision(camera);

    var results = camera.getAllUnreadResults();
    boolean targetVisible = false;
    double targetYaw = 0.0;
    double distanceError = 0.0;
    double yawError = 0.0;

    if (!results.isEmpty()) {
      var result = results.get(results.size() - 1);
      if (result.hasTargets()) {
        // At least one AprilTag was seen by the camera
        for (var target : result.getTargets()) {
          // check if it is a reef apriltag
          Set<Integer> reefTagIds = Set.of(6, 7, 8, 9, 10, 11, 17, 18, 19, 20, 21, 22);
          if (reefTagIds.contains(target.getFiducialId())) {
            // Found reef april tag
            targetYaw = target.getYaw();
            yawError = kP_Yaw * targetYaw;

            // get apriltag pose
            Optional<Pose3d> tagPoseOptional = fieldLayout.getTagPose(target.getFiducialId());

            // Convert to 2D pose
            Pose2d tagPose = tagPoseOptional.get().toPose2d();

            // Create target pose in front of tag
            // The transform puts us TARGET_DISTANCE_METERS in front of the tag
            Transform2d frontOfTag =
                new Transform2d(
                    new Translation2d(TARGET_DISTANCE_METERS, 0),
                    new Rotation2d(Math.PI)); // Face the tag

            Pose2d targetPose = tagPose.plus(frontOfTag);

            // Get current robot pose
            Pose2d currentPose = drivetrain.getState().Pose;

            // Calculate error
            double xError = targetPose.getX() - currentPose.getX();
            double yError = targetPose.getY() - currentPose.getY();

            // Apply PID control
            double xVelocity = xError * kP_Drive;
            double yVelocity = yError * kP_Strafe;

            drivetrain.setControl(
                new SwerveRequest.FieldCentric()
                    .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
                    .withVelocityX(xVelocity)
                    .withVelocityY(yVelocity)
                    .withRotationalRate(-yawError));
          }
        }

        // use this to end the command
        if (distanceError < ALIGNMENT_TOLERANCE) {
          isAligned = true;
        }
      }
    } else {
      drivetrain.setControl(
          new SwerveRequest.FieldCentric()
              .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
              .withVelocityX(0)
              .withVelocityY(0)
              .withRotationalRate(0));
    }

    SmartDashboard.putBoolean("photonvision/targetVisible", targetVisible);
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    drivetrain.setControl(
        new SwerveRequest.FieldCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
            .withVelocityX(0)
            .withVelocityY(0)
            .withRotationalRate(0));
  }
}
