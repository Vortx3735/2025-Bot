package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import java.util.Set;
import java.util.function.BooleanSupplier;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;

public class AutoAlignL3 extends Command {
  private final CommandSwerveDrivetrain drivetrain;
  private final PhotonCamera camera;

  private static boolean isAligned = false;

  private static final double TARGET_DISTANCE_METERS = 0.1143;
  private static final double kP_Yaw = 0.05;
  private static final double kP_Drive = 0.2; // tune this kp so robot dont tip when elevator up

  public AutoAlignL3(CommandSwerveDrivetrain drivetrain, PhotonCamera camera) {
    this.drivetrain = drivetrain;
    this.camera = camera;
    addRequirements(drivetrain);
  }

  public BooleanSupplier isAligned() {
    return () -> isAligned;
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    var results = camera.getAllUnreadResults();
    boolean targetVisible = false;
    double targetYaw = 0.0;
    double targetRange = 0.0;

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
            targetRange =
                PhotonUtils.calculateDistanceToTargetMeters(
                    0.5, // Measured with a tape measure, or in CAD.
                    0.2225, // From 2025 game manual
                    Units.degreesToRadians(-30.0), // Measured with a protractor, or in CAD.
                    Units.degreesToRadians(target.getPitch()));
            targetVisible = true;

            distanceError =
                (TARGET_DISTANCE_METERS - targetRange) * kP_Drive; // u might need to increase kp
            yawError = kP_Yaw * targetYaw;

            drivetrain.setControl(
                new SwerveRequest.FieldCentric()
                    .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
                    .withVelocityX(0.3)
                    .withVelocityY(0)
                    .withRotationalRate(-yawError));
          }
        }

        // use this to end the command
        if (distanceError < 0.05) {
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
