package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

public class AutoAlignCommand extends Command {
  private final CommandSwerveDrivetrain drivetrain;
  private final PhotonCamera intake;

  // PID constants for alignment
  private static final double kP_Yaw = 0.2; // Proportional constant for yaw correction
  private static final double kP_Distance = 0.2; // Proportional constant for distance correction
  private static final double YAW_THRESHOLD = 1.0; // Degrees threshold for alignment
  private static final double DISTANCE_THRESHOLD = 0.1; // Meters threshold for alignment

  public AutoAlignCommand(CommandSwerveDrivetrain drivetrain, PhotonCamera intake) {
    this.drivetrain = drivetrain;
    this.intake = intake;
    addRequirements(drivetrain);
  }

  @Override
  public void execute() {
    // Get the latest result from the camera
    PhotonPipelineResult result = intake.getLatestResult();

    if (result.hasTargets()) {
      System.out.println("yay");
      var target = result.getBestTarget();
      double yaw = target.getYaw(); // Horizontal offset to the AprilTag
      double distance = target.getBestCameraToTarget().getY(); // Distance to the tag (forward)
      SmartDashboard.putNumber("vision/Distance", distance);
      SmartDashboard.putNumber("vision/Yaw", yaw);

      // Calculate adjustments for yaw and forward movement
      double yawAdjustment = kP_Yaw * yaw;
      double distanceAdjustment = kP_Distance * distance;

      // Apply swerve drive request
      drivetrain.setControl(
          new SwerveRequest.FieldCentric()
              .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
              .withVelocityX(distanceAdjustment)
              .withVelocityY(0) // No lateral movement for alignment
              .withRotationalRate(-yawAdjustment));
    } else {
      // Stop the robot if no targets are found
      drivetrain.setControl(
          new SwerveRequest.FieldCentric()
              .withVelocityX(0.0)
              .withVelocityY(0.0)
              .withRotationalRate(0.0));
    }
  }

  @Override
  public boolean isFinished() {
    // PhotonPipelineResult result = intake.getLatestResult();

    // if (result.hasTargets()) {
    //     var target = result.getBestTarget();
    //     double yaw = target.getYaw();
    //     double distance = target.getBestCameraToTarget().getTranslation().getZ();

    //     // Finish command when robot is aligned
    //     return Math.abs(yaw) < YAW_THRESHOLD && Math.abs(distance) < DISTANCE_THRESHOLD;
    // }
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    // Stop the robot
    drivetrain.setControl(
        new SwerveRequest.FieldCentric()
            .withVelocityX(0.0)
            .withVelocityY(0.0)
            .withRotationalRate(0.0));
  }
}
