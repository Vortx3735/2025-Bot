package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import edu.wpi.first.math.controller.PIDController;

public class AutoAlignCommand extends Command {
  private final CommandSwerveDrivetrain drivetrain;
  private final PhotonCamera intakeCamera;

  // PID constants for alignment
  private static PIDController rotationPidController;
  private static PIDController drivePidControllerX;
  private static PIDController drivePidControllerY;

  private static final double kP_Yaw = 0.07; // Proportional constant for yaw correction
  private static final double kP_Distance = 0.5; // Proportional constant for distance correction
  private static final double YAW_THRESHOLD = 1.0; // Degrees threshold for alignment
  private static final double DISTANCE_THRESHOLD = 0.1; // Meters threshold for alignment
  private static final double TARGET_DISTANCE_METERS = 0.36; // L2

  public AutoAlignCommand(CommandSwerveDrivetrain drivetrain, PhotonCamera intakeCamera) {
    this.drivetrain = drivetrain;
    this.intakeCamera = intakeCamera;
    rotationPidController = new PIDController(kP_Yaw, 0, 0);
    drivePidControllerX = new PIDController(1, 0, 0);
    drivePidControllerY = new PIDController(1.5, 0, 0);

    rotationPidController.setSetpoint(Math.PI);
    drivePidControllerX.setSetpoint(TARGET_DISTANCE_METERS);
    rotationPidController.setTolerance(0.005);


    addRequirements(drivetrain);
  }

  @Override
  public void execute() {
    // Get the latest result from the camera
    PhotonPipelineResult result = intakeCamera.getLatestResult();

    if (result.hasTargets()) {
      var target = result.getBestTarget();
      // double yaw = target.getYaw(); // Horizontal offset to the AprilTag
      double distanceX = target.getBestCameraToTarget().getX(); // Distance to the tag (forward)
      double yaw = target.getBestCameraToTarget().getRotation().getZ();
      double distanceY = target.getBestCameraToTarget().getY();
      SmartDashboard.putNumber("vision/Distance", distanceX);
      SmartDashboard.putNumber("vision/Yaw", yaw);

      // Calculate adjustments for yaw and forward movement
      double yawAdjustment = rotationPidController.calculate(yaw);
      double distanceAdjustment = drivePidControllerX.calculate(TARGET_DISTANCE_METERS - distanceX);
      double distanceYAdjustment = drivePidControllerY.calculate(distanceY+0.02);

      SmartDashboard.putNumber("vision/distanceAdjustment", distanceAdjustment);
      SmartDashboard.putNumber("vision/distanceYAdjustment", distanceYAdjustment);
      SmartDashboard.putNumber("vision/rotationAdjustment", yawAdjustment);

      if(rotationPidController.atSetpoint()){
        yawAdjustment = 0;
      }

      // Apply swerve drive request
      drivetrain.setControl(
          new SwerveRequest.FieldCentric()
              .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
              .withVelocityX(distanceAdjustment)
              .withVelocityY(-distanceYAdjustment) // No lateral movement for alignment
              .withRotationalRate(yawAdjustment));
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
