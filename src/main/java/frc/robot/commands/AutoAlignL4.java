package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

public class AutoAlignL4 extends Command {
  private final CommandSwerveDrivetrain drivetrain;
  private final PhotonCamera intakeCamera;

  // PID constants for alignment
  private static PIDController rotationPidController;
  private static PIDController drivePidControllerX;
  private static PIDController drivePidControllerY;

  private static final double kP_Yaw = 0.4; // Proportional constant for yaw correction
  private static final double kP_Distance = 0.5; // Proportional constant for distance correction
  private static final double YAW_THRESHOLD = .01; // Degrees threshold for alignment
  private static final double X_THRESHOLD = 0.1; // Meters threshold for alignment
  private static final double Y_THRESHOLD = 0.1; // Meters threshold for alignment
  private static final double TARGET_DISTANCE_METERS = 0.203;  // L2

  public double yawAdjustment;
  public double xAdjustment;
  public double yAdjustment;

  public AutoAlignL4(CommandSwerveDrivetrain drivetrain, PhotonCamera intakeCamera) {
    this.drivetrain = drivetrain;
    this.intakeCamera = intakeCamera;
    rotationPidController = new PIDController(kP_Yaw, 0, 0);
    drivePidControllerX = new PIDController(2.6, 0, 0);
    drivePidControllerY = new PIDController(1.5, 0, 0);

    rotationPidController.setSetpoint(Math.PI);
    drivePidControllerX.setSetpoint(TARGET_DISTANCE_METERS);
    rotationPidController.setTolerance(0.0001);

    addRequirements(drivetrain);
  }

  public boolean isXAligned() {
    return Math.abs(xAdjustment) < X_THRESHOLD;
  }

  public boolean isYAligned() {
    return Math.abs(yAdjustment) < Y_THRESHOLD;
  }

  public boolean isYawAligned() {
    System.out.println(yawAdjustment);
    return Math.abs(yawAdjustment) < YAW_THRESHOLD;
  }

  public boolean isAligned() {
    return isXAligned() && isYAligned() && isYawAligned();
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
      yawAdjustment = rotationPidController.calculate(yaw);
      xAdjustment = drivePidControllerX.calculate(distanceX, TARGET_DISTANCE_METERS);
      yAdjustment = drivePidControllerY.calculate(distanceY, 0.02);

      SmartDashboard.putNumber("vision/xAdjustment", xAdjustment);
      SmartDashboard.putNumber("vision/yAdjustment", yAdjustment);
      SmartDashboard.putNumber("vision/rotationAdjustment", yawAdjustment);

      SmartDashboard.putBoolean("vision/isAligned", isAligned());
      SmartDashboard.putBoolean("vision/isYawAligned", isYawAligned());
      SmartDashboard.putBoolean("vision/isXAligned", isXAligned());
      SmartDashboard.putBoolean("vision/isYAligned", isYAligned());

      // if (rotationPidController.atSetpoint()) {
      //   yawAdjustment = 0;
      // }

      // Apply swerve drive request
      drivetrain.setControl(
          new SwerveRequest.FieldCentric()
              .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
              .withVelocityX(-xAdjustment)
              .withVelocityY(-yAdjustment) // No lateral movement for alignment
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
    return isAligned();
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
