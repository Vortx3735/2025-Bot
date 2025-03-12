package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

public class AutoAlignCommand extends Command {
  private final CommandSwerveDrivetrain drivetrain;
  private final PhotonCamera intakeCamera;

  // PID constants for alignment
  private static PIDController rotationPidController;
  private static PIDController drivePidControllerX;
  private static PIDController drivePidControllerY;
  private static final double kP_Yaw = 12; // Proportional constant for yaw correction
  private static final double kP_X = 4.5;
  private static final double kP_Y = 5;

  private static final double YAW_THRESHOLD = 0.12; // Degrees threshold for alignment
  private static final double X_THRESHOLD = 0.01; // Meters threshold for alignment
  private static final double Y_THRESHOLD = 0.01; // Meters threshold for alignment

  private static final double TARGET_X = 0.42; // Target distance in meters
  private static final double TARGET_Y = -0.01; // Target distance in meters
  private static final double TARGET_YAW = -0.01; // Target rotation

  private double yawAdjustment;
  private double xAdjustment;
  private double yAdjustment;
  private double yaw;

  private double distanceX;
  private double distanceY;

  private Timer timer;

  public AutoAlignCommand(CommandSwerveDrivetrain drivetrain, PhotonCamera intakeCamera) {
    this.drivetrain = drivetrain;
    this.intakeCamera = intakeCamera;
    rotationPidController = new PIDController(kP_Yaw, 0, 0);
    drivePidControllerX = new PIDController(kP_X, 0, 0.1);
    drivePidControllerY = new PIDController(kP_Y, 0, 0.1);

    rotationPidController.setTolerance(YAW_THRESHOLD);

    timer = new Timer();

    addRequirements(drivetrain);
  }

  @Override
  public void initialize() {
    timer.restart();
  }

  public boolean isXAligned() {
    return Math.abs(distanceX - TARGET_X) < X_THRESHOLD;
  }

  public boolean isYAligned() {
    return Math.abs(distanceY - TARGET_Y) < Y_THRESHOLD;
  }

  public boolean isYawAligned() {
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
      distanceX = target.getBestCameraToTarget().getX(); // Distance to the tag (forward)
      distanceY = target.getBestCameraToTarget().getY();
      yaw = (target.getBestCameraToTarget().getRotation().getZ());

      if (yaw < 0) {
        yaw += Math.PI;
      } else {
        yaw -= Math.PI;
      }

      // Calculate adjustments for yaw and forward movement
      yawAdjustment = rotationPidController.calculate(yaw, TARGET_YAW);
      xAdjustment = drivePidControllerX.calculate(distanceX, TARGET_X);
      yAdjustment = drivePidControllerY.calculate(distanceY, TARGET_Y);

      if (xAdjustment > .75) {
        xAdjustment = .75;
      }
      if (xAdjustment < -.75) {
        xAdjustment = -.75;
      }
      if (yAdjustment > .75) {
        yAdjustment = .75;
      }
      if (yAdjustment < -.75) {
        yAdjustment = -.75;
      }
      if (yawAdjustment > .75) {
        yawAdjustment = .75;
      }
      if (yawAdjustment < -.75) {
        yawAdjustment = -.75;
      }
      if (rotationPidController.atSetpoint()) {
        yawAdjustment = 0;
      }
      if (rotationPidController.atSetpoint()) {
        yawAdjustment = 0;
      }
      if (!isYawAligned()) {
        drivetrain.setControl(
            new SwerveRequest.RobotCentric()
                .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
                .withVelocityX(0)
                .withVelocityY(0) // No lateral movement for alignment
                .withRotationalRate(-yawAdjustment));
      } else {
        drivetrain.setControl(
            new SwerveRequest.RobotCentric()
                .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
                .withVelocityX(-xAdjustment)
                .withVelocityY(-yAdjustment) // No lateral movement for alignment
                .withRotationalRate(-yawAdjustment));
      }

      SmartDashboard.putNumber("vision/DistanceX", distanceX);
      SmartDashboard.putNumber("vision/DistanceY", distanceY);
      SmartDashboard.putNumber("vision/Yaw", yaw);

      SmartDashboard.putNumber("vision/xAdjustment", xAdjustment);
      SmartDashboard.putNumber("vision/yAdjustment", yAdjustment);
      SmartDashboard.putNumber("vision/rotationAdjustment", yawAdjustment);

      SmartDashboard.putBoolean("vision/isXAligned", isXAligned());
      SmartDashboard.putBoolean("vision/isYAligned", isYAligned());
      SmartDashboard.putBoolean("vision/isYawAligned", isYawAligned());
      SmartDashboard.putBoolean("vision/isAligned", isAligned());

    } else {
      // Stop the robot if no targets are found
      end(true);
    }
  }

  @Override
  public boolean isFinished() {
    return isAligned() || timer.hasElapsed(1);
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
