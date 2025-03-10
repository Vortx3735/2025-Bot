package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import edu.wpi.first.wpilibj.Timer;
public class AutoAlignCommand extends Command {
  private final CommandSwerveDrivetrain drivetrain;
  private final PhotonCamera intakeCamera;

  // PID constants for alignment
  private static PIDController rotationPidController;
  private static PIDController drivePidControllerX;
  private static PIDController drivePidControllerY;
  private static final double kP_Yaw = 0.03; // Proportional constant for yaw correction
  private static final double YAW_THRESHOLD = 0.5; // Degrees threshold for alignment
  private static final double X_THRESHOLD = 0.07; // Meters threshold for alignment
  private static final double Y_THRESHOLD = 0.01; // Meters threshold for alignment
  private static final double TARGET_DISTANCE_METERS = 0.42; // L2

  public double yawAdjustment;
  public double xAdjustment;
  public double yAdjustment;
  public double yaw;

  public double distanceX;
  public double distanceY;

  private Timer timer;

  public AutoAlignCommand(CommandSwerveDrivetrain drivetrain, PhotonCamera intakeCamera) {
    this.drivetrain = drivetrain;
    this.intakeCamera = intakeCamera;
    rotationPidController = new PIDController(kP_Yaw, 0.02, 0.0);
    drivePidControllerX = new PIDController(3, 0, 0);
    drivePidControllerY = new PIDController(4.5, 0, 0);

    rotationPidController.setTolerance(YAW_THRESHOLD);

    timer = new Timer();

    addRequirements(drivetrain);
  }

  @Override
  public void initialize() {
    timer.restart();
  }

  public boolean isXAligned() {
    return Math.abs(distanceX - TARGET_DISTANCE_METERS) < X_THRESHOLD;
  }

  public boolean isYAligned() {
    return Math.abs(distanceY - 0.03) < Y_THRESHOLD;
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
      yaw = (target.getBestCameraToTarget().getRotation().getZ()) * (180 / Math.PI);

      if (yaw < 0) {
        yaw += 180;
      } else {
        yaw -= 180;
      }

      distanceY = target.getBestCameraToTarget().getY();
      SmartDashboard.putNumber("vision/Distance", distanceX);
      SmartDashboard.putNumber("vision/Yaw", yaw);

      // Calculate adjustments for yaw and forward movement
      yawAdjustment = rotationPidController.calculate(yaw, 0);
      xAdjustment = drivePidControllerX.calculate(distanceX, TARGET_DISTANCE_METERS);
      yAdjustment = drivePidControllerY.calculate(distanceY, -0.03);

      SmartDashboard.putNumber("vision/xAdjustment", xAdjustment);
      SmartDashboard.putNumber("vision/yaw", yaw);
      SmartDashboard.putNumber("vision/yAdjustment", yAdjustment);
      SmartDashboard.putNumber("vision/rotationAdjustment", yawAdjustment);

      SmartDashboard.putBoolean("vision/isAligned", isAligned());
      SmartDashboard.putBoolean("vision/isYawAligned", isYawAligned());
      SmartDashboard.putBoolean("vision/isXAligned", isXAligned());
      SmartDashboard.putBoolean("vision/isYAligned", isYAligned());

      if (rotationPidController.atSetpoint()) {
        yawAdjustment = 0;
      }
      drivetrain.setControl(
        new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
            .withVelocityX(-xAdjustment)
            .withVelocityY(-yAdjustment) // No lateral movement for alignment
            .withRotationalRate(-yawAdjustment));

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
