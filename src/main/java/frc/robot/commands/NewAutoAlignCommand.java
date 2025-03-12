package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

public class NewAutoAlignCommand extends Command {
  private final CommandSwerveDrivetrain drivetrain;
  private final PhotonCamera intakeCamera;

  // PID constants for alignment
  private PIDController yawPID;
  private PIDController xPID;
  private PIDController yPID;

  private static final double kP_Yaw = 2.5; // Proportional constant for yaw correction
  private static final double kP_X = 2;
  private static final double kP_Y = 6;

  
  private static final double YAW_THRESHOLD = 0.12; // Degrees threshold for alignment
  private static final double X_THRESHOLD = 0.03; // Meters threshold for alignment
  private static final double Y_THRESHOLD = 0.01; // Meters threshold for alignment

  private static double TARGET_X = 0.42; // Target distance in meters
  private static final double TARGET_Y = -0.01; // Target distance in meters
  private static final double TARGET_YAW = -0.01; // Target rotation

  private double yawAdjustment;
  private double xAdjustment;
  private double yAdjustment;

  private double yaw;
  private double distanceX;
  private double distanceY;

  public NewAutoAlignCommand(
      CommandSwerveDrivetrain drivetrain, PhotonCamera intakeCamera, double targetXDistance) {
    TARGET_X = targetXDistance;

    this.drivetrain = drivetrain;
    this.intakeCamera = intakeCamera;
    yawPID = new PIDController(kP_Yaw, 0, 0);
    xPID = new PIDController(kP_X, 0, 0.25);
    yPID = new PIDController(kP_Y, 0, 0.25);

    yawPID.setTolerance(YAW_THRESHOLD);
    xPID.setTolerance(X_THRESHOLD);
    yPID.setTolerance(Y_THRESHOLD);

    yawPID.setSetpoint(TARGET_YAW);
    xPID.setSetpoint(TARGET_X);
    yPID.setSetpoint(TARGET_Y);

    addRequirements(drivetrain);
  }

  public boolean isAligned() {
    return xPID.atSetpoint() && yPID.atSetpoint() && yawPID.atSetpoint();
  }

  @Override
  public void execute() {
    // Get the latest result from the camera
    PhotonPipelineResult result = intakeCamera.getLatestResult();

    if (result.hasTargets()) {
      var target = result.getBestTarget();

      distanceX = target.getBestCameraToTarget().getX(); // Distance to the tag (forward)
      distanceY = target.getBestCameraToTarget().getY();
      yaw = (target.getBestCameraToTarget().getRotation().getZ());

      if (yaw < 0) {
        yaw += Math.PI;
      } else {
        yaw -= Math.PI;
      }

      // Calculate adjustments for yaw and forward movement
      yawAdjustment = yawPID.calculate(yaw, TARGET_YAW);
      xAdjustment = xPID.calculate(distanceX, TARGET_X);
      yAdjustment = yPID.calculate(distanceY, TARGET_Y);

      xAdjustment = MathUtil.clamp(xAdjustment, -0.75, 0.75);
      yAdjustment = MathUtil.clamp(yAdjustment, -0.75, 0.75);
      yawAdjustment = MathUtil.clamp(yawAdjustment, -0.75, 0.75);

      if (!yawPID.atSetpoint()) {
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
                .withVelocityY(-yAdjustment)
                .withRotationalRate(-yawAdjustment));
      }

      SmartDashboard.putNumber("vision/DistanceX", distanceX);
      SmartDashboard.putNumber("vision/DistanceY", distanceY);
      SmartDashboard.putNumber("vision/Yaw", yaw);

      SmartDashboard.putNumber("vision/xAdjustment", xAdjustment);
      SmartDashboard.putNumber("vision/yAdjustment", yAdjustment);
      SmartDashboard.putNumber("vision/rotationAdjustment", yawAdjustment);

      SmartDashboard.putBoolean("vision/isXAligned", xPID.atSetpoint());
      SmartDashboard.putBoolean("vision/isYAligned", yPID.atSetpoint());
      SmartDashboard.putBoolean("vision/isYawAligned", yawPID.atSetpoint());
      SmartDashboard.putBoolean("vision/isAligned", isAligned());
    } else {
      // Stop the robot if no targets are found
      end(true);
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
