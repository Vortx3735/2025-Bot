package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

public class AutoAlignHpCommand extends Command {
  private final CommandSwerveDrivetrain drivetrain;
  private final PhotonCamera intakeCamera;
  private static final double TARGET_TIMEOUT = 0.15;

  // PID constants for alignment
  private PIDController yawPID;
  private PIDController xPID;
  private PIDController yPID;

  private double lastTargetTime = 0;
  private double lastDistanceX = 0;
  private double lastDistanceY = 0;
  private double lastYaw = 0;

  private static final double kP_Yaw = 3.5; // Proportional constant for yaw correction
  private static final double kP_X = 7
  ;
  private static final double kP_Y = 2.5;

  // private static final double BIG_YAW_THRESHOLD = 0.12; // Degrees threshold for alignment
  private static final double YAW_THRESHOLD = 0.05; // Degrees threshold for alignment
  private static final double X_THRESHOLD = 0.03; // Meters threshold for alignment
  private static final double Y_THRESHOLD = 0.01; // Meters threshold for alignment

  private static double TARGET_X = 0.68;
  ; // Target distance in meters
  private static double TARGET_Y;// Target distance in meters
  private static final double TARGET_YAW = -0.0349; // Target rotation

  private double yawAdjustment;
  private double xAdjustment;
  private double yAdjustment;

  private double yaw;
  private double distanceX;
  private double distanceY;

  public AutoAlignHpCommand(CommandSwerveDrivetrain drivetrain, PhotonCamera intakeCamera, double targetY) {
    this.drivetrain = drivetrain;
    this.intakeCamera = intakeCamera;
    yawPID = new PIDController(kP_Yaw, 0, .2);
    xPID = new PIDController(kP_X, 0, 0);
    yPID = new PIDController(kP_Y, 0, 0);

    TARGET_Y = targetY;

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

      xAdjustment = MathUtil.clamp(xAdjustment, -1, 1);
      yAdjustment = MathUtil.clamp(yAdjustment, -0.75, 0.75);
      yawAdjustment = MathUtil.clamp(yawAdjustment, -2.5, 2.5);


      // if(xPID.atSetpoint()){
      //   xAdjustment = 0;
      // }

      // if(yPID.atSetpoint()){
      //   yAdjustment = 0;
      // }

      // if(yawPID.atSetpoint()){
      //   yawAdjustment = 0;
      // }

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

      lastTargetTime = Timer.getFPGATimestamp();
      lastDistanceX = distanceX;
      lastDistanceY = distanceY;
      lastYaw = yaw;
    } else {
      // Stop the robot if no targets are found
      if (Timer.getFPGATimestamp() - lastTargetTime < TARGET_TIMEOUT) {
        distanceX = lastDistanceX;
        distanceY = lastDistanceY;
        yaw = lastYaw;
      } else {
        end(true);
      }
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
