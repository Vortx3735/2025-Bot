package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;

public class AutoAlignCommand extends Command {
  private final Drive drivetrain;

  // PID constants for alignment
  private PIDController yawPID;
  private PIDController xPID;
  private PIDController yPID;

  private final double kP_Yaw = 2; // Proportional constant for yaw correction
  private final double kP_X = 1;
  private final double kP_Y = 2;

  private final double YAW_THRESHOLD = 0.12; // Degrees threshold for alignment
  private final double X_THRESHOLD = 0.03; // Meters threshold for alignment
  private final double Y_THRESHOLD = 0.01; // Meters threshold for alignment

  private double TARGET_X; // Target distance in meters 0.42
  private final double TARGET_Y = 0; // Target distance in meters
  private final double TARGET_YAW = -0.01; // Target rotation

  private double yawAdjustment;
  private double xAdjustment;
  private double yAdjustment;

  private double yaw;
  private double distanceX;
  private double distanceY;

  private Pose2d targetPose;
  private Pose2d tagPose;

  public AutoAlignCommand(Drive drivetrain, Pose2d targetPose, Pose2d tagPose) {
    this.targetPose = targetPose;
    this.tagPose = tagPose;
    TARGET_X = 0.55;
    SmartDashboard.putNumber("AutoAlign/TargetX", TARGET_X);
    this.drivetrain = drivetrain;
    yawPID = new PIDController(kP_Yaw, 0, 0);
    xPID = new PIDController(kP_X, 0, 0);
    yPID = new PIDController(kP_Y, 0, 0);

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

  public static Command generateCommand(Drive swerve, Pose2d goalPose, Pose2d tagPose) {
    return new AutoAlignCommand(swerve, goalPose, tagPose)
        .andThen(
            () -> {
              swerve.runVelocity(new ChassisSpeeds(0, 0, 0));
              swerve.stopWithX();
            });
  }

  @Override
  public void execute() {
    Translation2d vectorToTarget =
        tagPose.getTranslation().minus(drivetrain.getPose().getTranslation());
    Translation2d robotRelativeVector =
        vectorToTarget.rotateBy(drivetrain.getPose().getRotation().unaryMinus());
    distanceX = robotRelativeVector.getX(); // Positive means target is in front
    distanceY = robotRelativeVector.getY(); // Positive means target is to the left

    yaw =
        targetPose.relativeTo(drivetrain.getPose()).getRotation().getRadians()
            + Math.PI; // Yaw to the tag

    if (yaw < 0) {
      yaw += Math.PI;
    } else {
      yaw -= Math.PI;
    }

    // Calculate adjustments for yaw and forward movement
    yawAdjustment = yawPID.calculate(yaw, TARGET_YAW);
    xAdjustment = xPID.calculate(distanceX, TARGET_X);
    yAdjustment = yPID.calculate(distanceY, TARGET_Y);

    xAdjustment = MathUtil.clamp(xAdjustment, -0.2, 0.2);
    yAdjustment = MathUtil.clamp(yAdjustment, -0.2, 0.2);
    yawAdjustment = MathUtil.clamp(yawAdjustment, -0.2, 0.2);

    drivetrain.runVelocity(new ChassisSpeeds(-xAdjustment, -yAdjustment, -yawAdjustment));

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
  }

  @Override
  public boolean isFinished() {
    return isAligned();
  }

  @Override
  public void end(boolean interrupted) {
    // Stop the robot
    drivetrain.runVelocity(new ChassisSpeeds(0, 0, 0));
  }
}
