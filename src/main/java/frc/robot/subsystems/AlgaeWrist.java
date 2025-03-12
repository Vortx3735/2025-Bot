package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AlgaeWrist extends SubsystemBase {

  private PIDController wristPID;
  private double ki, kp, kd;

  private ArmFeedforward wristFF;
  private double ka, kg, ks, kv;

  private PIDController algaePID;
  private ArmFeedforward algaeFF;

  static SparkMax leftAlgaeMotor;
  static SparkMax rightAlgaeMotor;
  static SparkMax algaeWrist1;

  private final CANcoder wristEncoder;

  private double position;

  public double wristDownDefault = -0.3;
  public double wristUpDefault = 0.6;
  public double errorDefault = 0.03;

  public double wristSpeedDown;
  public double wristSpeedUp;
  public double error;
  public Object moveWristUp;
  public double leftIntakeCurrent;
  public double rightIntakeCurrent;

  public double averageCurrent;
  public int currentLimit = 35;

  /**
   * @param leftMotorID The CAN ID of the left intake motor.
   * @param rightMotorID The CAN ID of the right intake motor.
   * @param wristID The CAN ID of the wrist motor.
   * @param wristEncoderID The CAN ID of the wrist encoder.
   */
  public AlgaeWrist(int wristID, int wristEncoderID) {
    // Configure motor settings
    SparkMaxConfig algaeWristConfig = new SparkMaxConfig();

    // Initialize wrist motor and encoder
    algaeWrist1 = new SparkMax(wristID, MotorType.kBrushless);
    wristEncoder = new CANcoder(wristEncoderID);

    // Configure wrist motor settings
    algaeWristConfig.inverted(true).idleMode(IdleMode.kBrake);
    algaeWristConfig.encoder.positionConversionFactor(1000).velocityConversionFactor(1000);
    algaeWristConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder).pid(1.0, 0.0, 0.0);

    algaeWrist1.configure(
        algaeWristConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    wristSpeedDown = wristDownDefault;
    wristSpeedUp = wristUpDefault;
    error = errorDefault;

    algaePID = new PIDController(4, 0, 0);
    algaeFF = new ArmFeedforward(0, 0.11, 0);
  }

  /**
   * @Param targetPos The target position to move the wrist to.
   */
  public boolean moveWristToPosition(double targetPos) {
    if (Math.abs(targetPos - position) < .02) {
      stopWrist();
      return true;
    }
    algaeWrist1.set(
        MathUtil.clamp(
            algaePID.calculate(position, targetPos) + algaeFF.calculate(position, targetPos),
            -0.6,
            0.6));
    return false;
  }

  public void moveWrist(double speed) {
    // move wrist
    algaeWrist1.set(speed);
  }

  public void moveWristUp() {
    algaeWrist1.set(wristSpeedUp);
  }

  public void stowWrist() {
    moveWristToPosition(-0.42);
  }

  public void unstowWrist() {
    if (Elevator.getPosition() > 1) {
      moveWristToPosition(-0.64);
    }
  }

  public void moveWristDown() {
    algaeWrist1.set(wristSpeedDown);
  }

  public void stopWrist() {
    // stop wrist
    algaeWrist1.set(0);
  }

  public double getWristPosition() {
    // get wrist position
    return wristEncoder.getAbsolutePosition().getValueAsDouble();
  }

  public void resetWristPosition() {
    // reset wrist position
    algaeWrist1.getEncoder().setPosition(0);
  }

  public void hold() {
    algaeWrist1.set(
        wristPID.calculate(position * 2 * Math.PI, (int) position * 2 * Math.PI)
            + wristFF.calculate(position * 2 * Math.PI, kv));
  }

  public void publishInitialValues() {
    // Publish initial values to the dashboard
    SmartDashboard.putNumber("AlgaeWrist/Wrist P", kp);
    SmartDashboard.putNumber("AlgaeWrist/Wrist I", ki);
    SmartDashboard.putNumber("AlgaeWrist/Wrist D", kd);

    SmartDashboard.putNumber("AlgaeWrist/Wrist A", ka);
    SmartDashboard.putNumber("AlgaeWrist/Wrist G", kg);
    SmartDashboard.putNumber("AlgaeWrist/Wrist S", ks);
    SmartDashboard.putNumber("AlgaeWrist/Wrist V", kv);

    SmartDashboard.putNumber("AlgaeWrist/Wrist Up Speed", wristSpeedUp);
    SmartDashboard.putNumber("AlgaeWrist/Wrist Down Speed", wristSpeedDown);
    SmartDashboard.putNumber("AlgaeWrist/Wrist Error", error);
  }

  @Override
  public void periodic() {
    // Update wrist position
    position = getWristPosition();

    // Get PID values from the dashboard (or use default values)
    kp = SmartDashboard.getNumber("AlgaeWrist/Wrist P", kp);
    ki = SmartDashboard.getNumber("AlgaeWrist/Wrist I", ki);
    kd = SmartDashboard.getNumber("AlgaeWrist/Wrist D", kd);

    ka = SmartDashboard.getNumber("AlgaeWrist/Wrist A", ka);
    kg = SmartDashboard.getNumber("AlgaeWrist/Wrist G", kg);
    ks = SmartDashboard.getNumber("AlgaeWrist/Wrist S", ks);
    kv = SmartDashboard.getNumber("AlgaeWrist/Wrist V", kv);

    wristSpeedDown = SmartDashboard.getNumber("AlgaeWrist/Wrist Down Speed", wristDownDefault);
    wristSpeedUp = SmartDashboard.getNumber("AlgaeWrist/Wrist Up Speed", wristUpDefault);
    error = SmartDashboard.getNumber("AlgaeWrist/Wrist Error", errorDefault);

    // Publish Wrist Position
    SmartDashboard.putNumber("AlgaeWrist/Wrist Position", position);
  }
}
