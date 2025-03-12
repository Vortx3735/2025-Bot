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

public class AlgaeIntake extends SubsystemBase {

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
  private double intakeSpeed = 0.5;

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

  public static boolean hasAlgae = false;

  /**
   * @param leftMotorID The CAN ID of the left intake motor.
   * @param rightMotorID The CAN ID of the right intake motor.
   * @param wristID The CAN ID of the wrist motor.
   * @param wristEncoderID The CAN ID of the wrist encoder.
   */
  public AlgaeIntake(int leftMotorID, int rightMotorID, int wristID, int wristEncoderID) {
    // Configure motor settings
    SparkMaxConfig algaeMotorConfig = new SparkMaxConfig();
    SparkMaxConfig algaeWristConfig = new SparkMaxConfig();

    // Initialize intake motors
    leftAlgaeMotor = new SparkMax(leftMotorID, MotorType.kBrushless);
    rightAlgaeMotor = new SparkMax(rightMotorID, MotorType.kBrushless);

    // Set motor configurations
    algaeMotorConfig.inverted(true).idleMode(IdleMode.kBrake);
    algaeMotorConfig.secondaryCurrentLimit(currentLimit);
    leftAlgaeMotor.configure(
        algaeMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    rightAlgaeMotor.configure(
        algaeMotorConfig.inverted(false),
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

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

    resetAlgae();
  }

  public boolean hasAlgae() {
    averageCurrent = (leftAlgaeMotor.getOutputCurrent() + rightAlgaeMotor.getOutputCurrent()) / 2;
    // averageCurrent += averageCurrent/2;
    if (averageCurrent >= (currentLimit - 5)) {
      hasAlgae = true;
    }
    return hasAlgae;
  }

  public static void resetAlgae() {
    hasAlgae = false;
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

  public void intake() {
    unstowWrist();
    if (hasAlgae()) {
      stopIntake();
    } else {
      leftAlgaeMotor.set(intakeSpeed);
      rightAlgaeMotor.set(intakeSpeed);
    }
  }

  public void outtake() {
    leftAlgaeMotor.set(-intakeSpeed);
    rightAlgaeMotor.set(-intakeSpeed);
    resetAlgae();
  }

  public void stopIntake() {
    // stop motor
    leftAlgaeMotor.set(0);
    rightAlgaeMotor.set(0);
  }

  public void moveWrist(double speed) {
    // move wrist
    algaeWrist1.set(speed);
  }

  public void moveWristUp() {
    algaeWrist1.set(wristSpeedUp);
  }

  public void stowWrist() {
    if (!hasAlgae) {
      stopIntake();
      moveWristToPosition(-0.42);
    }
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
    SmartDashboard.putNumber("AlgaeIntake/Wrist P", kp);
    SmartDashboard.putNumber("AlgaeIntake/Wrist I", ki);
    SmartDashboard.putNumber("AlgaeIntake/Wrist D", kd);

    SmartDashboard.putNumber("AlgaeIntake/Wrist A", ka);
    SmartDashboard.putNumber("AlgaeIntake/Wrist G", kg);
    SmartDashboard.putNumber("AlgaeIntake/Wrist S", ks);
    SmartDashboard.putNumber("AlgaeIntake/Wrist V", kv);

    SmartDashboard.putNumber("AlgaeIntake/intakeSpeed", intakeSpeed);

    SmartDashboard.putNumber("AlgaeIntake/Wrist Up Speed", wristSpeedUp);
    SmartDashboard.putNumber("AlgaeIntake/Wrist Down Speed", wristSpeedDown);
    SmartDashboard.putNumber("AlgaeIntake/Wrist Error", error);
    SmartDashboard.putNumber("AlgaeIntake/Left Intake Current", leftIntakeCurrent);
    SmartDashboard.putNumber("AlgaeIntake/Right Intake Current", rightIntakeCurrent);
  }

  @Override
  public void periodic() {
    // Update wrist position
    position = getWristPosition();

    // Get PID values from the dashboard (or use default values)
    kp = SmartDashboard.getNumber("AlgaeIntake/Wrist P", kp);
    ki = SmartDashboard.getNumber("AlgaeIntake/Wrist I", ki);
    kd = SmartDashboard.getNumber("AlgaeIntake/Wrist D", kd);

    ka = SmartDashboard.getNumber("AlgaeIntake/Wrist A", ka);
    kg = SmartDashboard.getNumber("AlgaeIntake/Wrist G", kg);
    ks = SmartDashboard.getNumber("AlgaeIntake/Wrist S", ks);
    kv = SmartDashboard.getNumber("AlgaeIntake/Wrist V", kv);

    wristSpeedDown = SmartDashboard.getNumber("AlgaeIntake/Wrist Down Speed", wristDownDefault);
    wristSpeedUp = SmartDashboard.getNumber("AlgaeIntake/Wrist Up Speed", wristUpDefault);
    error = SmartDashboard.getNumber("AlgaeIntake/Wrist Error", errorDefault);

    intakeSpeed = SmartDashboard.getNumber("AlgaeIntake/intakeSpeed", intakeSpeed);

    // Publish Wrist Position
    SmartDashboard.putNumber("AlgaeIntake/Wrist Position", position);
    SmartDashboard.putNumber("AlgaeIntake/Left Max Intake Current", leftIntakeCurrent);
    SmartDashboard.putNumber("AlgaeIntake/Right Max Intake Current", rightIntakeCurrent);
    SmartDashboard.putNumber("AlgaeIntake/Left Intake Current", leftAlgaeMotor.getOutputCurrent());
    SmartDashboard.putNumber(
        "AlgaeIntake/Right Intake Current", rightAlgaeMotor.getOutputCurrent());
    SmartDashboard.putBoolean("AlgaeIntake/Has Algae", hasAlgae);
    SmartDashboard.putNumber("AlgaeIntake/Average Current", averageCurrent);

    if (leftAlgaeMotor.getOutputCurrent() > leftIntakeCurrent) {
      leftIntakeCurrent = leftAlgaeMotor.getOutputCurrent();
    }
    if (rightAlgaeMotor.getOutputCurrent() > rightIntakeCurrent) {
      rightIntakeCurrent = rightAlgaeMotor.getOutputCurrent();
    }
  }
}
