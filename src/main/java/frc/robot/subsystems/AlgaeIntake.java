package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AlgaeIntake extends SubsystemBase {
  static SparkMax leftAlgaeMotor;
  static SparkMax rightAlgaeMotor;

  private double intakeSpeed = 0.5;

  /**
   * @param leftMotorID The CAN ID of the left intake motor.
   * @param rightMotorID The CAN ID of the right intake motor.
   */
  public AlgaeIntake(int leftMotorID, int rightMotorID) {
    // Configure motor settings
    SparkMaxConfig algaeMotorConfig = new SparkMaxConfig();

    // Initialize intake motors
    leftAlgaeMotor = new SparkMax(leftMotorID, MotorType.kBrushless);
    rightAlgaeMotor = new SparkMax(rightMotorID, MotorType.kBrushless);

    // Set motor configurations
    algaeMotorConfig.inverted(true).idleMode(IdleMode.kBrake);
    leftAlgaeMotor.configure(
        algaeMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    rightAlgaeMotor.configure(
        algaeMotorConfig.inverted(false),
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
  }

  public void intake() {
    leftAlgaeMotor.set(intakeSpeed);
    rightAlgaeMotor.set(intakeSpeed);
  }

  public void outtake() {
    leftAlgaeMotor.set(-intakeSpeed);
    rightAlgaeMotor.set(-intakeSpeed);
  }

  public void stopIntake() {
    // stop motor
    leftAlgaeMotor.set(0);
    rightAlgaeMotor.set(0);
  }

  public void publishInitialValues() {
    SmartDashboard.putNumber("AlgaeIntake/intakeSpeed", intakeSpeed);
  }

  @Override
  public void periodic() {
    intakeSpeed = SmartDashboard.getNumber("AlgaeIntake/intakeSpeed", intakeSpeed);
  }
}
