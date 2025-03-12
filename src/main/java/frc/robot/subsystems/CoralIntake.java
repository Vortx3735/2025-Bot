package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SensorConstants;

public class CoralIntake extends SubsystemBase {

  public static SparkMax leftCoralMotor;
  public static SparkMax rightCoralMotor;

  private double intakeSpeed = 0.25;

  public DigitalInput leftCoralBeamBreak = new DigitalInput(SensorConstants.CORAL_LEFT_BEAM_BREAK);
  public DigitalInput rightCoralBeamBreak =
      new DigitalInput(SensorConstants.CORAL_RIGHT_BEAM_BREAK);

  // aaron chang
  /**
   * @param leftMotorId The CAN ID of the left intake motor.
   * @param rightMotorId The CAN ID of the right intake motor.
   * @param wristId The CAN ID of the wrist motor.
   * @param wristEncoderId The CAN ID of the wrist encoder.
   */
  public CoralIntake(int leftMotorId, int rightMotorId) {
    // Motor configurations
    SparkMaxConfig coralMotorConfig = new SparkMaxConfig();

    // Initialize intake motors
    leftCoralMotor = new SparkMax(leftMotorId, MotorType.kBrushless);
    rightCoralMotor = new SparkMax(rightMotorId, MotorType.kBrushless);

    // Set motor configurations
    coralMotorConfig.inverted(true).idleMode(IdleMode.kBrake);
    leftCoralMotor.configure(
        coralMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    rightCoralMotor.configure(
        coralMotorConfig.inverted(false),
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
  }

  public boolean getPositionFinished(double setpoint, double currentPos) {
    return (Math.abs(setpoint - currentPos) < 0.03);
  }

  // returns true assuming beam break is broken
  public boolean hasCoral() {
    return (hasLeftCoral() || hasRightCoral());
  }

  public boolean hasLeftCoral() {
    return !leftCoralBeamBreak.get();
  }

  public boolean hasRightCoral() {
    return !rightCoralBeamBreak.get();
  }

  public Command intakeCommand(double speed) {
    return new RunCommand(() -> intake(speed), this)
        .withName("Coral Intake at Speed" + speed + "Command");
  }

  public Command intakeCommand() {
    return new RunCommand(() -> intake(intakeSpeed), this).withName("Coral Intake Command");
  }

  public Command outtakeCommand() {
    return new RunCommand(() -> outtake(), this).withName("Coral Outtake Command");
  }

  public Command stopIntakeCommand() {
    return new InstantCommand(() -> stopIntake(), this).withName("Stop Coral Intake Command");
  }

  // public FunctionalCommand holdCommand() {
  //   double currentPos = position;
  //   return new RunCommand(() -> hold(currentPos));
  // }

  public void intake(double speed) {
    if (!hasRightCoral()) {
      rightCoralMotor.set(speed);
    }
    if (!hasLeftCoral()) {
      leftCoralMotor.set(speed);
    }
  }

  private void outtake() {
    leftCoralMotor.set(-intakeSpeed - 0.3);
    rightCoralMotor.set(-intakeSpeed - 0.3);
  }

  private void outtake(double speed) {
    double x = speed;
    leftCoralMotor.set(x);
    rightCoralMotor.set(x);
  }

  public void stopIntake() {
    // stop motor
    leftCoralMotor.set(0);
    rightCoralMotor.set(0);
  }

  public void publishInitialValues() {
    SmartDashboard.putNumber("CoralIntake/intakeSpeed", intakeSpeed);
  }

  @Override
  public void periodic() {
    // Update wrist position
    SmartDashboard.putBoolean("CoralIntake/Left Coral", hasLeftCoral());
    SmartDashboard.putBoolean("CoralIntake/Right Coral", hasRightCoral());

    // Update intake speed
    intakeSpeed = SmartDashboard.getNumber("CoralIntake/intakeSpeed", intakeSpeed);
  }
}
