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

  private static SparkMax leftCoralMotor;
  private static SparkMax rightCoralMotor;

  private double intakeSpeed = 0.25;

  private DigitalInput leftCoralBeamBreak = new DigitalInput(SensorConstants.CORAL_LEFT_BEAM_BREAK);
  private DigitalInput rightCoralBeamBreak =
      new DigitalInput(SensorConstants.CORAL_RIGHT_BEAM_BREAK);

  // aaron chang
  /**
   * @param leftMotorId The CAN ID of the left intake motor.
   * @param rightMotorId The CAN ID of the right intake motor.
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

    SmartDashboard.putNumber("CoralIntake/intakeSpeed", intakeSpeed);
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

  private void intake(double speed) {
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

  @Override
  public void periodic() {
    intakeSpeed = SmartDashboard.getNumber("CoralIntake/intakeSpeed", intakeSpeed);
    SmartDashboard.putBoolean("CoralIntake/Left Coral", hasLeftCoral());
    SmartDashboard.putBoolean("CoralIntake/Right Coral", hasRightCoral());
  }
}
