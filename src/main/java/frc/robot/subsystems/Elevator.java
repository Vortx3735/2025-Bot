package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.BooleanSupplier;

public class Elevator extends SubsystemBase {
  public static TalonFX leftElevatorMotor;
  public static TalonFX rightElevatorMotor;
  private static CANcoder elevatorEncoder;

  public static double position;
  public double elevatorSpeed;

  // create a Motion Magic request, voltage output
  private final MotionMagicVoltage m_request = new MotionMagicVoltage(0);

  // Define soft limits
  public static double LOWER_LIMIT = 0;
  public static double UPPER_LIMIT = 5;
  public static double maxPosition = 0;
  public static double positionCutoff = 2;

  /**
   * @param encoderID CAN ID of the CANcoder.
   * @param leftMotorID CAN ID of the left elevator motor.
   * @param rightMotorID CAN ID of the right elevator motor.
   */
  public Elevator(int encoderID, int leftMotorID, int rightMotorID) {
    elevatorEncoder = new CANcoder(encoderID);
    leftElevatorMotor = new TalonFX(leftMotorID);
    rightElevatorMotor = new TalonFX(rightMotorID);

    configureCANcoder();
    configureTalonFX();

    elevatorSpeed = 0.15;
  }

  /**
   * This method configures the CANcoder sensor with specific settings for the absolute sensor
   * discontinuity point and magnet offset, and applies these settings to the ElevatorEncoder
   */
  private void configureCANcoder() {
    CANcoderConfiguration cc_cfg = new CANcoderConfiguration();
    cc_cfg.MagnetSensor.AbsoluteSensorDiscontinuityPoint =
        0.0; // Set to an appropriate double value
    cc_cfg.MagnetSensor.MagnetOffset = 0.4;
    cc_cfg.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
    elevatorEncoder.getConfigurator().apply(cc_cfg);
  }

  public void configureTalonFX() {
    TalonFXConfiguration fx_cfg = new TalonFXConfiguration();
    fx_cfg.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
    fx_cfg.Feedback.SensorToMechanismRatio = 15.0;
    fx_cfg.Feedback.RotorToSensorRatio = 1;
    fx_cfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    // Set PID & Feedforward gains

    fx_cfg.Slot0.kP = 35;
    fx_cfg.Slot0.kI = 0;
    fx_cfg.Slot0.kD = 0.0;
    fx_cfg.Slot0.kV = 0.8;
    fx_cfg.Slot0.kA = 0.075;
    fx_cfg.Slot0.kG = 0.368;

    // Motion Magic settings
    fx_cfg.MotionMagic.MotionMagicCruiseVelocity = 1.5;
    fx_cfg.MotionMagic.MotionMagicAcceleration = 25;
    fx_cfg.MotionMagic.MotionMagicJerk = 300;

    leftElevatorMotor.setPosition(0);
    rightElevatorMotor.setPosition(0);

    leftElevatorMotor.getConfigurator().apply(fx_cfg);
    rightElevatorMotor.getConfigurator().apply(fx_cfg);
  }

  public void hold(double currentPos) {
    leftElevatorMotor.setControl(m_request.withPosition(currentPos));
    rightElevatorMotor.setControl(m_request.withPosition(currentPos));
  }

  public void zeroElevator() {
    leftElevatorMotor.setPosition(0);
    rightElevatorMotor.setPosition(0);
  }

  /**
   * Moves the elevator to the specified position using Motion Magic control
   *
   * @param targetPosition The target position to move the elevator to
   */
  public void moveElevatorToPosition(double targetPos) {
    // Prevent moving past soft limits
    leftElevatorMotor.setControl(m_request.withPosition(targetPos));
    rightElevatorMotor.setControl(m_request.withPosition(targetPos));
  }

  public void moveElevatorToPositionSlow(double targetPos) {
    // Prevent moving past soft limits
    leftElevatorMotor.setControl(m_request.withPosition(targetPos));
    rightElevatorMotor.setControl(m_request.withPosition(targetPos));
  }

  public boolean getPositionFinished(double setpoint) {
    return Math.abs(setpoint - position) < .025;
  }

  public static double getPosition() {
    return position;
  }

  public Command moveElevatorToBottom() {
    double setpoint = 0;
    return new RunCommand(() -> moveElevatorToPosition(setpoint))
        .until(() -> this.getPositionFinished(setpoint));
  }

  public Command moveElevatorToHP() {
    double setpoint = 1.04;
    return new RunCommand(() -> moveElevatorToPosition(setpoint))
        .until(() -> this.getPositionFinished(setpoint));
  }

  public Command moveElevatorToL1() {
    double setpoint = 0.18;
    return new RunCommand(() -> moveElevatorToPosition(setpoint))
        .until(() -> this.getPositionFinished(setpoint));
  }
  public Command moveElevatorToL2() {
    double setpoint = 0.514;
    return new RunCommand(() -> moveElevatorToPosition(setpoint))
        .until(() -> this.getPositionFinished(setpoint));
  }

  public Command moveElevatorToL2Auto() {
    double setpoint = 0.22;
    return new RunCommand(() -> moveElevatorToPosition(setpoint), this)
        .until(() -> this.getPositionFinished(setpoint));
  }

  public Command moveElevatorToL3() {
    double setpoint = 1.94;
    return new RunCommand(() -> moveElevatorToPosition(setpoint))
        .until(() -> this.getPositionFinished(setpoint));
  }

  public Command moveElevatorToL4() {
    double setpoint = 4.9;
    return new RunCommand(() -> moveElevatorToPosition(setpoint))
        .until(() -> this.getPositionFinished(setpoint));
  }

  public double getElevatorCoefficient() {
    // if (position > positionCutoff){
    //   double a = maxPosition-positionCutoff;
    //   double b = position - positionCutoff;
    //   if ((a-b)<0.125){
    //     return(0.125);
    //   }
    //   else{
    //     return ((a-b)/a);
    //   }
    // }
    // else{
    return (1.0);
    // }
  }

  public void moveElevatorUp() {
    if (position <= UPPER_LIMIT) {
      leftElevatorMotor.set(elevatorSpeed);
      rightElevatorMotor.set(elevatorSpeed);
    } else {
      stopElevator();
    }
  }

  public void moveElevatorUpSetpoint() {
    if (position <= UPPER_LIMIT) {
      // leftElevatorMotor.set(elevatorSpeed);
      // rightElevatorMotor.set(elevatorSpeed);
      moveElevatorToPosition(position + .1);
    } else {
      stopElevator();
    }
  }

  public void moveElevatorDown() {
    if (position >= LOWER_LIMIT) {
      leftElevatorMotor.set(-elevatorSpeed);
      rightElevatorMotor.set(-elevatorSpeed);
    } else {
      stopElevator();
    }
  }

  public void moveElevatorDownSetpoint() {
    if (position >= LOWER_LIMIT) {
      // leftElevatorMotor.set(-elevatorSpeed);
      // rightElevatorMotor.set(-elevatorSpeed);
      moveElevatorToPosition(position - .1);
    } else {
      stopElevator();
    }
  }

  public BooleanSupplier atL4() {
    return () -> position > 4.8;
  }

  public void setElevatorSpeed(double speed) {
    leftElevatorMotor.set(speed);
    rightElevatorMotor.set(speed);
  }

  public void stopElevator() {
    leftElevatorMotor.set(0);
    rightElevatorMotor.set(0);
  }

  public void setBrakeMode() {
    leftElevatorMotor.setNeutralMode(NeutralModeValue.Brake);
    rightElevatorMotor.setNeutralMode(NeutralModeValue.Brake);
  }

  public void publishInitialValues() {
    SmartDashboard.putNumber("elevator/Elevator Speed", elevatorSpeed);
    SmartDashboard.putNumber("UpperLimit", UPPER_LIMIT);
    SmartDashboard.putNumber("LowerLimit", LOWER_LIMIT);
  }

  @Override
  public void periodic() {
    position = leftElevatorMotor.getPosition().getValueAsDouble();

    UPPER_LIMIT = SmartDashboard.getNumber("UpperLimit", UPPER_LIMIT);
    LOWER_LIMIT = SmartDashboard.getNumber("LowerLimit", LOWER_LIMIT);

    // Values
    SmartDashboard.putNumber("elevator/Elevator Position", position);
    SmartDashboard.putNumber(
        "elevator/Kraken Left pos", leftElevatorMotor.getPosition().getValueAsDouble());
    SmartDashboard.putNumber(
        "elevator/Kraken Right Pos", rightElevatorMotor.getPosition().getValueAsDouble());

    // Add Slider to dynamically change Elevator Speed
    elevatorSpeed = SmartDashboard.getNumber("elevator/Elevator Speed", elevatorSpeed);
  }
}
