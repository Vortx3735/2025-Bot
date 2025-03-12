package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.BooleanSupplier;

public class Elevator extends SubsystemBase {
  public static TalonFX leftElevatorMotor;
  public static TalonFX rightElevatorMotor;
  private static CANcoder elevatorEncoder;

  public double position;
  public double elevatorSpeed;

  // create a Motion Magic request, voltage output
  private final MotionMagicVoltage m_request = new MotionMagicVoltage(0);

  // Define soft limits
  public static double LOWER_LIMIT = 0;
  public static double UPPER_LIMIT = 5;
  public static double maxPosition = 0;
  public static double positionCutoff = 2;

  private final int kGearRatio = 15;
  private final Mechanism2d elevatorMech = new Mechanism2d(1, 6);
  private final MechanismRoot2d mechBase = elevatorMech.getRoot("base", 0.5, 0);
  private final DCMotorSim m_motorSimModel =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60Foc(2), 0.001, kGearRatio),
          DCMotor.getKrakenX60Foc(2));

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

    mechBase.append(new MechanismLigament2d("elevator", 0.1, 90));
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
    fx_cfg.MotionMagic.MotionMagicCruiseVelocity = 4.2;
    fx_cfg.MotionMagic.MotionMagicAcceleration = 15;
    fx_cfg.MotionMagic.MotionMagicJerk = 200;

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

  public boolean isSafe(){
    if(position>1){
      return true;
    }
    else {
      return false;
    }
  }

  public boolean getPositionFinished(double setpoint) {
    return Math.abs(setpoint - position) < .025;
  }

  public double getPosition() {
    return position;
  }

  public Command moveElevatorToBottom() {
    double setpoint = 0;
    return new RunCommand(() -> moveElevatorToPosition(setpoint), this)
        .until(() -> this.getPositionFinished(setpoint))
        .withName("Move Elevator to Bottom");
  }

  public Command moveElevatorToHP() {
    double setpoint = 1.04;
    return new RunCommand(() -> moveElevatorToPosition(setpoint), this)
        .until(() -> this.getPositionFinished(setpoint))
        .withName("Move Elevator to HP");
  }

  public Command moveElevatorToL1() {
    double setpoint = 0.18;
    return new RunCommand(() -> moveElevatorToPosition(setpoint), this)
        .until(() -> this.getPositionFinished(setpoint))
        .withName("Move Elevator to L1");
  }

  public Command moveElevatorToL2() {
    double setpoint = 0.4;
    return new RunCommand(() -> moveElevatorToPosition(setpoint), this)
        .until(() -> this.getPositionFinished(setpoint))
        .withName("Move Elevator to L2");
  }

  public Command moveElevatorToL3() {
    double setpoint = 1.8;
    return new RunCommand(() -> moveElevatorToPosition(setpoint), this)
        .until(() -> this.getPositionFinished(setpoint))
        .withName("Move Elevator to L3");
  }

  public Command moveElevatorToL4() {
    double setpoint = 4.93;
    // double setpoint = 4.95;
    return new RunCommand(() -> moveElevatorToPosition(setpoint), this)
        .until(() -> this.getPositionFinished(setpoint))
        .withName("Move Elevator to L4");
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
    // SmartDashboard.putNumber("elevator/Elevator Speed", elevatorSpeed);
    // SmartDashboard.putNumber("elevator/UpperLimit", UPPER_LIMIT);
    // SmartDashboard.putNumber("elevator/LowerLimit", LOWER_LIMIT);
  }

  @Override
  public void periodic() {
    position = leftElevatorMotor.getPosition().getValueAsDouble();

    mechBase.setPosition(0, position);

    UPPER_LIMIT = SmartDashboard.getNumber("elevator/UpperLimit", UPPER_LIMIT);
    LOWER_LIMIT = SmartDashboard.getNumber("elevator/LowerLimit", LOWER_LIMIT);

    // Values
    SmartDashboard.putNumber("elevator/Elevator Position", position);
    SmartDashboard.putNumber(
        "elevator/Kraken Left pos", leftElevatorMotor.getPosition().getValueAsDouble());
    SmartDashboard.putNumber(
        "elevator/Kraken Right Pos", rightElevatorMotor.getPosition().getValueAsDouble());

    // Add Slider to dynamically change Elevator Speed
    elevatorSpeed = SmartDashboard.getNumber("elevator/Elevator Speed", elevatorSpeed);
    
    SmartDashboard.putBoolean("elevator/isSafe", isSafe());

    SmartDashboard.putData("elevator/Visualizer", elevatorMech);
  }

  @Override
  public void simulationPeriodic() {
    var leftTalonFXSim = leftElevatorMotor.getSimState();
    var rightTalonFXSim = rightElevatorMotor.getSimState();
    // This method will be called once per scheduler run during simulation

    // set the supply voltage of the TalonFX
    leftTalonFXSim.setSupplyVoltage(RobotController.getBatteryVoltage());
    rightTalonFXSim.setSupplyVoltage(RobotController.getBatteryVoltage());

    // get the motor voltage of the TalonFX
    var motorVoltage = leftTalonFXSim.getMotorVoltageMeasure();

    // use the motor voltage to calculate new position and velocity
    // using WPILib's DCMotorSim class for physics simulation
    m_motorSimModel.setInputVoltage(motorVoltage.in(Volts));
    m_motorSimModel.update(0.020); // assume 20 ms loop time

    // apply the new rotor position and velocity to the TalonFX;
    // note that this is rotor position/velocity (before gear ratio), but
    // DCMotorSim returns mechanism position/velocity (after gear ratio)
    leftTalonFXSim.setRawRotorPosition(m_motorSimModel.getAngularPosition().times(kGearRatio));
    leftTalonFXSim.setRotorVelocity(m_motorSimModel.getAngularVelocity().times(kGearRatio));
    rightTalonFXSim.setRawRotorPosition(m_motorSimModel.getAngularPosition().times(kGearRatio));
    rightTalonFXSim.setRotorVelocity(m_motorSimModel.getAngularVelocity().times(kGearRatio));
  }
}
