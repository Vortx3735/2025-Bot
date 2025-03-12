package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Radians;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.sim.CANcoderSimState;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CoralWrist extends SubsystemBase {

  public static SparkMax coralWrist;

  private final CANcoder wristEncoder;
  private double position;

  private PIDController coralPID;
  private double ki, kp, kd;

  public double wristSpeedDown = -0.1;
  public double wristSpeedUp = 0.2;
  public double error;
  private double intakeSpeed = 0.25;

  private final int kGearRatio = 60;
  private final Mechanism2d coralArmMech = new Mechanism2d(4, 4);
  private final MechanismLigament2d coralArm;
  private final MechanismRoot2d mechBase = coralArmMech.getRoot("Coral Arm Pivot", 2, 2);
  private final DCMotor wristGearbox = DCMotor.getNeo550(1);
  private SparkMaxSim wristSim;
  private final SingleJointedArmSim mArmSim =
      new SingleJointedArmSim(
          LinearSystemId.createDCMotorSystem(DCMotor.getNeo550(1), 0.25, kGearRatio),
          DCMotor.getNeo550(1),
          kGearRatio,
          .1,
          0,
          2 * Math.PI,
          false,
          Math.PI);

  // aaron chang
  /**
   * @param wristId The CAN ID of the wrist motor.
   * @param wristEncoderId The CAN ID of the wrist encoder.
   */
  public CoralWrist(int wristId, int wristEncoderId) {
    // Motor configurations
    SparkMaxConfig coralWristConfig = new SparkMaxConfig();

    // Initialize wrist motor and encoder
    coralWrist = new SparkMax(wristId, MotorType.kBrushless);
    wristEncoder = new CANcoder(wristEncoderId);
    kp = 3;
    coralPID = new PIDController(kp, ki, kd);

    // Configure wrist motor settings
    coralWristConfig.inverted(false).idleMode(IdleMode.kBrake);
    // coralWristConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder).pid(1.0, 0.0,
    // 0.0);
    coralWrist.configure(
        coralWristConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    wristSim = new SparkMaxSim(coralWrist, wristGearbox);
    coralArm = mechBase.append(new MechanismLigament2d("Coral Arm", 1, 90));
  }

  public boolean getPositionFinished(double setpoint, double currentPos) {
    return (Math.abs(setpoint - currentPos) < 0.03);
  }

  public void moveWristUp() {
    moveWrist(wristSpeedUp);
  }

  public void moveWristDown() {
    moveWrist(wristSpeedDown);
  }

  public void moveWrist(double speed) {
    coralWrist.set(speed);
  }

  public void stopWrist() {
    // stop wrist
    coralWrist.set(0);
  }

  /**
   * @Param targetPos The target position to move the wrist to.
   */
  public Command moveWristToPosition(double targetPos) {
    return new FunctionalCommand(
            () -> {},
            () -> coralWrist.set(coralPID.calculate(position, targetPos)),
            (x) -> this.hold(position),
            () -> this.atSetpoint(targetPos),
            this)
        .withName("Move Coral Wrist to Position");
  }

  public Boolean atSetpoint(double targetPos) {
    if (Math.abs(targetPos - position) < .02) {
      return true;
    }
    return false;
  }

  public Command moveWristToHP() {
    return moveWristToPosition(-0.34).withName("Move Coral Wrist to HP");
  }

  public Command moveWristToL2() {
    // return moveWristToPosition(-0.38).withName("Move Coral Wrist to L2");
    return moveWristToPosition(-0.36).withName("Move Coral Wrist to L2");
  }

  public Command moveWristToL3() {
    return moveWristToPosition(-0.38).withName("Move Coral Wrist to L3");
  }

  public Command moveWristToL4() {
    // return moveWristToPosition(-0.48).withName("Move Coral Wrist to L4");
    return moveWristToPosition(-0.44).withName("Move Coral Wrist to L4");
    // return moveWristToPosition(-0.46).withName("Move Coral Wrist to L4");
  }

  public void hold(double targetPos) {
    // coralWrist.set(coralPID.calculate(position, targetPos) + coralFF.calculate(position, kv));
    coralWrist.set(coralPID.calculate(position, targetPos));
  }

  public double getWristPosition() {
    // get wrist position
    return wristEncoder.getAbsolutePosition().getValueAsDouble();
  }

  public void publishInitialValues() {
    SmartDashboard.putNumber("CoralWrist/intakeSpeed", intakeSpeed);

    SmartDashboard.putNumber("CoralWrist/Wrist Up Speed", wristSpeedUp);
    SmartDashboard.putNumber("CoralWrist/Wrist Down Speed", wristSpeedDown);
  }

  @Override
  public void periodic() {
    // Update wrist position
    position = getWristPosition();

    wristSpeedUp = SmartDashboard.getNumber("CoralWrist/Wrist Up Speed", wristSpeedUp);
    wristSpeedDown = SmartDashboard.getNumber("CoralWrist/Wrist Down Speed", wristSpeedDown);

    // Publish Wrist Position
    SmartDashboard.putNumber("CoralWrist/Wrist Position", position);
    SmartDashboard.putNumber("CoralWrist/Wrist Position", position);
    SmartDashboard.putBoolean(
        "CoralWrist/Wristbooleanfinished", getPositionFinished(-0.38, position));

    SmartDashboard.putData("CoralWrist/CoralWristVisualizer", coralArmMech);
  }

  @Override
  public void simulationPeriodic() {
    CANcoderSimState wristEncoderSim = wristEncoder.getSimState();
    // In this method, we update our simulation of what our arm is doing
    // First, we set our "inputs" (voltages)
    mArmSim.setInput(wristSim.getAppliedOutput() * RoboRioSim.getVInVoltage());

    // Next, we update it. The standard loop time is 20ms.
    mArmSim.update(0.02);

    // Now, we update the Spark Flex
    wristSim.iterate(
        Units.radiansPerSecondToRotationsPerMinute( // motor velocity, in RPM
            mArmSim.getVelocityRadPerSec()),
        RoboRioSim.getVInVoltage(), // Simulated battery voltage, in Volts
        0.02); // Time interval, in Seconds

    // SimBattery estimates loaded battery voltages
    // This should include all motors being simulated
    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(mArmSim.getCurrentDrawAmps()));

    // Update any external GUI displays or values as desired
    // For example, a Mechanism2d Arm based on the simulated arm angle
    coralArm.setAngle(Units.radiansToDegrees(mArmSim.getAngleRads()));
    Angle armAngle = Angle.ofBaseUnits(mArmSim.getAngleRads() - Math.PI, Radians);
    wristEncoderSim.setRawPosition(armAngle);
  }
}
