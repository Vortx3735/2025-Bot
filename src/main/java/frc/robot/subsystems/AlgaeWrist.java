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

public class AlgaeWrist extends SubsystemBase {

  private PIDController algaePID;
  // private ArmFeedforward algaeFF;

  static SparkMax leftAlgaeMotor;
  static SparkMax rightAlgaeMotor;
  static SparkMax algaeWrist;

  private final CANcoder wristEncoder;

  private static double position;

  public double wristSpeedDown = -0.3;
  public double wristSpeedUp = 0.6;
  public double error = 0.03;

  public static final double STOW_POS = -0.42;

  private final int kGearRatio = 150;
  private final Mechanism2d algaeArmMech = new Mechanism2d(4, 4);
  private final MechanismLigament2d algaeArm;
  private final MechanismRoot2d mechBase = algaeArmMech.getRoot("Algae Arm Pivot", 2, 2);
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

  /**
   * @param wristID The CAN ID of the wrist motor.
   * @param wristEncoderID The CAN ID of the wrist encoder.
   */
  public AlgaeWrist(int wristID, int wristEncoderID) {
    // Configure motor settings
    SparkMaxConfig algaeWristConfig = new SparkMaxConfig();

    // Initialize wrist motor and encoder
    algaeWrist = new SparkMax(wristID, MotorType.kBrushless);
    wristEncoder = new CANcoder(wristEncoderID);

    // Configure wrist motor settings
    algaeWristConfig.inverted(true).idleMode(IdleMode.kBrake);

    algaeWrist.configure(
        algaeWristConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    algaePID = new PIDController(4, 0, 0);
    // algaeFF = new ArmFeedforward(0, 0.11, 0);

    wristSim = new SparkMaxSim(algaeWrist, wristGearbox);
    algaeArm = mechBase.append(new MechanismLigament2d("Algae Arm", 1, 90));

    SmartDashboard.putNumber("AlgaeWrist/Wrist Up Speed", wristSpeedUp);
    SmartDashboard.putNumber("AlgaeWrist/Wrist Down Speed", wristSpeedDown);
    SmartDashboard.putNumber("AlgaeWrist/Wrist Error", error);
  }

  /**
   * @Param targetPos The target position to move the wrist to.
   */
  // public boolean moveWristToPosition(double targetPos) {
  //   if (Math.abs(targetPos - position) < .02) {
  //     stopWrist();
  //     return true;
  //   }
  //   algaeWrist.set(
  //       MathUtil.clamp(
  //           algaePID.calculate(position, targetPos) + algaeFF.calculate(position, targetPos),
  //           -0.6,
  //           0.6));
  //   return false;
  // }

  /**
   * @Param targetPos The target position to move the wrist to.
   */
  public Command moveWristToPosition(double targetPos) {
    return new FunctionalCommand(
            () -> {},
            () -> algaeWrist.set(algaePID.calculate(position, targetPos)),
            (x) -> this.hold(targetPos),
            () -> this.atSetpoint(targetPos),
            this)
        .withName("Move Algae Wrist to Position");
  }

  public boolean atSetpoint(double targetPos) {
    if (Math.abs(targetPos - position) < .02) {
      return true;
    }
    return false;
  }

  public boolean isStowed() {
    if (getWristPosition() + error >= STOW_POS) {
      return true;
    }
    return false;
  }

  public void moveWrist(double speed) {
    // move wrist
    algaeWrist.set(speed);
  }

  public void moveWristUp() {
    algaeWrist.set(wristSpeedUp);
  }

  public void moveWristDown() {
    algaeWrist.set(wristSpeedDown);
  }

  public void stopWrist() {
    // stop wrist
    algaeWrist.set(0);
  }

  public Command stowWrist() {
    return moveWristToPosition(STOW_POS).withName("Stow Wrist");
  }

  public Command unstowWrist() {
    // return moveWristToPosition(-0.64);
    return moveWristToPosition(-0.5).withName("Unstow Wrist");
  }

  public double getWristPosition() {
    // get wrist position
    return wristEncoder.getAbsolutePosition().getValueAsDouble();
  }

  // public void hold(double targetPos) {
  //   algaeWrist.set(
  //     algaePID.calculate(position * 2 * Math.PI, targetPos * 2 * Math.PI)
  //           + wristFF.calculate(position * 2 * Math.PI, kv));
  // }

  public void hold(double targetPos) {
    // coralWrist.set(coralPID.calculate(position, targetPos) + coralFF.calculate(position, kv));
    algaeWrist.set(algaePID.calculate(position, targetPos));
  }

  @Override
  public void periodic() {
    // Update wrist position
    position = getWristPosition();

    // Get PID values from the dashboard (or use default values)
    wristSpeedDown = SmartDashboard.getNumber("AlgaeWrist/Wrist Down Speed", wristSpeedDown);
    wristSpeedUp = SmartDashboard.getNumber("AlgaeWrist/Wrist Up Speed", wristSpeedUp);
    error = SmartDashboard.getNumber("AlgaeWrist/Wrist Error", error);

    // Publish Wrist Position
    SmartDashboard.putNumber("AlgaeWrist/Wrist Position", position);

    SmartDashboard.putData("AlgaeWrist/AlgaeWristVisualizer", algaeArmMech);
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
    algaeArm.setAngle(Units.radiansToDegrees(mArmSim.getAngleRads()));
    Angle armAngle = Angle.ofBaseUnits(mArmSim.getAngleRads() - Math.PI, Radians);
    wristEncoderSim.setRawPosition(armAngle);
  }
}
