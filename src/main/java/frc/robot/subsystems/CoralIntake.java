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
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SensorConstants;
import frc.robot.commands.CoralHPCommand;

public class CoralIntake extends SubsystemBase {

  public static SparkMax leftCoralMotor;
  public static SparkMax rightCoralMotor;
  public static SparkMax coralWrist;

  private final CANcoder wristEncoder;
  private double position;

  private PIDController coralPID;
  private double ki, kp, kd;

  private ArmFeedforward coralFF;
  private double kg, ks, kv;

  public double wristSpeedDown = -0.1;
  public double wristSpeedUp = 0.2;
  public double error;
  private double intakeSpeed = 0.25;

  public DigitalInput leftCoralBeamBreak = new DigitalInput(SensorConstants.CORAL_LEFT_BEAM_BREAK);
  public DigitalInput rightCoralBeamBreak =
      new DigitalInput(SensorConstants.CORAL_RIGHT_BEAM_BREAK);

  private final int kGearRatio = 60;
  private final Mechanism2d coralArmMech = new Mechanism2d(4, 4);
  private final MechanismLigament2d coralArm;
  private final MechanismRoot2d mechBase = coralArmMech.getRoot("Arm Pivot", 2, 2);
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
   * @param leftMotorId The CAN ID of the left intake motor.
   * @param rightMotorId The CAN ID of the right intake motor.
   * @param wristId The CAN ID of the wrist motor.
   * @param wristEncoderId The CAN ID of the wrist encoder.
   */
  public CoralIntake(int leftMotorId, int rightMotorId, int wristId, int wristEncoderId) {
    // Motor configurations
    SparkMaxConfig coralMotorConfig = new SparkMaxConfig();
    SparkMaxConfig coralWristConfig = new SparkMaxConfig();

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

    // Initialize wrist motor and encoder
    coralWrist = new SparkMax(wristId, MotorType.kBrushless);
    wristEncoder = new CANcoder(wristEncoderId);
    kg = 0.05;
    kp = 3;
    coralFF = new ArmFeedforward(ks, kg, kv);
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
    return new CoralHPCommand(position, -intakeSpeed - .1).withName("Coral Outtake Command");
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
    leftCoralMotor.set(-intakeSpeed - 0.1);
    rightCoralMotor.set(-intakeSpeed - 0.1);
  }

  public void stopIntake() {
    // stop motor
    leftCoralMotor.set(0);
    rightCoralMotor.set(0);
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
            () -> System.out.println("INTIALIZED"),
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

  public Command coralHPCommand() {
    return new CoralHPCommand(-0.34, intakeSpeed).withName("Coral HP Command");
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
    SmartDashboard.putNumber("CoralIntake/intakeSpeed", intakeSpeed);

    SmartDashboard.putNumber("CoralIntake/Wrist Up Speed", wristSpeedUp);
    SmartDashboard.putNumber("CoralIntake/Wrist Down Speed", wristSpeedDown);
  }

  @Override
  public void periodic() {
    // Update wrist position
    position = getWristPosition();

    wristSpeedUp = SmartDashboard.getNumber("CoralIntake/Wrist Up Speed", wristSpeedUp);
    wristSpeedDown = SmartDashboard.getNumber("CoralIntake/Wrist Down Speed", wristSpeedDown);

    SmartDashboard.putBoolean("CoralIntake/Left Coral", hasLeftCoral());
    SmartDashboard.putBoolean("CoralIntake/Right Coral", hasRightCoral());

    // Update intake speed
    intakeSpeed = SmartDashboard.getNumber("CoralIntake/intakeSpeed", intakeSpeed);

    // Publish Wrist Position
    SmartDashboard.putNumber("CoralIntake/Wrist Position", position);
    SmartDashboard.putNumber("CoralIntake/Wrist Position", position);
    SmartDashboard.putBoolean(
        "CoralIntake/Wristbooleanfinished", getPositionFinished(-0.38, position));

    SmartDashboard.putData("CoralIntake/CoralIntakeVisualizer", coralArmMech);
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
    Angle coralArmAngle = Angle.ofBaseUnits(mArmSim.getAngleRads() - Math.PI, Radians);
    wristEncoderSim.setRawPosition(coralArmAngle);
  }
}
