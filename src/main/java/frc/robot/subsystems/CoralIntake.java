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
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.SensorConstants;
import java.util.function.BooleanSupplier;

import org.photonvision.targeting.TargetCorner;

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
    // coralWristConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder).pid(1.0, 0.0, 0.0);
    coralWrist.configure(coralWristConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public boolean getPositionFinished(){
    return coralPID.atSetpoint();
  }

  // public void intake() {
  //   if (rightCoralDetected()|| leftCoralDetected()){
  //     CommandScheduler.getInstance()
  //         .schedule(
  //             new SequentialCommandGroup(
  //                 new WaitCommand(0.125), new InstantCommand(() -> stopIntake(), this)));
  //             // new InstantCommand(() -> stopIntake(), this));
  //   } else {
  //     moveIntake();
  //   }
  // }
  public void intake() {
    if (rightCoralDetected()||leftCoralDetected()){
      CommandScheduler.getInstance()
          .schedule(
              new SequentialCommandGroup(
                  new WaitCommand(0.125), new InstantCommand(() -> stopIntake()
                  , this)));
    }
    else{
      moveIntake();
    }
    // if (rightCoralDetected()){
    //   CommandScheduler.getInstance()
    //       .schedule(
    //           new SequentialCommandGroup(
    //               new WaitCommand(0.125), new InstantCommand(() -> rightCoralMotor.set(0)
    //               , this)));
    //           // new InstantCommand(() -> stopIntake(), this));
    // } 
    // if(!rightCoralDetected()){
    //   moveRightIntake();
    // }
    
    // if (leftCoralDetected()) {
    //   CommandScheduler.getInstance()
    //       .schedule(
    //           new SequentialCommandGroup(
    //               new WaitCommand(0.125), new InstantCommand(() -> leftCoralMotor.set(0)
    //               , this)));      
    // }
    // if(!leftCoralDetected()){
    //   moveLeftIntake();
    // }
  }
  public void intakeAtSpeed(double speed){
    leftCoralMotor.set(speed);
    rightCoralMotor.set(speed);
  }
  public void moveIntake() {
    leftCoralMotor.set(intakeSpeed);
    rightCoralMotor.set(intakeSpeed);
  }
  public void moveLeftIntake() {
    leftCoralMotor.set(intakeSpeed);
  }
  public void moveRightIntake() {
    rightCoralMotor.set(intakeSpeed);
  }
  // returns true assuming beam break is broken
  public boolean hasCoral() {
    if (rightCoralBeamBreak.get() && leftCoralBeamBreak.get()) { 
      return false;
    }
    return true;
  }
  // returns true assuming beam break is broken
  public boolean hasLeftCoral() {
    if (rightCoralBeamBreak.get() && leftCoralBeamBreak.get()) { 
      return false;
    }
    return true;
  }
  public boolean hasRightCoral() {
    if (rightCoralBeamBreak.get() && leftCoralBeamBreak.get()) { 
      return false;
    }
    return true;
  }

  public void outtake() {
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
  public boolean moveWristToPosition(double targetPos) {
    if (Math.abs(targetPos - position) < .005) {
      stopWrist();
      return true;
    }
    coralWrist.set(
        MathUtil.clamp(
            coralPID.calculate(position, targetPos) + coralFF.calculate(position, targetPos),
            -0.2,
            0.3));
    return false;
  }
  

  public boolean moveWristToHPandIntake() {
    intake();
    return moveWristToPosition(-0.34);
  }
  public boolean moveWristToHP(){
    AlgaeIntake.resetAlgae();
    return moveWristToPosition(-0.34);
  }

  public boolean moveWristToL2() {
    return moveWristToPosition(-0.38);
  }
  public void L2Auto() {
    // double setpoint = -0.34;
    // return new RunCommand(()->moveWristToPosition(setpoint),this).until(() -> this.getPositionFinished());
    this.moveWristToPosition(-0.45);
    this.intakeAtSpeed(-0.1);
  }

  public boolean moveWristToL3() {
    return moveWristToL2();
  }

  public boolean moveWristToL4() {
    return moveWristToPosition(-0.48);
  }

  public void hold() {
    coralWrist.set(coralPID.calculate(position, position) + coralFF.calculate(position, kv));
  }

  public double getWristPosition() {
    // get wrist position
    return wristEncoder.getAbsolutePosition().getValueAsDouble();
  }

  public boolean leftCoralDetected() {
    // check if coral is detected type shi
    return !leftCoralBeamBreak.get();
  }

  public boolean rightCoralDetected() {
    // check if coral is detected type shi
    return !rightCoralBeamBreak.get();
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

    SmartDashboard.putBoolean("CoralIntake/Left Beam Break", leftCoralBeamBreak.get());
    SmartDashboard.putBoolean("CoralIntake/Right Beam Break", rightCoralBeamBreak.get());

    // Update intake speed
    intakeSpeed = SmartDashboard.getNumber("CoralIntake/intakeSpeed", intakeSpeed);

    // Publish Wrist Position
    SmartDashboard.putNumber("CoralIntake/Wrist Position", position);
  }
}
