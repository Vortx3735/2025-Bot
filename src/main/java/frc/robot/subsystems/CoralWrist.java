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
    private final CANcoder wristEncoder;
    public static SparkMax coralWrist;

    private PIDController coralPID;
    private double ki, kp, kd;
    private double position;
    public double wristSpeedDown = -0.1;
    public double wristSpeedUp = 0.2;
    public double error;

    // sim
    private SparkMaxSim wristSim;
    private final DCMotor wristGearbox = DCMotor.getNeo550(1);
    private final int kGearRatio = 15;
    private final Mechanism2d coralArmMech = new Mechanism2d(4, 4);
    private final MechanismLigament2d coralArm;
    private final MechanismRoot2d mechBase = coralArmMech.getRoot("Arm Pivot", 2, 2);
    private final SingleJointedArmSim mArmSim = new SingleJointedArmSim(
            LinearSystemId.createDCMotorSystem(DCMotor.getNeo550(1), 0.25, kGearRatio),
            DCMotor.getNeo550(1),
            kGearRatio,
            .1,
            0,
            2 * Math.PI,
            false,
            Math.PI);

    /**
     * @param wristId        The CAN ID of the wrist motor.
     * @param wristEncoderId The CAN ID of the wrist encoder.
     */
    public CoralWrist(int wristId, int wristEncoderId) {
        SparkMaxConfig coralWristConfig = new SparkMaxConfig();
        coralWrist = new SparkMax(wristId, MotorType.kBrushless);
        wristEncoder = new CANcoder(wristEncoderId);
        kp = 3;
        coralPID = new PIDController(kp, ki, kd);

        // Configure wrist motor settings
        coralWristConfig.inverted(false).idleMode(IdleMode.kBrake);
        coralWrist.configure(
                coralWristConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // sim
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

    /**
     * @param speed
     */
    public void moveWrist(double speed) {
        coralWrist.set(speed);
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
        // coralWrist.set(coralPID.calculate(position, targetPos) +
        // coralFF.calculate(position, kv));
        coralWrist.set(coralPID.calculate(position, targetPos));
    }

    public double getWristPosition() {
        // get wrist position
        return wristEncoder.getAbsolutePosition().getValueAsDouble();
    }

    public Boolean atSetpoint(double targetPos) {
        if (Math.abs(targetPos - position) < .02) {
            return true;
        }
        return false;
    }

    public void stopWrist() {
        // stop wrist
        coralWrist.set(0);
    }

    public void publishInitialValues() {
        SmartDashboard.putNumber("CoralIntake/Wrist Up Speed", wristSpeedUp);
        SmartDashboard.putNumber("CoralIntake/Wrist Down Speed", wristSpeedDown);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run

        // Update wrist position
        position = getWristPosition();

        wristSpeedUp = SmartDashboard.getNumber("CoralIntake/Wrist Up Speed", wristSpeedUp);
        wristSpeedDown = SmartDashboard.getNumber("CoralIntake/Wrist Down Speed", wristSpeedDown);
    }

    @Override
    public void simulationPeriodic() {
        SmartDashboard.putData("CoralIntake/CoralIntakeVisualizer", coralArmMech);

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
        Angle coralArmAngle = Angle.ofBaseUnits(mArmSim.getAngleRads(), Radians);
        wristEncoderSim.setRawPosition(coralArmAngle);
    }
}
