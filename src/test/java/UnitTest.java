import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkMax;
import frc.robot.subsystems.CoralIntake;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

public class UnitTest {
  static final double DELTA = 1e-2; // acceptable deviation range

  SparkMax motor;
  SparkMaxSim motor_Sim;
  CoralIntake m_CoralIntake;

  @BeforeEach
  void setup() {
    // assert HAL.initialize(500, 0); // initialize the HAL, crash if failed

    // // intialize coral intake
    // m_CoralIntake = new CoralIntake(1, 2, 3);
    // motor = CoralIntake.coralWrist;
    // motor_Sim = new SparkMaxSim(motor, DCMotor.getNeo550(1));
  }

  @Test
  void intakeTest() {}
}
