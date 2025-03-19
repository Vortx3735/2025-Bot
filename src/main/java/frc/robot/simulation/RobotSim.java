package frc.robot.simulation;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import java.util.function.Supplier;

public class RobotSim implements Runnable {
  private final StructPublisher<Pose3d> drivetrainPub;
  public final StructPublisher<Pose3d> elevatorPub;
  private final StructPublisher<Pose3d> carriagePub;
  private final StructPublisher<Pose3d> wristPub;
  private final StructArrayPublisher<Pose3d> coralPub;
  private final DoubleEntry elevatorHeightSub;
  private final DoubleEntry carriageHeightSub;
  private final DoubleEntry wristAngleSub;
  private final Supplier<Pose2d> driveSupplier;

  private static final double INITIAL_ELEVATOR_HEIGHT = 0.5; // meters
  private static final double WRIST_OFFSET = 0.3; // meters

  private static final double MIN_HEIGHT = 0.0; // Minimum elevator height in meters
  private static final double MAX_HEIGHT = 0.74; // Maximum elevator height in meters

  // Wrist rotation constants - tune these as needed
  private static final double WRIST_MIN_ANGLE = 0; // Minimum wrist angle in radians
  private static final double WRIST_MAX_ANGLE =
      Math.PI / 2; // Maximum wrist angle in radians (90 degrees)

  public RobotSim(Supplier<Pose2d> robotPoseSupplier) {
    driveSupplier = robotPoseSupplier;
    var inst = NetworkTableInstance.getDefault();

    drivetrainPub = inst.getStructTopic("/Simulation/DrivePose", Pose3d.struct).publish();
    elevatorPub = inst.getStructTopic("/Simulation/ElevatorPose", Pose3d.struct).publish();
    carriagePub = inst.getStructTopic("/Simulation/CarriagePose", Pose3d.struct).publish();
    wristPub = inst.getStructTopic("/Simulation/WristPose", Pose3d.struct).publish();
    coralPub = inst.getStructArrayTopic("/Simulation/CoralPoses", Pose3d.struct).publish();

    elevatorHeightSub = inst.getDoubleTopic("/Simulation/ElevatorHeight").getEntry(0.0);
    carriageHeightSub = inst.getDoubleTopic("/Simulation/CarriageHeight").getEntry(0.0);
    wristAngleSub = inst.getDoubleTopic("/Simulation/WristAngle").getEntry(0.0);
    wristAngleSub.set(0.0);
  }

  @Override
  public void run() {
    var drivePose = new Pose3d(driveSupplier.get()); // Get base robot pose
    drivetrainPub.set(drivePose);

    // Read elevator height from NetworkTables (provided by Elevator subsystem)
    double elevatorHeight = elevatorHeightSub.get();
    double carriageHeight = carriageHeightSub.get();
    double wristAngle = wristAngleSub.get();

    // Base position of the elevator (as per JSON file)
    Translation3d elevatorBase = new Translation3d(0.17, 0.02, 0.08);

    // Apply movement relative to the base position
    var elevatorPose =
        new Pose3d(
            elevatorBase.plus(new Translation3d(0, 0, elevatorHeight)), // Move vertically
            Rotation3d.kZero);
    elevatorPub.set(elevatorPose);

    // Carriage - Moves with elevator AND has its own extension
    var carriagePose =
        new Pose3d(
            elevatorPose.getTranslation().plus(new Translation3d(0, 0, carriageHeight)),
            Rotation3d.kZero);
    carriagePub.set(carriagePose);

    // Wrist - Mounted on carriage with rotation around X-axis
    var wristPose =
        new Pose3d(
            carriagePose
                .getTranslation()
                .plus(new Translation3d(0.2, 0, 0.4)), // Offset from carriage
            new Rotation3d(0, wristAngle, 0)); // Rotate around X-axis
    wristPub.set(wristPose);
    // Coral - Moves with wrist
    var coralPose =
        wristPose.transformBy(new Transform3d(new Translation3d(0.1, 0, 0.1), Rotation3d.kZero));
    coralPub.set(new Pose3d[] {coralPose});
  }
}
