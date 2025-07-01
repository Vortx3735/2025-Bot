// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.vision.VisionConstants.robotToCamera0;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.CommandFactory;
import frc.robot.commands.autoalign.PositionPIDCommand;
import frc.robot.commands.defaultcommands.*;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.AlgaeIntake;
import frc.robot.subsystems.AlgaeWrist;
import frc.robot.subsystems.CoralIntake;
import frc.robot.subsystems.CoralWrist;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.vision.QuestNav;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOPhotonVision;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
import frc.robot.util.VorTXControllerXbox;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeCoralOnFly;
import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonCamera;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  public static LED led = new LED(0, 50); // LED port and length

  private final Vision vision;

  private double MaxSpeed =
      TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
  private double MaxAngularRate =
      RotationsPerSecond.of(0.75)
          .in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final SwerveRequest.FieldCentric drive =
      new SwerveRequest.FieldCentric()
          .withDeadband(MaxSpeed * 0.1)
          .withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
          .withDriveRequestType(
              DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

  // Replace Drive with CommandSwerveDrivetrain via TunerConstants
  public static CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

  public static final CoralIntake coralIntake =
      new CoralIntake(
          Constants.CoralConstants.CORAL_LEFTINTAKEMOTOR_ID,
          Constants.CoralConstants.CORAL_RIGHTINTAKEMOTOR_ID);

  public static final CoralWrist coralWrist =
      new CoralWrist(
          Constants.CoralConstants.CORAL_WRISTPIVOT_MOTOR_ID,
          Constants.CoralConstants.CORAL_WRISTPIVOT_ENCODER_ID);

  public static final AlgaeIntake algaeIntake =
      new AlgaeIntake(
          Constants.AlgaeConstants.LEFTINTAKE_MOTOR_ID,
          Constants.AlgaeConstants.RIGHTINTAKE_MOTOR_ID);

  public static final AlgaeWrist algaeWrist =
      new AlgaeWrist(
          Constants.AlgaeConstants.WRISTPIVOT_MOTOR_ID,
          Constants.AlgaeConstants.WRISTPIVOT_ENCODER_ID);

  public static final Elevator elevator =
      new Elevator(
          Constants.ElevatorConstants.ELEVATOR_ENCODER_ID,
          Constants.ElevatorConstants.ELEVATOR_LEFTMOTOR_ID,
          Constants.ElevatorConstants.ELEVATOR_RIGHTMOTOR_ID);

  private SwerveDriveSimulation driveSimulation = null;

  // Controller
  public static final VorTXControllerXbox driver = new VorTXControllerXbox(0);
  public static final VorTXControllerXbox operator = new VorTXControllerXbox(1);

  // Dashboard inputs
  //   private final SendableChooser<Command> autoChooser;

  public static final PhotonCamera reefCamera = new PhotonCamera("reefCamera");

  public static final QuestNav questNav = new QuestNav();

  /* Path follower */
  private final AutoFactory autoFactory;
  private final AutoRoutines autoRoutines;
  private final AutoChooser autoChooser = new AutoChooser();

  public static boolean globalIsAligned = false;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    SmartDashboard.putData(CommandScheduler.getInstance());
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drivetrain = TunerConstants.createDrivetrain();
        vision =
            new Vision(
                drivetrain, new VisionIOPhotonVision(reefCamera, VisionConstants.robotToCamera0));
        // alignToReef = new AlignToReef(drivetrain, aprilTagLayout);
        break;
      case SIM:
        // Sim robot, instantiate physics sim IO implementations

        driveSimulation =
            new SwerveDriveSimulation(
                CommandSwerveDrivetrain.mapleSimConfig, new Pose2d(3, 3, new Rotation2d()));
        SimulatedArena.getInstance().addDriveTrainSimulation(driveSimulation);
        drivetrain = TunerConstants.createDrivetrain();
        vision =
            new Vision(
                drivetrain,
                new VisionIOPhotonVisionSim(
                    reefCamera,
                    VisionConstants.camera0Name,
                    robotToCamera0,
                    driveSimulation::getSimulatedDriveTrainPose));
        // alignToReef = new AlignToReef(drivetrain, aprilTagLayout);
        break;

      default:
        // Replayed robot, disable IO implementations
        drivetrain = TunerConstants.createDrivetrain();
        vision = new Vision(drivetrain, new VisionIO() {}, new VisionIO() {});
        break;
    }

    // // Name commands
    // NamedCommands.registerCommand("moveElevatorToBottom", elevator.moveElevatorToBottom());
    // NamedCommands.registerCommand("moveWristToHP", coralWrist.moveWristToHP());
    // NamedCommands.registerCommand(
    //     "zeroElevator", new InstantCommand(() -> elevator.zeroElevator()).withTimeout(0.05));
    // NamedCommands.registerCommand("scoreL4", CommandFactory.scoreL4Command());
    // NamedCommands.registerCommand("intake", coralIntake.intakeCommand().withTimeout(1.5));
    // NamedCommands.registerCommand(
    //     "autoalign", PositionPIDCommand.generateCommand(drivetrain, reefCamera).withTimeout(4));
    // NamedCommands.registerCommand(
    //     "hpCommandandVision",
    //     CommandFactory.hpCommand()
    //         .withDeadline(
    //             PositionPIDCommand.generateCommand(drivetrain, reefCamera).withTimeout(4)));

    // autoChooser = AutoBuilder.buildAutoChooser();
    // // Set up SysId routines
    // autoChooser.addOption(
    //     "Drive SysId (Quasistatic Forward)",
    //     drivetrain.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    // autoChooser.addOption(
    //     "Drive SysId (Quasistatic Reverse)",
    //     drivetrain.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    // autoChooser.addOption(
    //     "Drive SysId (Dynamic Forward)",
    // drivetrain.sysIdDynamic(SysIdRoutine.Direction.kForward));
    // autoChooser.addOption(
    //     "Drive SysId (Dynamic Reverse)",
    // drivetrain.sysIdDynamic(SysIdRoutine.Direction.kReverse));
    // autoChooser.addOption("TestAuto", new PathPlannerAuto("TestAuto"));

    // SmartDashboard.putData("Auto Chooser", autoChooser);
    coralIntake.setDefaultCommand(new DefaultCoralIntakeCommand(coralIntake));
    coralWrist.setDefaultCommand(new DefaultCoralWristCommand(coralWrist));
    algaeIntake.setDefaultCommand(new DefaultAlgaeIntakeCommand(algaeIntake));
    algaeWrist.setDefaultCommand(new DefaultAlgaeWristCommand(algaeWrist));
    elevator.setDefaultCommand(new DefaultElevatorCommand(elevator));
    led.setDefaultCommand(new RunCommand(() -> led.VorTXGradient(), led));

    autoFactory = drivetrain.createAutoFactory();
    autoRoutines = new AutoRoutines(autoFactory);

    // autoChooser.addRoutine("Test Auto 4", autoRoutines::testAuto4);
    // autoChooser.addRoutine("CenterReef", autoRoutines::centerRoutine);
    autoChooser.addRoutine("One L4 Left", autoRoutines::oneL4Left);
    autoChooser.addRoutine("Two L4 Left", autoRoutines::twoL4Left);
    autoChooser.addRoutine("One L4 Right", autoRoutines::oneL4Right);
    autoChooser.addRoutine("Two L4 Right", autoRoutines::twoL4Right);
    autoChooser.addRoutine("One L4 Center", autoRoutines::oneL4Center);
    autoChooser.addRoutine("AlignAndScore", autoRoutines::alignAndScore);

    SmartDashboard.putData("Auto Chooser", autoChooser);
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Default command, normal field-relative drive
    drivetrain.setDefaultCommand(
        // Drivetrain will execute this command periodically
        drivetrain
            .applyRequest(
                () ->
                    driver.rb.getAsBoolean()
                        ? // if right bumper is pressed then reduce speed of robot
                        drive // coefficients can be changed to driver preferences
                            .withVelocityX(
                                -driver.getLeftY()
                                    * drivetrain.getMaxSpeed()
                                    * elevator.getElevatorCoefficient()
                                    / 6) // divide drive speed by 4
                            .withVelocityY(
                                -driver.getLeftX()
                                    * drivetrain.getMaxSpeed()
                                    * elevator.getElevatorCoefficient()
                                    / 6) // divide drive speed by 4
                            .withRotationalRate(
                                -driver.getRightX()
                                    * drivetrain.getMaxRotation()
                                    / 4) // divide turn sppeed by 3
                        : driver.lb.getAsBoolean()
                            ? drive
                                .withVelocityX(
                                    -driver.getLeftY()
                                        * drivetrain.getMaxSpeed()
                                        * elevator.getElevatorCoefficient()
                                        / 3) // Drive forward with negative Y
                                // (forward)
                                .withVelocityY(
                                    -driver.getLeftX()
                                        * drivetrain.getMaxSpeed()
                                        * elevator.getElevatorCoefficient()
                                        / 3) // Drive left with negative X (left)
                                .withRotationalRate(
                                    -driver.getRightX() * drivetrain.getMaxRotation() / 2)
                            : drive
                                .withVelocityX(
                                    -driver.getLeftY()
                                        * drivetrain.getMaxSpeed()
                                        * elevator.getElevatorCoefficient()) // Drive forward with
                                // negative Y
                                // (forward)
                                .withVelocityY(
                                    -driver.getLeftX()
                                        * drivetrain.getMaxSpeed()
                                        * elevator
                                            .getElevatorCoefficient()) // Drive left with negative X
                                // (left)
                                .withRotationalRate(
                                    -driver.getRightX()
                                        * drivetrain
                                            .getMaxRotation()) // Drive counterclockwise with
                // negative X
                // (left)
                )
            .withName("Default Drive Command"));

    // Reset gyro / odometry
    final Runnable resetGyro =
        Constants.currentMode == Constants.Mode.SIM
            ? () ->
                drivetrain.resetPose(
                    driveSimulation.getSimulatedDriveTrainPose()) // reset odometry to
            // actual robot pose
            // during simulation
            : () ->
                drivetrain.resetPose(
                    new Pose2d(
                        drivetrain.getState().Pose.getTranslation(), new Rotation2d())); // zero
    // gyro
    driver.menu.onTrue(Commands.runOnce(resetGyro, drivetrain).ignoringDisable(true));

    driver.xButton.whileTrue(PositionPIDCommand.generateCommand(drivetrain, reefCamera));

    // Beam Break
    Trigger coralDetected = new Trigger(() -> coralIntake.hasCoral());
    Trigger leftCoralDetected = new Trigger(() -> coralIntake.hasLeftCoral());
    Trigger rightCoralDetected = new Trigger(() -> coralIntake.hasRightCoral());

    Trigger coralNotDetected = coralDetected.negate();
    coralNotDetected.whileTrue(coralWrist.moveWristToHP());

    leftCoralDetected.onTrue(new WaitCommand(.05).andThen(coralIntake.stopIntakeCommand()));
    rightCoralDetected.onTrue(new WaitCommand(.05).andThen(coralIntake.stopIntakeCommand()));
    // leftCoralDetected.onTrue(coralIntake.stopIntakeCommand());
    // rightCoralDetected.onTrue(coralIntake.stopIntakeCommand());
    leftCoralDetected.onFalse(
        new WaitCommand(.4).andThen(coralIntake.stopIntakeCommand().withName("Left Trigger Stop")));
    rightCoralDetected.onFalse(
        new WaitCommand(.4)
            .andThen(coralIntake.stopIntakeCommand().withName("Right Trigger Stop")));

    operator.povLeft.whileTrue(coralWrist.moveWristUp());
    operator.povRight.whileTrue(coralWrist.moveWristDown());

    // Human Player
    operator.aButton.whileTrue(CommandFactory.hpCommand());

    // Coral Intake with Beam
    operator.lt.whileTrue(CommandFactory.hpCommandSlightlyHigher());

    // Coral Outtake
    operator.rt.whileTrue(coralIntake.outtakeCommand());

    // Algae Intake
    operator.lb.whileTrue(CommandFactory.hpCommandSlightlyLower());
    // Algae Outtake
    // operator.rb.whileTrue(new RunCommand(() -> led.funny(), led));
    operator.rb.whileTrue(new RunCommand(() -> led.VorTXBreathe(), led));

    // elevator up
    operator.povUp.whileTrue(
        new RunCommand(() -> elevator.moveElevatorUp(), elevator).withName("Move Elevator Up"));
    // elevator down
    operator.povDown.whileTrue(
        new RunCommand(() -> elevator.moveElevatorDown(), elevator).withName("Move Elevator Down"));

    operator.rs.whileTrue(
        new RunCommand(() -> algaeWrist.moveWristDown(), algaeWrist)
            .withName("Move Algae Wrist Down"));
    operator.ls.whileTrue(
        new RunCommand(() -> algaeWrist.moveWristUp(), algaeWrist).withName("Move Algae Wrist Up"));
    operator.view.onTrue(
        new InstantCommand(() -> elevator.zeroElevator(), elevator).withName("Zero Elevator"));

    // SIM CODEEEEEE
    if (Constants.currentMode == Constants.Mode.SIM) {
      // L4 placement
      driver.yButton.onTrue(
          Commands.runOnce(
              () ->
                  SimulatedArena.getInstance()
                      .addGamePieceProjectile(
                          new ReefscapeCoralOnFly(
                              driveSimulation.getSimulatedDriveTrainPose().getTranslation(),
                              new Translation2d(0.4, 0.2),
                              driveSimulation.getDriveTrainSimulatedChassisSpeedsFieldRelative(),
                              driveSimulation.getSimulatedDriveTrainPose().getRotation(),
                              Meters.of(2),
                              MetersPerSecond.of(1.5),
                              Degrees.of(-80)))));

      // L2
      operator
          .xButton
          .onTrue(CommandFactory.movetoL2Command())
          .onFalse(
              new InstantCommand(
                  () ->
                      SimulatedArena.getInstance()
                          .addGamePieceProjectile(
                              new ReefscapeCoralOnFly(
                                  driveSimulation.getSimulatedDriveTrainPose().getTranslation(),
                                  new Translation2d(0.5, 0.2),
                                  driveSimulation
                                      .getDriveTrainSimulatedChassisSpeedsFieldRelative(),
                                  driveSimulation.getSimulatedDriveTrainPose().getRotation(),
                                  Meters.of(1.35),
                                  MetersPerSecond.of(1.5),
                                  Degrees.of(-60)))));
      // L3
      operator
          .yButton
          .onTrue(CommandFactory.movetoL3Command())
          .onFalse(
              (new InstantCommand(
                  () ->
                      SimulatedArena.getInstance()
                          .addGamePieceProjectile(
                              new ReefscapeCoralOnFly(
                                  driveSimulation.getSimulatedDriveTrainPose().getTranslation(),
                                  new Translation2d(0.6, 0.2),
                                  driveSimulation
                                      .getDriveTrainSimulatedChassisSpeedsFieldRelative(),
                                  driveSimulation.getSimulatedDriveTrainPose().getRotation(),
                                  Meters.of(1.35),
                                  MetersPerSecond.of(1.5),
                                  Degrees.of(-60))))));
      // L4
      operator.bButton.onTrue(CommandFactory.ScoreL4CommandSim(driveSimulation));
    } else {
      // L2
      operator
          .xButton
          .onTrue(CommandFactory.movetoL2Command())
          .onFalse(CommandFactory.outtakeCommand());
      // L3
      operator
          .yButton
          .onTrue(CommandFactory.movetoL3Command())
          .onFalse(CommandFactory.outtakeCommand());
      // L4
      operator
          .bButton
          .onTrue(CommandFactory.movetoL4Command())
          .onFalse(CommandFactory.outtakeCommand());
    }
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.selectedCommand();
  }

  public void resetSimulationField() {
    if (Constants.currentMode != Constants.Mode.SIM) return;

    driveSimulation.setSimulationWorldPose(new Pose2d(3, 3, new Rotation2d()));
    SimulatedArena.getInstance().resetFieldForAuto();
  }

  public Pose2d getSimulationPose() {
    return driveSimulation.getSimulatedDriveTrainPose();
  }

  public void updateSimulation() {
    if (Constants.currentMode != Constants.Mode.SIM) return;

    SimulatedArena.getInstance().simulationPeriodic();
    Logger.recordOutput(
        "FieldSimulation/RobotPosition", driveSimulation.getSimulatedDriveTrainPose());
    Logger.recordOutput("RobotPose/RobotPosition", drivetrain.getState().Pose);
    Logger.recordOutput(
        "FieldSimulation/Coral", SimulatedArena.getInstance().getGamePiecesArrayByType("Coral"));
    Logger.recordOutput(
        "FieldSimulation/Algae", SimulatedArena.getInstance().getGamePiecesArrayByType("Algae"));
  }
}
