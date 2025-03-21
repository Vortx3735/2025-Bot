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

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static frc.robot.subsystems.vision.VisionConstants.aprilTagLayout;
import static frc.robot.subsystems.vision.VisionConstants.robotToCamera0;
import static frc.robot.subsystems.vision.VisionConstants.robotToCamera1;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
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
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.CommandFactory;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.autos.AlignToReef;
import frc.robot.commands.defaultcommands.*;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.AlgaeIntake;
import frc.robot.subsystems.AlgaeWrist;
import frc.robot.subsystems.CoralIntake;
import frc.robot.subsystems.CoralWrist;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.GyroIOSim;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOTalonFXReal;
import frc.robot.subsystems.drive.ModuleIOTalonFXSim;
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
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  public final Drive drive;
  private final Vision vision;

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
  private final VorTXControllerXbox driver = new VorTXControllerXbox(0);
  private final VorTXControllerXbox operator = new VorTXControllerXbox(1);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  private AlignToReef alignToReef;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    SmartDashboard.putData(CommandScheduler.getInstance());
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalonFXReal(TunerConstants.FrontLeft),
                new ModuleIOTalonFXReal(TunerConstants.FrontRight),
                new ModuleIOTalonFXReal(TunerConstants.BackLeft),
                new ModuleIOTalonFXReal(TunerConstants.BackRight),
                (pose) -> {});
        vision =
            new Vision(
                drive,
                new VisionIOPhotonVision(
                    VisionConstants.camera0Name, VisionConstants.robotToCamera0),
                new VisionIOPhotonVision(
                    VisionConstants.camera1Name, VisionConstants.robotToCamera1));
        alignToReef = new AlignToReef(drive, aprilTagLayout);
        break;
      case SIM:
        // Sim robot, instantiate physics sim IO implementations

        driveSimulation =
            new SwerveDriveSimulation(Drive.mapleSimConfig, new Pose2d(3, 3, new Rotation2d()));
        SimulatedArena.getInstance().addDriveTrainSimulation(driveSimulation);
        drive =
            new Drive(
                new GyroIOSim(driveSimulation.getGyroSimulation()),
                new ModuleIOTalonFXSim(TunerConstants.FrontLeft, driveSimulation.getModules()[0]),
                new ModuleIOTalonFXSim(TunerConstants.FrontRight, driveSimulation.getModules()[1]),
                new ModuleIOTalonFXSim(TunerConstants.BackLeft, driveSimulation.getModules()[2]),
                new ModuleIOTalonFXSim(TunerConstants.BackRight, driveSimulation.getModules()[3]),
                driveSimulation::setSimulationWorldPose);
        vision =
            new Vision(
                drive,
                new VisionIOPhotonVisionSim(
                    VisionConstants.camera0Name,
                    robotToCamera0,
                    driveSimulation::getSimulatedDriveTrainPose),
                new VisionIOPhotonVisionSim(
                    VisionConstants.camera1Name,
                    robotToCamera1,
                    driveSimulation::getSimulatedDriveTrainPose));
        alignToReef = new AlignToReef(drive, aprilTagLayout);
        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                (pose) -> {});
        vision = new Vision(drive, new VisionIO() {}, new VisionIO() {});
        break;
    }
    // Name commands
    NamedCommands.registerCommand("moveElevatorToBottom", elevator.moveElevatorToBottom());
    NamedCommands.registerCommand(
        "zeroElevator", new InstantCommand(() -> elevator.zeroElevator()).withTimeout(0.05));
    NamedCommands.registerCommand("moveWristToHP", coralWrist.moveWristToHP());
    NamedCommands.registerCommand("intakeCoral", coralIntake.intakeCommand().withTimeout(0.2));
    NamedCommands.registerCommand("scoreL4", CommandFactory.ScoreL4CommandSim(driveSimulation));

    // SYS ID ROUTINES
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up SysId routines
    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption("TestAuto", new PathPlannerAuto("TestAuto"));

    SmartDashboard.putData("Auto Chooser", autoChooser.getSendableChooser());
    coralIntake.setDefaultCommand(new DefaultCoralIntakeCommand(coralIntake));
    coralWrist.setDefaultCommand(new DefaultCoralWristCommand(coralWrist));
    algaeIntake.setDefaultCommand(new DefaultAlgaeIntakeCommand(algaeIntake));
    algaeWrist.setDefaultCommand(new DefaultAlgaeWristCommand(algaeWrist));
    elevator.setDefaultCommand(new DefaultElevatorCommand(elevator));

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
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive, () -> -driver.getLeftY(), () -> -driver.getLeftX(), () -> -driver.getRightX()));

    // Reset gyro / odometry
    final Runnable resetGyro =
        Constants.currentMode == Constants.Mode.SIM
            ? () -> drive.setPose(driveSimulation.getSimulatedDriveTrainPose()) // reset odometry to
            // actual robot pose
            // during simulation
            : () ->
                drive.setPose(
                    new Pose2d(drive.getPose().getTranslation(), new Rotation2d())); // zero
    // gyro
    driver.menu.onTrue(Commands.runOnce(resetGyro, drive).ignoringDisable(true));

    driver.xButton.whileTrue(alignToReef.generateCommand());

    // Beam Break
    Trigger coralDetected = new Trigger(() -> coralIntake.hasCoral());
    Trigger leftCoralDetected = new Trigger(() -> coralIntake.hasLeftCoral());
    Trigger rightCoralDetected = new Trigger(() -> coralIntake.hasRightCoral());

    Trigger coralNotDetected = coralDetected.negate();
    coralNotDetected.whileTrue(coralWrist.moveWristToHP());

    leftCoralDetected.onTrue(new WaitCommand(.2).andThen(coralIntake.stopIntakeCommand()));
    rightCoralDetected.onTrue(new WaitCommand(.2).andThen(coralIntake.stopIntakeCommand()));
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
    operator.lt.whileTrue(
        Commands.parallel(coralIntake.intakeCommand(), elevator.moveElevatorToHPHigher()));

    // Coral Outtake
    operator.rt.whileTrue(coralIntake.outtakeCommand());

    // Algae Intake
    operator.lb.whileTrue(algaeIntake.intakeCommand());
    // Algae Outtake
    operator.rb.whileTrue(algaeIntake.outtakeCommand());

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
      operator.bButton.onTrue(CommandFactory.scoreL4Command());
    }
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
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
    Logger.recordOutput(
        "FieldSimulation/Coral", SimulatedArena.getInstance().getGamePiecesArrayByType("Coral"));
    Logger.recordOutput(
        "FieldSimulation/Algae", SimulatedArena.getInstance().getGamePiecesArrayByType("Algae"));
  }
}
