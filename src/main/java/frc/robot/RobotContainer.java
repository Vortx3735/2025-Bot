// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.*;
import frc.robot.commands.defaultcommands.*;
import frc.robot.subsystems.AlgaeIntake;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.CoralIntake;
import frc.robot.subsystems.CoralWrist;
import frc.robot.subsystems.Elevator;
import frc.robot.util.TunerConstants;
import frc.robot.util.VorTXControllerXbox;
import org.photonvision.PhotonCamera;

public class RobotContainer {
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
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

  public final Telemetry logger = new Telemetry(MaxSpeed, MaxAngularRate);

  private final VorTXControllerXbox driver = new VorTXControllerXbox(0);
  private final VorTXControllerXbox operator = new VorTXControllerXbox(1);

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
          Constants.AlgaeConstants.RIGHTINTAKE_MOTOR_ID,
          Constants.AlgaeConstants.WRISTPIVOT_MOTOR_ID,
          Constants.AlgaeConstants.WRISTPIVOT_ENCODER_ID);

  public static final Elevator elevator =
      new Elevator(
          Constants.ElevatorConstants.ELEVATOR_ENCODER_ID,
          Constants.ElevatorConstants.ELEVATOR_LEFTMOTOR_ID,
          Constants.ElevatorConstants.ELEVATOR_RIGHTMOTOR_ID);

  /* Path follower */
  private final AutoFactory autoFactory;
  private final AutoRoutines autoRoutines;
  private final AutoChooser autoChooser = new AutoChooser();

  public static PhotonCamera reefCamera = new PhotonCamera("reefCamera");
  public static PhotonCamera hpCamera = new PhotonCamera("hpCamera");

  private AutoAlignCommand autoAlignHP = new AutoAlignCommand(drivetrain, hpCamera);
  private AutoAlignCommand autoAlignReef = new AutoAlignCommand(drivetrain, reefCamera);

  public RobotContainer() {
    configureBindings();
    configureNetworkTables();
    // Auton

    autoFactory = drivetrain.createAutoFactory();
    autoRoutines = new AutoRoutines(autoFactory);

    // autoChooser.addRoutine("Test Auto 4", autoRoutines::testAuto4);
    // autoChooser.addRoutine("CenterReef", autoRoutines::centerRoutine);
    autoChooser.addRoutine("VisionAuton", autoRoutines::visionAutoRoutine);

    SmartDashboard.putData("Auto Chooser", autoChooser);

    elevator.publishInitialValues();
    coralIntake.publishInitialValues();
    algaeIntake.publishInitialValues();

    // default commands
    coralIntake.setDefaultCommand(new DefaultCoralIntakeCommand(coralIntake));
    coralWrist.setDefaultCommand(new DefaultCoralWristCommand(coralWrist));
    algaeIntake.setDefaultCommand(new DefaultAlgaeIntakeCommand(algaeIntake));
    elevator.setDefaultCommand(new DefaultElevatorCommand(elevator));

    elevator.configureTalonFX();
  }

  private void configureNetworkTables() {
    logger.initSwerveTable(drivetrain.getState());
  }

  public void updateNetworkTables() {
    drivetrain.registerTelemetry(logger::telemeterize);
  }

  private void configureBindings() {
    // DRIVER
    // Note that X is defined as forward according to WPILib convention,
    // and Y is defined as to the left according to WPILib convention.
    drivetrain.setDefaultCommand(
        // Drivetrain will execute this command periodically
        drivetrain
            .applyRequest(
                () ->
                    driver.rb.getAsBoolean() == true
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
                        : driver.lb.getAsBoolean() == true
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

    driver.aButton.whileTrue(drivetrain.applyRequest(() -> brake));
    driver.bButton.whileTrue(
        drivetrain.applyRequest(
            () ->
                point.withModuleDirection(new Rotation2d(-driver.getLeftY(), -driver.getLeftX()))));

    driver.xButton.whileTrue(autoAlignReef);

    // test whiletrue first then this
    // driver.xButton.onTrue(
    //     Commands.sequence(
    //         autoAlignL2.until(autoAlignL2.isAligned()),
    //         Commands.parallel(
    //             new RunCommand(() -> coralIntake.moveWristToL2(), coralIntake),
    //             new RunCommand(() -> elevator.moveElevatorToL2(), elevator),
    //             new RunCommand(() -> algaeIntake.stowWrist(), algaeIntake)
    //         )
    //     )
    // );

    // Run SysId routines when holding back/start and X/Y.
    // Note that each routine should be run exactly once in a single log.
    driver.view.and(driver.yButton).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
    driver.view.and(driver.xButton).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
    driver.menu.and(driver.yButton).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
    driver.menu.and(driver.xButton).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

    // reset the field-centric heading on menu button
    driver.menu.onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

    // temp button binding for algae wrist
    driver.povUp.whileTrue(
        new RunCommand(() -> algaeIntake.stowWrist(), algaeIntake).withName("Algae Stow Wrist"));
    driver.povDown.whileTrue(
        new RunCommand(() -> algaeIntake.unstowWrist(), algaeIntake)
            .withName("Algae Unstow Wrist"));

    // OPERATOR
    operator.povLeft.whileTrue(
        new RunCommand(() -> coralWrist.moveWristUp(), coralWrist).withName("Coral Wrist Up"));
    operator.povRight.whileTrue(
        new RunCommand(() -> coralWrist.moveWristDown(), coralWrist).withName("Coral Wrist Down"));

    // Human Player
    operator.aButton.whileTrue(CommandFactory.hpCommand());
    // L2
    operator.xButton.onTrue(CommandFactory.scoreL2Command());
    // L3
    operator.yButton.onTrue(CommandFactory.scoreL3Command());
    // L4
    operator.bButton.onTrue(CommandFactory.scoreL4Command());

    // Coral Intake with Beam
    operator.lt.whileTrue(coralIntake.intakeCommand().withName("Coral Intake"));

    // Coral Outtake
    operator.rt.whileTrue(coralIntake.outtakeCommand().withName("Coral Outtake"));

    // Algae Intake
    operator.lb.whileTrue(
        new RunCommand(() -> algaeIntake.intake(), algaeIntake).withName("Algae Intake"));
    // Algae Outtake
    operator.rb.whileTrue(
        new RunCommand(() -> algaeIntake.outtake(), algaeIntake).withName("Algae Outtake"));

    // elevator up
    operator.povUp.whileTrue(
        new RunCommand(() -> elevator.moveElevatorUp(), elevator).withName("Move Elevator Up"));
    // elevator down
    operator.povDown.whileTrue(
        new RunCommand(() -> elevator.moveElevatorDown(), elevator).withName("Move Elevator Down"));

    operator.rs.whileTrue(
        new RunCommand(() -> algaeIntake.moveWristDown(), algaeIntake)
            .withName("Move Algae Wrist Down"));
    operator.ls.whileTrue(
        new RunCommand(() -> algaeIntake.moveWristUp(), algaeIntake)
            .withName("Move Algae Wrist Up"));
    operator.view.onTrue(
        new InstantCommand(() -> elevator.zeroElevator(), elevator).withName("Zero Elevator"));
  }

  public Command getAutonomousCommand() {
    return autoChooser.selectedCommand();
  }
}
