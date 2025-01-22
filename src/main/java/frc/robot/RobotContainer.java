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
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.util.TunerConstants;
import frc.robot.util.VorTXControllerXbox;

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

  private final Telemetry logger = new Telemetry(MaxSpeed);

  private final VorTXControllerXbox joystick = new VorTXControllerXbox(0);

  private final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

  /* Path follower */
  private final AutoFactory autoFactory;
  private final AutoRoutines autoRoutines;
  private final AutoChooser autoChooser = new AutoChooser();

  public RobotContainer() {
    autoFactory = drivetrain.createAutoFactory();
    autoRoutines = new AutoRoutines(autoFactory);

    autoChooser.addRoutine("Test Auto 1", autoRoutines::testAuto1);
    SmartDashboard.putData("Auto Chooser", autoChooser);

    configureBindings();
    configureNetworkTables();
  }

  private void configureNetworkTables() {
    logger.configureNetworkTables();
  }

  public void updateNetworkTables() {
    double[] robotPos = drivetrain.getRobotPosition();
    double[] encoderPos = drivetrain.getEncoderPositions();
    logger.updateNetworkTables(robotPos, encoderPos);
  }

  private void configureBindings() {
    // Note that X is defined as forward according to WPILib convention,
    // and Y is defined as to the left according to WPILib convention.
    drivetrain.setDefaultCommand(
        // Drivetrain will execute thigradlew.bat :spotlessApplys command periodically
        // Drivetrain will execute thigradlew.bat :spotlessApplys command periodically
        drivetrain.applyRequest(
            () ->
                joystick.rightBumper().getAsBoolean() == true
                    ? // if right bumper is pressed then reduce speed of robot
                    drive // coefficients can be changed to driver preferences
                        .withVelocityX(
                            -joystick.getLeftY() * MaxSpeed / 4) // divide drive speed by 4
                        .withVelocityY(
                            -joystick.getLeftX() * MaxSpeed / 4) // divide drive speed by 4
                        .withRotationalRate(
                            -joystick.getRightX() * MaxAngularRate / 3) // divide turn sppeed by 3
                    : drive
                        .withVelocityX(
                            -joystick.getLeftY()
                                * MaxSpeed) // Drive forward with negative Y (forward)
                        .withVelocityY(
                            -joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                        .withRotationalRate(
                            -joystick.getRightX()
                                * MaxAngularRate) // Drive counterclockwise with negative X (left)
            ));

    joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
    joystick
        .b()
        .whileTrue(
            drivetrain.applyRequest(
                () ->
                    point.withModuleDirection(
                        new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))));

    // Run SysId routines when holding back/start and X/Y.
    // Note that each routine should be run exactly once in a single log.
    joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
    joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
    joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
    joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

    // reset the field-centric heading on left bumper press
    joystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));
    drivetrain.registerTelemetry(logger::telemeterize);
  }

  public Command getAutonomousCommand() {
    return autoChooser.selectedCommand();
  }
}
