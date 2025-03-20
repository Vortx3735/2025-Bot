// package frc.robot.commands.autos;

// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.math.trajectory.TrapezoidProfile;
// import edu.wpi.first.wpilibj.RobotBase;
// import edu.wpi.first.wpilibj.smartdashboard.Field2d;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.Commands;
// import edu.wpi.first.wpilibj2.command.WrapperCommand;
// import frc.robot.subsystems.drive.Drive;

// public class DriveToReefPoint {
//     Field2d field;
//     private final int targetTagID;
//     private final Drive swerve;

//     public DriveToReefPoint(Drive swerve, int targetTagID) {

//         this.targetTagID = targetTagID;
//         this.swerve = swerve;

//         if (RobotBase.isSimulation()) {
//             field = new Field2d();
//             SmartDashboard.putData("Field 2", field);
//         }
//     }

//     public Command generate() {
//         Command command = Commands.none();
//         TrapezoidProfile.Constraints translationConstraints = new TrapezoidProfile.Constraints(5,
// 1);
//         command = command.andThen(
//                 new DriveToPointCommand(approachPoint.getTranslation(), translationConstraints,
// 0.4, 0.5, swerve));

//         return command;

//     }

// }
