// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  // these are the constants for the Motor Id's
  public static final class Drive_train { // Id's are all placeholders
    public static final int Left_Drive_ID = 1;
    public static final int Right_Drive_ID = 2;
  }

  public static final class Algae_Intake {
    public static final int Intake_Motor_ID = 3;
    public static final int Wrist_Motor_ID = 4;
  }

  public static final class Coral_Intake {
    public static final int Intake_Motor_ID = 5;
    public static final int Wrist_Motor_ID = 6;
  }

  public static final class Climber {
    public static final int Climber_Motor_ID = 7;
  }

  public static final class elevator {
    public static final int Elevator_Motor_ID = 8;
  }

  // These are for the port mappings
  public static final class Sensors {
    public static final int Beam_Break_Sensors_ID = 9;
    public static final int Gyro_Poter_ID = 10;
    public static final int Acclerometer_port_ID = 11;
  }

  public static final class Controller {
    public static final int Controller_ID =
        12; // let me know if the controller id is for all keybinds or just for the...controller
  }

  public static final class Other_Peripherals {
    public static final int Camera_Sensor_ID = 13;
    public static final int April_Tags_Allingment_ID = 14;
  }

  // these ID's are for Subsystem SetPoints
  public static final class Elevator_Setpoints {
    public static final int Human_Player_position_setpoint = 15; // Placeholder
    public static final int L1_HEIGHT_ID = 16; // Placeholder
    public static final int L2_HEIGHT_ID = 17; // Placeholder
    public static final int L3_HEIGHT_ID = 18; // Placeholder
    public static final int L4_HEIGHT_ID = 19;
  }

  public static final class Wrist_Setpoints {
    public static final int Coral_Intake_ID = 20; // placeholder
    public static final int Move_To_ElevatorPostion_ID = 21;
    public static final int Move_To_HumanPlayer_ID = 22;
  }

  public static final class Climber_Setpoint {
    public static final int Climber_Height_ID = 23;
  }

  // these ID's are for speed limits
  public static final class Speed_Limits {
    public static final int SpeedLimit_DriveTrainID = 24;
    public static final int RotationLimit_ID = 25;
  }

  // these ID's are for autonamous constants
  public static final class autonmanus_constants {
    public static final int Path_Parameters_ID = 26;
    public static final int April_Tags_Allingment_ID = 27;
    // Below are Auton set point
    public static final int Start_position_ID = 28;
    public static final int setpointIDS = 29;
  }

  // these are Miscellaneous ID's
  public static final class Deadband {
    public static final int Deadband_ID = 30;
  }

  public static final class PID {
    public static double elevator_ID = 31;
    public static double wrist_position_ID = 32;
    public static double auton_allingment_ID = 33;
  }

  // This class is for the physics based safety thresholds SAFETY FIRST
  public static final class Safety_Thresholds {
    public static final int Max_Speed_ID = 34;
    public static final int Max_Acceleration_ID = 35;
    public static final int Max_Tilt_ID = 36;
  }
}
