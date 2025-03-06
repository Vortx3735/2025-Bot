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
  public static final class DriveTrainConstants { // Id's are all placeholders
    public static final int FRONTLEFT_DRIVE_ID = 1;
    public static final int FRONTLEFT_TURN_ID = 2;
    public static final int FRONTLEFT_CANCODER = 3;

    public static final int FRONTRIGHT_DRIVE_ID = 4;
    public static final int FRONTRIGHT_TURN_ID = 5;
    public static final int FRONTRIGHT_CANCODER = 6;

    public static final int BACKLEFT_DRIVE_ID = 7;
    public static final int BACKLEFT_TURN_ID = 8;
    public static final int BACKLEFT_CANCODER = 9;

    public static final int BACKRIGHT_DRIVE_ID = 10;
    public static final int BACKRIGHT_TURN_ID = 11;
    public static final int BACKRIGHT_CANCODER = 12;

    public static final int PIGEON_ID = 23;
  }

  public static final class AlgaeConstants {
    public static final int LEFTINTAKE_MOTOR_ID = 13;
    public static final int RIGHTINTAKE_MOTOR_ID = 14;
    public static final int WRISTPIVOT_MOTOR_ID = 15;
    public static final int WRISTPIVOT_ENCODER_ID = 16; // cancoder
  }

  public static final class CoralConstants {
    public static final int CORAL_LEFTINTAKEMOTOR_ID = 17;
    public static final int CORAL_RIGHTINTAKEMOTOR_ID = 18;
    public static final int CORAL_WRISTPIVOT_MOTOR_ID = 19;
    public static final int CORAL_WRISTPIVOT_ENCODER_ID = 20; // cancoder
    public static final double WRIST_LOWER_LIMIT = 0.0; // Define lower bound in radians
    public static final double WRIST_UPPER_LIMIT = Math.PI; // Define upper bound in radians
  }

  public static final class ElevatorConstants {
    public static final int ELEVATOR_LEFTMOTOR_ID = 24;
    public static final int ELEVATOR_RIGHTMOTOR_ID = 25;
    public static final int ELEVATOR_ENCODER_ID = 26; // cancoder

    public static final double LEVEL_1 = 1.0;
    public static final double LEVEL_2 = 2.0;
    public static final double LEVEL_3 = 3.0;
    public static final double LEVEL_4 = 4.0;
  }

  public static final class SensorConstants {
    public static final int CORAL_LEFT_BEAM_BREAK = 2;
    public static final int CORAL_RIGHT_BEAM_BREAK = 0;
    public static final int ALGAE_BEAM_BREAK = 5;
  }

  public static final class PDH {
    public static final int BACK_RIGHT_DRIVE_PDH = 0;
    public static final int BACK_LEFT_DRIVE_PDH = 1;
    public static final int ALGAE_PIVOT_PDH = 4;
    public static final int ALGAE_RIGHT_PDH = 5;
    public static final int ALGAE_LEFT_PDH = 6;
    public static final int CORAL_PIVOT_PDH = 7;
    public static final int CORAL_RIGHT_PDH = 8;
    public static final int CORAL_LEFT_PDH = 9;
    public static final int ELEVATOR_LEFT_PDH = 12;
    public static final int ELEVATOR_RIGHT_PDH = 13;
    public static final int BACK_LEFT_TURN_PDH = 14;
    public static final int BACK_RIGHT_TURN_PDH = 15;
    public static final int FRONT_LEFT_TURN_PDH = 16;
    public static final int FRONT_RIGHT_TURN_PDH = 17;
    public static final int FRONT_LEFT_DRIVE_PDH = 18;
    public static final int FRONT_RIGHT_DRIVE_PDH = 19;
  }
}
