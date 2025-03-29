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

import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.RobotBase;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

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

    public static final PPHolonomicDriveController kDriveController =
        new PPHolonomicDriveController(
            new PIDConstants(5.0, 0.0, 0.0), new PIDConstants(5.0, 0.0, 0.0));
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
    public static final int CORAL_LEFT_BEAM_BREAK = 3;
    public static final int CORAL_RIGHT_BEAM_BREAK = 2;
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

  public static final class VisionConstants {
    public static final Transform3d CENTER_TO_CAMERA =
        new Transform3d(
            10.5,
            0.0,
            5.0,
            new Rotation3d(0, 10, 0)); // x = center to front, y = center to left, z = center to top
  }
}
