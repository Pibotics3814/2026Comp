// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import swervelib.math.Matter;


/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants { // Controller imputs buttons
    public static final int kDriverControllerPort = 3;
    public static final int kOperatorControllerPort1 = 1;
    public static final int kOperatorControllerPort2 = 2;
    public static final double DEADBAND = 0.1;
  }
    public static class ModuleConstants{
    public static final double kWheelDiameterMeters = 0.1016; // 4 inches in meters
    public static final double kDriveMotorGearRatio = 6.86; // Example gear ratio
    public static final double kSteerMotorGearRatio = 12.8; // Example gear ratio
    public static final double kDriveEncoderPositionFactor = (kWheelDiameterMeters * Math.PI) / kDriveMotorGearRatio; // meters per motor rotation
    public static final double kDriveEncoderVelocityFactor = kDriveEncoderPositionFactor / 60.0; // meters per second per RPM
    public static final double kSteerEncoderPositionFactor = 360.0 / kSteerMotorGearRatio; // degrees per motor rotation
    public static final double kSteerEncoderVelocityFactor = kSteerEncoderPositionFactor / 60.0; // degrees per second per RPM

    public static final double kSteerP = 0.5; // Example P value
    public static final double kSteerI = 0.0; // Example I value
    public static final double kSteerD = 0.0; // Example D value
    public static final double kSteerFF = 0.0; // Example Feedforward value
    public static final double kDriveP = 0.1; // Example P value
    public static final double kDriveI = 0.0; // Example I value
    public static final double kDriveD = 0.0; // Example D value
    public static final double kDriveFF = 0.0; // Example Feedforward value
    }
    public static final double MAX_SPEED = 0; // meters per second
    public static final double LOOP_TIME = 0;
    public static final double ROBOT_MASS = 0; // kg


    public static final Matter CHASSIS    = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
   
  
   //IDS all in JSON files
}