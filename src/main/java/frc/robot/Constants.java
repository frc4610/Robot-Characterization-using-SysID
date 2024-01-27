// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.XboxController;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class DeviceIds{
    public static final int R_DRIVE1_ID = 0;
    public static final int R_DRIVE2_ID = 1;
    public static final int L_DRIVE1_ID = 2;
    public static final int L_DRIVE2_ID = 3;

    public static final int GYRO = 4;

    //Right Encoder
    public static final int  R_Encoder_A = 0;
    public static final int  R_Encoder_B = 1;

    //Left Encoder
    public static final int  L_Encoder_A = 2;
    public static final int  L_Encoder_B = 3;
  }

  public static class OperatorConstants {
    public static final int DriverControlPort = 0;
    public static final int LeftYaxis = XboxController.Axis.kLeftY.value;
    public static final int RightYaxis = XboxController.Axis.kRightY.value;
  
  }

  public static class DrivetrainConstants{

    public static final double kLinearDistanceConversionFactor = Units.inchesToMeters(6)*Math.PI;
  }
}
