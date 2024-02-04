// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.DifferentialFollower;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.units.MutableMeasure.mutable;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;


import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;

public class DriveTrain extends SubsystemBase {
  /** Creates a new DriveTrian. */

  TalonFX rightFrontDrive;
  TalonFX rightBackDrive;
  TalonFX leftFrontDrive;
  TalonFX leftBackDrive;

  private final DifferentialDrive m_drive = new DifferentialDrive(leftFrontDrive, rightFrontDrive);

  //Encoders
  private final Encoder m_rightEncoder = new Encoder(Constants.DeviceIds.R_Encoder_A, Constants.DeviceIds.R_Encoder_B); // Right Side
  private final Encoder m_leftEncoder = new Encoder(Constants.DeviceIds.L_Encoder_A, Constants.DeviceIds.L_Encoder_B); // Left Side
 
  //Gyro
  Pigeon2 m_gryo = new Pigeon2(Constants.DeviceIds.GYRO);

  //Odometry
  private final DifferentialDriveOdometry m_Odometry;

   // Mutable holder for unit-safe voltage values, persisted to avoid reallocation.
  private final MutableMeasure<Voltage> m_appliedVoltage = mutable(Volts.of(0));
  // Mutable holder for unit-safe linear distance values, persisted to avoid reallocation.
  private final MutableMeasure<Distance> m_distance = mutable(Meters.of(0));
  // Mutable holder for unit-safe linear velocity values, persisted to avoid reallocation.
  private final MutableMeasure<Velocity<Distance>> m_velocity = mutable(MetersPerSecond.of(0));

  // Create a new SysId routine for characterizing the drive.
  private final SysIdRoutine m_sysIdRoutine =
      new SysIdRoutine(
          // Empty config defaults to 1 volt/second ramp rate and 7 volt step voltage.
          new SysIdRoutine.Config(),
          new SysIdRoutine.Mechanism(
              // Tell SysId how to plumb the driving voltage to the motors.
              (Measure<Voltage> volts) -> {
                leftFrontDrive.setVoltage(volts.in(Volts));
                rightFrontDrive.setVoltage(volts.in(Volts));
              },
              // Tell SysId how to record a frame of data for each motor on the mechanism being
              // characterized.
              log -> {
                // Record a frame for the left motors.  Since these share an encoder, we consider
                // the entire group to be one motor.
                log.motor("drive-left")
                    .voltage(
                        m_appliedVoltage.mut_replace(
                              leftFrontDrive.get() * RobotController.getBatteryVoltage(), Volts))
                    .linearPosition(m_distance.mut_replace(m_leftEncoder.getDistance(), Meters))
                    .linearVelocity(
                        m_velocity.mut_replace(m_leftEncoder.getRate(), MetersPerSecond));
                // Record a frame for the right motors.  Since these share an encoder, we consider
                // the entire group to be one motor.
                log.motor("drive-right")
                    .voltage(
                        m_appliedVoltage.mut_replace(
                            rightFrontDrive.get() * RobotController.getBatteryVoltage(), Volts))
                    .linearPosition(m_distance.mut_replace(m_rightEncoder.getDistance(), Meters))
                    .linearVelocity(
                        m_velocity.mut_replace(m_rightEncoder.getRate(), MetersPerSecond));
              },
              // Tell SysId to make generated commands require this subsystem, suffix test state in
              // WPILog with this subsystem's name ("drive")
              this));

  
  private final ShuffleboardTab m_DataTab;

  public DriveTrain() {

  m_DataTab = Shuffleboard.getTab("Device Data Tab");

  m_DataTab.addDouble("Right Encoder Distance", () -> m_rightEncoder.getDistance());
  m_DataTab.addDouble("Right Encoder Rate", () -> m_rightEncoder.getRate());

  m_DataTab.addDouble("Left Encoder Distance", () -> m_leftEncoder.getDistance());
  m_DataTab.addDouble("Left Encoder Rate", () -> m_leftEncoder.getRate());

  m_DataTab.addDouble("Gyro Heading", () -> m_gryo.getAngle());

  rightFrontDrive = new TalonFX(Constants.DeviceIds.R_DRIVE1_ID);
  rightBackDrive = new TalonFX(Constants.DeviceIds.R_DRIVE2_ID);
  leftFrontDrive = new TalonFX(Constants.DeviceIds.L_DRIVE1_ID);
  leftBackDrive = new TalonFX(Constants.DeviceIds.L_DRIVE2_ID);


//enables safety feature 
  rightFrontDrive.setSafetyEnabled(false);
  leftFrontDrive.setSafetyEnabled(false);
  rightBackDrive.setSafetyEnabled(false);
  leftBackDrive.setSafetyEnabled(false);

  leftBackDrive.setControl(new DifferentialFollower(Constants.DeviceIds.L_DRIVE1_ID, false));
  rightBackDrive.setControl(new DifferentialFollower(Constants.DeviceIds.R_DRIVE1_ID, false));

  //sets left inverted
  rightFrontDrive.setInverted(true);
  leftFrontDrive.setInverted(false);
  
  //converts 
  m_leftEncoder.setDistancePerPulse(Constants.DrivetrainConstants.kLinearDistanceConversionFactor);
  m_rightEncoder.setDistancePerPulse(-Constants.DrivetrainConstants.kLinearDistanceConversionFactor);


  //resets gyro and encoders
  resetEncoders();
  m_gryo.reset();

  m_Odometry = new DifferentialDriveOdometry(m_gryo.getRotation2d(),
   m_leftEncoder.getDistance(), m_rightEncoder.getDistance());
  }

  public void drive(double rightSpeed, double leftspeed){

    rightFrontDrive.set(rightSpeed);
    leftFrontDrive.set(leftspeed);
      
  }

  public void resetEncoders(){
    m_leftEncoder.reset();
    m_rightEncoder.reset();
  }
  

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.dynamic(direction);
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_Odometry.update(m_gryo.getRotation2d(), m_leftEncoder.getDistance(), m_rightEncoder.getDistance());
  }
}
