// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveTrain;

public class Drive extends Command {

  private final DriveTrain m_drivetrain;
  private Supplier<Double> m_leftSpeed;
  private Supplier<Double> m_rightSpeed;
  
  public Drive(DriveTrain DriveTrain, Supplier<Double> leftSpeed, Supplier<Double> rightSpeed) {
    
    this.m_drivetrain = DriveTrain;
    this.m_leftSpeed = leftSpeed;
    this.m_rightSpeed = rightSpeed;
    addRequirements(m_drivetrain);
  
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("Drive Command Init");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    this.m_drivetrain.drive(this.m_rightSpeed.get(),this.m_leftSpeed.get());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
