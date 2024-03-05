// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;

public class RotateModulesToAngleCmd extends Command {
  double m_angleDeg;
  SwerveSubsystem m_drive;
  long m_startTime;
 
  /** Creates a new RotateModuleToAngleCmd. 
   * which is a test command to rotate a single module to a specified angle
   */
  public RotateModulesToAngleCmd( SwerveSubsystem drivetrain, double angleDeg ) {
    m_drive = drivetrain;
    m_angleDeg = angleDeg;
    addRequirements(m_drive);
  }

  // Called when the command is initially scheduled on each button press.
  @Override
  public void initialize() {
    // no need to "drive" each loop - steering PID takes care of 
    // continuous positioning after this call, at least until the
    // command finishes.
    m_drive.rotateModulesToAngle(m_angleDeg);

    // Add a baseline for the timeout that will end the command:
    m_startTime = System.currentTimeMillis();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // we could query the module looking for the angle having
    // reached the target, but since this is a test routine
    // for potentially buggy systems, instead 
    // we'll just end the command after a reasonable wait time,
    // in this case 1/2 second.
    return ((System.currentTimeMillis() - m_startTime) > 250);
  }
}
