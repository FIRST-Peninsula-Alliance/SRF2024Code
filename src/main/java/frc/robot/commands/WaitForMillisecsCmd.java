// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;

public class WaitForMillisecsCmd extends Command {
SwerveSubsystem m_swerve;
long m_startTimeMs;
long m_durationMs;

  /** Creates a new WaitForDriveCmd. */
  public WaitForMillisecsCmd(SwerveSubsystem swerve, long milliseconds) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_swerve = swerve;
    m_durationMs = milliseconds;
    addRequirements(swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_startTimeMs = System.currentTimeMillis();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return ((System.currentTimeMillis() - m_startTimeMs) > m_durationMs);
  }
}
