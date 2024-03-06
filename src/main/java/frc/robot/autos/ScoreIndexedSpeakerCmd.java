// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.MasterArmSubsystem;

public class ScoreIndexedSpeakerCmd extends Command {
  private MasterArmSubsystem m_noteConductor;

  /** Creates a new ScoreIndexedSpeakerCmd. */
  public ScoreIndexedSpeakerCmd(MasterArmSubsystem noteConductor) {
    m_noteConductor = noteConductor;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_noteConductor);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_noteConductor.prepForIndexedSpeakerScore();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_noteConductor.isReadyToScoreSpeaker()) {
      m_noteConductor.scoreNote();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_noteConductor.isIdle();
  }
}