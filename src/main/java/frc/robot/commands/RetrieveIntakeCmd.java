// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.MasterArmSubsystem;

public class RetrieveIntakeCmd extends Command {
  private MasterArmSubsystem  m_noteConductor;

  /** Creates a new RetrieveIntakeCmd. */
  public RetrieveIntakeCmd(MasterArmSubsystem noteConductor) {
    m_noteConductor = noteConductor;

    addRequirements(noteConductor);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // This is an autonomous only routine - no need to simulateNoteAcquired(), just force 
    // the retrieval of the intake whether a note has been picked up or not.
    m_noteConductor.retrieveNote();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_noteConductor.isIntakeRetrieved();
  }
}
