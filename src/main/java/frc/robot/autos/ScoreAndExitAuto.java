// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import frc.robot.commands.ScoreIndexedSpeakerCmd;
import frc.robot.subsystems.MasterArmSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class ScoreAndExitAuto extends SequentialCommandGroup {
  private MasterArmSubsystem m_noteConductor;
  private SwerveSubsystem m_swerveDrive;

  /** Creates a new ScoreAndMove. */
  public ScoreAndExitAuto(MasterArmSubsystem noteConductor, 
                          SwerveSubsystem swerveDrive) {
    m_noteConductor = noteConductor;
    m_swerveDrive = swerveDrive;
    
    addCommands(
                new ScoreIndexedSpeakerCmd(m_noteConductor),
                new JustExitCmd(m_swerveDrive),
                new InstantCommand(()-> m_swerveDrive.stop())
               );
    }
}

