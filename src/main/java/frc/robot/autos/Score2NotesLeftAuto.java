// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import frc.robot.subsystems.MasterArmSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.autos.TrajectoryPaths.MoveLeftNoteToLeftIndexedShotCmd;
import frc.robot.autos.TrajectoryPaths.MoveSpkrLeftToLeftNoteCmd;
import frc.robot.commands.DeployIntakeCmd;
import frc.robot.commands.RetrieveIntakeCmd;
import frc.robot.commands.ScoreIndexedSpeakerCmd;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class Score2NotesLeftAuto extends SequentialCommandGroup {

  /* Constructor */
  public Score2NotesLeftAuto(MasterArmSubsystem noteConductor, 
                             SwerveSubsystem swerveDrive) {
    addCommands(
                new ScoreIndexedSpeakerCmd(noteConductor),
                new DeployIntakeCmd(noteConductor),
                new MoveSpkrLeftToLeftNoteCmd(swerveDrive),
                // Assume we picked up note, but add a short wait to correct any misallignment
                
                new WaitCommand(1.0),
                new RetrieveIntakeCmd(noteConductor),
                // For 2 note auto, back up to speaker, then score indexed shot
                new MoveLeftNoteToLeftIndexedShotCmd(swerveDrive).withTimeout(3.0),
                new ScoreIndexedSpeakerCmd(noteConductor)
               );
  }
}

