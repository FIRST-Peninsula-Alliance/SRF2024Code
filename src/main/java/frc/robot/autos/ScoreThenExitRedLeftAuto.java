// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.autos.TrajectoryPaths.ExitRedSpkrLeftToLeftWallCmd;
import frc.robot.commands.ScoreIndexedSpeakerCmd;
import frc.robot.subsystems.MasterArmSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class ScoreThenExitRedLeftAuto extends SequentialCommandGroup {
  
  /* Constructor */
  public ScoreThenExitRedLeftAuto(MasterArmSubsystem noteConductor, 
                               SwerveSubsystem swerveDrive) {
    addCommands(
                new ScoreIndexedSpeakerCmd(noteConductor),
                new ExitRedSpkrLeftToLeftWallCmd(swerveDrive)
               );
  }
}
