// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import frc.robot.autos.TrajectoryPaths.ExitBlueSpkrRightToRightWallCmd;
import frc.robot.commands.ScoreIndexedSpeakerCmd;
import frc.robot.subsystems.MasterArmSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class ScoreThenExitBlueRightAuto extends SequentialCommandGroup {

  /* Constructor */
  public ScoreThenExitBlueRightAuto(MasterArmSubsystem noteConductor, 
                                    SwerveSubsystem swerveDrive) {
    addCommands(
                new ScoreIndexedSpeakerCmd(noteConductor),
                new ExitBlueSpkrRightToRightWallCmd(swerveDrive)
               );
  }
}