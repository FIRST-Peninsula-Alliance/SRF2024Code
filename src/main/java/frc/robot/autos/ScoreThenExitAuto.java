// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import frc.robot.Constants.AutoC;
import frc.robot.RobotContainer;
import frc.robot.autos.TrajectoryPaths.ExitSpkrLeftToLeftWallCmd;
import frc.robot.autos.TrajectoryPaths.ExitSpkrRightToRightWallCmd;
import frc.robot.commands.DoNothingCmd;
import frc.robot.commands.ScoreIndexedSpeakerCmd;
import frc.robot.subsystems.MasterArmSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class ScoreThenExitAuto extends SequentialCommandGroup {
  Command m_exitCmd;

  /** Creates a new ScoreAndMove. */
  public ScoreThenExitAuto(MasterArmSubsystem noteConductor, 
                           SwerveSubsystem swerveDrive) {

    switch (RobotContainer.getSelectedStartPosition()) {
      case AutoC.STARTING_LEFT:
        m_exitCmd = new ExitSpkrLeftToLeftWallCmd(swerveDrive);
        break;

      case AutoC.STARTING_CENTER:
        m_exitCmd = new DoNothingCmd();
        break;

      case AutoC.STARTING_RIGHT:
        m_exitCmd = new ExitSpkrRightToRightWallCmd(swerveDrive);
        break;

      default:
        System.out.println("Invalid Auto Start Position detected: "+RobotContainer.getSelectedStartPosition());
        break;
    }
    
    addCommands(
                new ScoreIndexedSpeakerCmd(noteConductor),
                m_exitCmd
               );
  }
}