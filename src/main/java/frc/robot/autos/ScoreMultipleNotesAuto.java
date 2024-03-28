// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import frc.robot.Constants.AutoC;
import frc.robot.RobotContainer;
import frc.robot.autos.TrajectoryPaths.MoveCenNoteToDistantShotCmd;
import frc.robot.autos.TrajectoryPaths.MoveDistShotToLeftNoteCmd;
import frc.robot.autos.TrajectoryPaths.MoveDistShotToRightNoteCmd;
import frc.robot.autos.TrajectoryPaths.MoveLeftNoteToIndexedShotCmd;
import frc.robot.autos.TrajectoryPaths.MoveRightNoteToDistantShotCmd;
import frc.robot.autos.TrajectoryPaths.MoveRightNoteToIndexedShotCmd;
import frc.robot.autos.TrajectoryPaths.MoveSpkrCenToCenNoteCmd;
import frc.robot.autos.TrajectoryPaths.MoveSpkrLeftToLeftNoteCmd;
import frc.robot.autos.TrajectoryPaths.MoveSpkrRightToRightNoteCmd;
import frc.robot.commands.DeployIntakeCmd;
import frc.robot.commands.RetrieveIntakeCmd;
import frc.robot.commands.ScoreDistantSpeakerCmd;
import frc.robot.commands.ScoreIndexedSpeakerCmd;
import frc.robot.subsystems.MasterArmSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class ScoreMultipleNotesAuto extends SequentialCommandGroup {

  public ScoreMultipleNotesAuto(MasterArmSubsystem noteConductor, 
                                SwerveSubsystem swerveDrive) {

    switch (RobotContainer.getSelectedStartPosition()) {
      case AutoC.STARTING_LEFT:
        addCommands(
                    new ScoreIndexedSpeakerCmd(noteConductor),
                    new DeployIntakeCmd(noteConductor),
                    new MoveSpkrLeftToLeftNoteCmd(swerveDrive),
                    // Assume we picked up note, but allow time to recover from any mis-allignment
                    new WaitCommand(1.0),
                    new RetrieveIntakeCmd(noteConductor),
                    new MoveLeftNoteToIndexedShotCmd(swerveDrive),
                    new ScoreIndexedSpeakerCmd(noteConductor),
                    new DeployIntakeCmd(noteConductor),
                    new MoveSpkrCenToCenNoteCmd(swerveDrive, false),
                    new WaitCommand(1.0),
                    new RetrieveIntakeCmd(noteConductor),
                    new MoveCenNoteToDistantShotCmd(swerveDrive),
                    new ScoreDistantSpeakerCmd(noteConductor),
                    new DeployIntakeCmd(noteConductor),
                    new MoveDistShotToRightNoteCmd(swerveDrive),
                    new WaitCommand(1.0),
                    new RetrieveIntakeCmd(noteConductor),
                    new MoveRightNoteToIndexedShotCmd(swerveDrive),
                    new ScoreIndexedSpeakerCmd(noteConductor)
                   );
        break;

      case AutoC.STARTING_CENTER:
        addCommands(
                    new ScoreIndexedSpeakerCmd(noteConductor),
                    new DeployIntakeCmd(noteConductor),
                    new MoveSpkrCenToCenNoteCmd(swerveDrive, true),
                    // Assume we picked up note, but allow time to recover from any mis-allignment
                    new WaitCommand(1.0),
                    new RetrieveIntakeCmd(noteConductor),
                    new MoveCenNoteToDistantShotCmd(swerveDrive),
                    new ScoreDistantSpeakerCmd(noteConductor),
                    new DeployIntakeCmd(noteConductor),
                    new MoveDistShotToRightNoteCmd(swerveDrive),
                    new WaitCommand(1.0),
                    new RetrieveIntakeCmd(noteConductor),
                    new MoveRightNoteToDistantShotCmd(swerveDrive),
                    new ScoreDistantSpeakerCmd(noteConductor),
                    new DeployIntakeCmd(noteConductor),
                    new MoveDistShotToLeftNoteCmd(swerveDrive),
                    new WaitCommand(1.0),
                    new RetrieveIntakeCmd(noteConductor),
                    new MoveLeftNoteToIndexedShotCmd(swerveDrive),
                    new ScoreIndexedSpeakerCmd(noteConductor)
                   );
        break;

      case AutoC.STARTING_RIGHT:
        addCommands(
                    new ScoreIndexedSpeakerCmd(noteConductor),
                    new DeployIntakeCmd(noteConductor),
                    new MoveSpkrRightToRightNoteCmd(swerveDrive),
                    // Assume we picked up note, but allow time to recover from any mis-allignment
                    new WaitCommand(1.0),
                    new RetrieveIntakeCmd(noteConductor),
                    new MoveRightNoteToIndexedShotCmd(swerveDrive),
                    new ScoreIndexedSpeakerCmd(noteConductor),
                    new DeployIntakeCmd(noteConductor),
                    new MoveSpkrCenToCenNoteCmd(swerveDrive, false),
                    new WaitCommand(1.0),
                    new RetrieveIntakeCmd(noteConductor),
                    new MoveCenNoteToDistantShotCmd(swerveDrive),
                    new ScoreDistantSpeakerCmd(noteConductor),
                    new DeployIntakeCmd(noteConductor),
                    new MoveDistShotToLeftNoteCmd(swerveDrive),
                    new WaitCommand(1.0),
                    new RetrieveIntakeCmd(noteConductor),
                    new MoveLeftNoteToIndexedShotCmd(swerveDrive),
                    new ScoreIndexedSpeakerCmd(noteConductor)
                   );
        break;

      default:
        System.out.println("Invalid Start Position: "+RobotContainer.getSelectedStartPosition());
        break;
    }
  }
}
