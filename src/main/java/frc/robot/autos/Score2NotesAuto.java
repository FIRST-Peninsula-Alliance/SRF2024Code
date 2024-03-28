package frc.robot.autos;

import frc.robot.Constants.*;
import frc.robot.subsystems.MasterArmSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.RobotContainer;
import frc.robot.autos.TrajectoryPaths.MoveCenNoteToIndexedShotCmd;
import frc.robot.autos.TrajectoryPaths.MoveLeftNoteToLeftIndexedShotCmd;
import frc.robot.autos.TrajectoryPaths.MoveRightNoteToRightIndexedShotCmd;
import frc.robot.autos.TrajectoryPaths.MoveSpkrCenToCenNoteCmd;
import frc.robot.autos.TrajectoryPaths.MoveSpkrLeftToLeftNoteCmd;
import frc.robot.autos.TrajectoryPaths.MoveSpkrRightToRightNoteCmd;
import frc.robot.commands.DeployIntakeCmd;
import frc.robot.commands.RetrieveIntakeCmd;
import frc.robot.commands.ScoreIndexedSpeakerCmd;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class Score2NotesAuto extends SequentialCommandGroup {

  /** Creates a new ScoreAndMove. */
  public Score2NotesAuto(MasterArmSubsystem noteConductor, 
                         SwerveSubsystem swerveDrive) {

     switch (RobotContainer.getSelectedStartPosition()) {
      case AutoC.STARTING_LEFT:
        addCommands(
                    new ScoreIndexedSpeakerCmd(noteConductor),
                    new DeployIntakeCmd(noteConductor),
                    new MoveSpkrLeftToLeftNoteCmd(swerveDrive),
                    // Assume we picked up note, but add a short wait to correct any misallignment
                    new WaitCommand(2.0),
                    new RetrieveIntakeCmd(noteConductor),
                    // For 2 note auto, back up to speaker, then score indexed shot
                    new MoveLeftNoteToLeftIndexedShotCmd(swerveDrive).withTimeout(1.8),
                    new ScoreIndexedSpeakerCmd(noteConductor)
                   );
        break;

      case AutoC.STARTING_CENTER:
        addCommands(
                    new ScoreIndexedSpeakerCmd(noteConductor),
                    new DeployIntakeCmd(noteConductor),
                    new MoveSpkrCenToCenNoteCmd(swerveDrive, true),
                    // Assume we picked up note, but add a short wait to correct any misallignment
                    new WaitCommand(2.0),
                    new RetrieveIntakeCmd(noteConductor),
                    // For 2 note auto, back up to speaker, then score indexed shot
                    new MoveCenNoteToIndexedShotCmd(swerveDrive).withTimeout(1.8),
                    new ScoreIndexedSpeakerCmd(noteConductor)
                   );
        break;

      case AutoC.STARTING_RIGHT:
        addCommands(
                    new ScoreIndexedSpeakerCmd(noteConductor),
                    new DeployIntakeCmd(noteConductor),
                    new MoveSpkrRightToRightNoteCmd(swerveDrive),
                    // Assume we picked up note, but add a short wait to correct any misallignment
                    new WaitCommand(2.0),
                    new RetrieveIntakeCmd(noteConductor),
                    // For 2 note auto, back up to speaker, then score indexed shot
                    new MoveRightNoteToRightIndexedShotCmd(swerveDrive).withTimeout(1.8),
                    new ScoreIndexedSpeakerCmd(noteConductor)
                   );
        break;

      default:
        System.out.println("Invalid Start Position: "+RobotContainer.getSelectedStartPosition());
        break;
    }
  }
}

