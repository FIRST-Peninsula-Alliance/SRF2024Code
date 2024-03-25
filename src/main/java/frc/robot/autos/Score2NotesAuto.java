package frc.robot.autos;

import frc.robot.subsystems.MasterArmSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.commands.ScoreIndexedSpeakerCmd;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class Score2NotesAuto extends SequentialCommandGroup {
  private MasterArmSubsystem m_noteConductor;
  private SwerveSubsystem m_swerveDrive;

  /** Creates a new ScoreAndMove. */
  public Score2NotesAuto(MasterArmSubsystem noteConductor, 
                         SwerveSubsystem swerveDrive) {
    m_noteConductor = noteConductor;
    m_swerveDrive = swerveDrive;
    
    addCommands(
                new ScoreIndexedSpeakerCmd(m_noteConductor),
                new InstantCommand(() -> m_noteConductor.acquireNote()),
                new WaitCommand(1.0),
                new JustExitCmd(m_swerveDrive),
                new InstantCommand(()->m_noteConductor.simulateNoteAcquired()),
                // Can we just score from the distance? For now, assume not,
                // so back up to speaker, then score indexed shot
                new ReturnToSpeakerCmd(m_swerveDrive),
                new ScoreIndexedSpeakerCmd(m_noteConductor)
                // superfluous? new InstantCommand(()-> m_swerveDrive.stop())
               );
    }
}

