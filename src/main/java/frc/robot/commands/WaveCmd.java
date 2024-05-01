// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

//import java.util.concurrent.ThreadLocalRandom;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.SDC;
import frc.robot.NotableConstants.IAC;
import frc.robot.NotableConstants.MAC;
import frc.robot.subsystems.MasterArmSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class WaveCmd extends Command {
  private SwerveSubsystem m_swerveDrive;
  private MasterArmSubsystem m_noteConductor;
  private double m_waveRotationDirection = 1.0;
  private double m_currentHeading;
  private long m_startTime;
  private long m_waitTime;

  /** Creates a new WaveCmd. */
  public WaveCmd(SwerveSubsystem swerveDrive, MasterArmSubsystem noteConductor) {
    m_swerveDrive = swerveDrive;
    m_noteConductor = noteConductor;

    addRequirements(m_swerveDrive, m_noteConductor);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
	// The robot is expected to be on a trailer pointing straight back in start configuration.
	// The robot rotation will be controlled by the gyro, so it must be reset to 0.0 
	// when starting, but it would also be affected by any turns made by the 
	// trailer during the parade. Fortunately the route for the Irrigation Festival
	// Parade is a straight shot with no turns, once upon Washington Street, so don't 
	// actually start the wave until that point.
	m_swerveDrive.zeroGyro();
	
    m_startTime = System.currentTimeMillis();
    m_waitTime = 1000;   // initial pause gives time for master arm to move up
	
	m_noteConductor.startWavingAtCrowd(IAC.NORMAL_WAVE_SPEED, IAC.NORMAL_WAVE_MAGNITURE);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // rotate from side to side, unless wait time is > 0, in which case continue waving but
    // do not continue the robot's rotation until the pause has expired.
    // Initializing m_waitTime to 1000 gives the master arm time to get into waving position
    // before rotation starts.
    // Note that after initialization, the inner arm wave speed is increased, and the
    // magnitude of the wave simultaneously decreased, for the duration of each rotation pause.
    // On pause expiration restore those settings to normal.
    if (m_waitTime > 0) {
      if ((System.currentTimeMillis() - m_startTime) < m_waitTime) {
        // Robot is still paused for rotation. Just call drive
        // with all arguments set to 0.0, and return.
        m_swerveDrive.drive(new Translation2d(0.0, 0.0),
                            0.0, 
                            true);
        return;
      } else {
        // Rotation pause is over. Restore normal wave speed and magnitude, and clear 
        // m_waitTime to allow rotation to resume.
        m_waitTime = 0;
        m_noteConductor.startWavingAtCrowd(IAC.NORMAL_WAVE_SPEED, IAC.NORMAL_WAVE_MAGNITURE);
        m_noteConductor.gotoPosition(MAC.AMP_SHOT_POS);
      }
    }

    // Not in a paused state, so continue rotating
    m_swerveDrive.drive(new Translation2d(0.0, 0.0),
                        (SDC.WAVE_SWERVE_ROTATE_SPEED * 
                          SDC.MAX_ROBOT_ANG_VEL_RAD_PER_SEC * 
                          m_waveRotationDirection), 
                        true);

    // And check if rotation limit to either side has been reached or exceeded
    m_currentHeading = m_swerveDrive.getYaw2d().getDegrees();
    if (((m_waveRotationDirection == 1.0) && (m_currentHeading > SDC.WAVE_ROTATION_EXTENT))
        ||
        ((m_waveRotationDirection == -1.0) && (m_currentHeading < -SDC.WAVE_ROTATION_EXTENT))) {
      // limit has been reached, so begin a rotation pause, and also start waving fast
      m_waveRotationDirection *= -1.0;
      m_startTime = System.currentTimeMillis();
      m_waitTime = SDC.WAVE_ROTATION_PAUSE_IN_MS;
      m_noteConductor.startWavingAtCrowd(IAC.FAST_WAVE_SPEED, IAC.FAST_WAVE_MAGNITURE);
      m_noteConductor.gotoPosition(MAC.WAVE_BOW_POS);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_noteConductor.stopWavingAtCrowd();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return ! m_noteConductor.isWavingAtCrowd();
  }
}
