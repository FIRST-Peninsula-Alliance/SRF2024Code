// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

//import java.util.concurrent.ThreadLocalRandom;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.SDC;
import frc.robot.NotableConstants.IAC;
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
    m_noteConductor.startWavingAtCrowd(IAC.NORMAL_WAVE_SPEED, IAC.NORMAL_WAVE_MAGNITURE);
    m_startTime = System.currentTimeMillis();
    m_waitTime = 1000;   // initial pause gives time for master arm to move up
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // rotate from side to side, unless wait time is > 0, in which case continue waving but
    // do not continue the robot's rotation until the pause has expired.
    if (m_waitTime > 0) {
      if ((System.currentTimeMillis() - m_startTime) < m_waitTime) {
        // Robot is still paused for rotation, waving faster than normal. Just call drive
        // with all arguments set to 0.0, and return.
        m_swerveDrive.drive(new Translation2d(0.0, 0.0),
                            0.0, 
                            true);
        return;
      } else {
        // Wait time is over. Restore normal wave speed, and clear m_waitTime to allow
        // rotation to resume.
        m_waitTime = 0;
        m_noteConductor.startWavingAtCrowd(IAC.NORMAL_WAVE_SPEED, IAC.NORMAL_WAVE_MAGNITURE);
      }
    }

    m_currentHeading = m_swerveDrive.getYaw2d().getRotations();
    if (((m_waveRotationDirection == 1.0) && (m_currentHeading > SDC.WAVE_ROTATION_EXTENT))
        ||
        ((m_waveRotationDirection == -1.0) && (m_currentHeading < -SDC.WAVE_ROTATION_EXTENT))) {
      // time to begin a rotation pause, and to start waving fast
      m_waveRotationDirection *= -1.0;
      m_startTime = System.currentTimeMillis();
      m_waitTime = SDC.WAVE_ROTATION_PAUSE_IN_MS;
      m_noteConductor.startWavingAtCrowd(IAC.FAST_WAVE_SPEED, IAC.FAST_WAVE_MAGNITURE);
    }

    m_swerveDrive.drive(new Translation2d(0.0, 0.0),
                        (SDC.WAVE_SWERVE_ROTATE_SPEED * 
                         SDC.MAX_ROBOT_ANG_VEL_RAD_PER_SEC * 
                         m_waveRotationDirection), 
                        true);
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
