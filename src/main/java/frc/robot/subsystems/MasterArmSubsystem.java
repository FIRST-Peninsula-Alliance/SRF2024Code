// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;

import java.util.Map;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.FileRecorder;
import frc.lib.util.FileRecorder.NoteEvent;
import frc.lib.util.FileRecorder.NoteRequest;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Constants.F;
import frc.robot.NotableConstants.MAC;
import frc.robot.commands.RumbleCmd;

public class MasterArmSubsystem extends SubsystemBase {
  // Change DEBUG_ON to false to turn off both tracking and manually 
  // single stepping though state changes. The latter allows 
  // observing the effects of each change before moving on.

 
  // Repetoire defines Higher level Note Handler States
  // involving multiple SubSystems, which for convenience (related to
  // access to SubSystem handles) is processed in this MasterArm Subsystem
  public enum Repetoire {
    NOTE_HANDLER_IDLE,
    PREP_TO_GET_NOTE,
    WAIT_FOR_NOTE,
    RETRIEVE_NOTE,
    WAIT_FOR_SPECIFIED_GOAL,
    PREP_FOR_SPEAKER_GOAL,
    WAIT_TO_SCORE_SPEAKER,
    SCORE_SPEAKER,
    PREP_FOR_AMP_GOAL,
    WAIT_TO_SCORE_AMP,
    SCORE_AMP,
    RETURN_FROM_AMP,
    PREP_FOR_WAVING,
    WAVING_AT_CROWD,
    DEBUG_HOLD;
  }

  private Repetoire m_nowPlaying = Repetoire.NOTE_HANDLER_IDLE;
  private Repetoire m_pendingNote;

  private int m_currentSeqNo;               // ...SeqNo's are used to step through m_nowPlaying SubStates
  private int m_pendingSeqNo;
  private boolean m_isDistantSpeakerShot;   // flag keeps track of which speaker shot is called for
  
  private double m_avgMasterRawAbsPos;
  private double m_temp;

  private double m_currentMasterArmSetpoint;
  private double m_positionError;
  public long m_startTime;                  // use to measure timeouts on Arm movement
  public long m_elapsedTime;

  /********************************************************
   * MasterArmSubsystem needs access to 3 other subsystems
   * in order to sequence actions appropriately, especially 
   * to avoid collisions and avoid exceeding limits on
   * extenstion past the robot frame perimeter. It was 
   * easiest to get their Handles by instantiating then
   * directly from this subystem, and requiring all public
   * access to them (e.g. bindings in RobotContainer) to 
   * go though MasterArmSubsystem intermediaries.
   * For debug purposes, a FileRecorder is created to log
   * all NoteHandler requests, movements, timeouts,
   * state changes and errors to a thumb drive.
   ********************************************************/
  private static boolean DEBUG_ON = false;
  public static boolean NOTE_LOGGING_ACTIVE = false;
  public static boolean CTRE_SIGNAL_LOGGING_ACTIVE = false;

  public FileRecorder      m_fileRecorder;
  public InnerArmSubsystem m_innerArmSubsystem;
  public IntakeSubsystem   m_intakeSubsystem;
  public ShooterSubsystem  m_shooterSubsystem;;

  // Local motors and sensors 
  private TalonFX m_masterArmMotor = new TalonFX(MAC.MASTER_ARM_FALCON_ID, Constants.CANIVORE_BUS_NAME);
  private CANcoder m_masterArmEncoder = new CANcoder(MAC.MASTER_ARM_ENCODER_ID, Constants.CANIVORE_BUS_NAME);
  private final MotionMagicVoltage m_masterArmMotionMagicCtrl = 
                                 new MotionMagicVoltage(0.0).withSlot(0);

  // Network Table entries for publishing MasterArmSubsystem data
  private GenericEntry m_absAxlePosEntry;
  private GenericEntry m_rawAxlePosEntry;
  private GenericEntry m_armSetpointPosEntry;
  private GenericEntry m_armPIDOutEntry;
  private GenericEntry m_AxleVelocityEntry;
  private GenericEntry m_falconAmpsEntry;
  private GenericEntry m_playingNowEntry;
  private GenericEntry m_pendingNoteEntry;
  private GenericEntry m_seqNoEntry;
  private GenericEntry m_pendingSeqNoEntry;
  private GenericEntry m_isDistantShotEntry;

  private boolean m_isSafeToReturn;
  private boolean m_noWaitToScore;
  private boolean m_masterArmIsReadyToShoot;
  private boolean m_innerArmIsReadyToShoot;
  private double m_waveDirection;     // + or - 1.0, used to control up and down reversals 
                                      // of inner arm drive to create "wave" during parade
  private double m_waveSpeed;         // Percent output motor drive for wave
  private double m_waveMagnitude;     // Rotations away from 0, per side, during wave
  private boolean m_cancelWave = false;   // flag used to cancel wave

  /******************************************************
   * Constructor for a new MasterArmSubsystem. 
   ******************************************************/
  public MasterArmSubsystem() {
    if (CTRE_SIGNAL_LOGGING_ACTIVE) {
      Robot.startCtreSignalLogger();
    }
    System.out.println("Starting File Recorder");
    m_fileRecorder = new FileRecorder("NoteData1", NOTE_LOGGING_ACTIVE);
    m_innerArmSubsystem = new InnerArmSubsystem(()-> getCurrentStateName(),
                                                ()-> getCurrentSeqNo(),
                                                m_fileRecorder);
    m_intakeSubsystem  = new IntakeSubsystem(()-> getCurrentStateName(),
                                             ()-> getCurrentSeqNo(),
                                             m_fileRecorder);
    m_shooterSubsystem = new ShooterSubsystem(()-> getCurrentStateName(),
                                              ()-> getCurrentSeqNo(),
                                              m_fileRecorder);

    configAbsMasterArmCANCoder();
    configMasterArmMotor();
  //  m_intakeSubsystem.setDefaultCommand(DefaultIntakeCmd(m_intakeSubsystem, ()->RobotContainer.getHidXboxCtrl().getLeftY()));
    m_nowPlaying = Repetoire.NOTE_HANDLER_IDLE;
    m_pendingNote = m_nowPlaying;
    // The following is needed, but is redundant here, as the inneerArmSubsystem
    // constructor initializes to vertical position. Still, it can't hurt to help
    // ensure it gets there.
    m_innerArmSubsystem.gotoVerticalUpPos();
    m_isDistantSpeakerShot = false;
    m_noWaitToScore = false;
    m_isSafeToReturn = false;
    setupMasterArmPublishing();
    // init the variable holding the avg raw arm position
    m_avgMasterRawAbsPos = getRawMasterArmPos();
    gotoPosition(MAC.INDEXED_SPEAKER_SHOT_POS);
  }

  /***********************************************
   * State / SeqNo status access methods
   ***********************************************/

  public String getCurrentStateName() {
    return m_nowPlaying.toString();
  }

  public int getCurrentSeqNo() {
    return m_currentSeqNo;
  }

  /**************************************************************
   * Higher level Note handling trigger and state change methods
   **************************************************************/
  public void acquireNote() {
    if (m_nowPlaying != Repetoire.NOTE_HANDLER_IDLE) {
      new RumbleCmd(2, .4, 200).schedule();
      System.out.println("AcquireNote @ NST State = "+m_nowPlaying.toString());
    }
    // Only allow a switch to acquire Note state if in the following states:
    if ((m_nowPlaying == Repetoire.NOTE_HANDLER_IDLE)
        || (m_nowPlaying != Repetoire.WAIT_FOR_SPECIFIED_GOAL)
        || (m_nowPlaying != Repetoire.PREP_FOR_SPEAKER_GOAL)
        || (m_nowPlaying != Repetoire.WAIT_TO_SCORE_SPEAKER)) {
      changeNoteStateTo(Repetoire.PREP_TO_GET_NOTE);
    }
  }

  // retrieveNote is called normally when a Note is acquired (indicated either
  // by sensor or button press) but is also called to initiate retraction when
  // Cancel is called for by the operator
  public void retrieveNote() {
    if (m_nowPlaying == Repetoire.WAIT_FOR_NOTE) {
      // WAIT_FOR_NOTE is the normal state for this method to be called in, 
      // and a note is likely present. Set intake accordingly, change state,
      // and exit
      changeNoteStateTo(Repetoire.RETRIEVE_NOTE);
      m_intakeSubsystem.holdNote();
      return;
    }
    if (m_nowPlaying == Repetoire.PREP_TO_GET_NOTE) {
      // PREP_TO_GET_NOTE state means that retrieveNote() was called 
      // due to a cancel Cancel request.
      // If prep has progressed past bumpers, do normal retrieve,
      if (m_currentSeqNo > 2) {
        changeNoteStateTo(Repetoire.RETRIEVE_NOTE);
        m_intakeSubsystem.holdNote(); 
      } else {
        // else turn off intake, set inner arms vertical, set master arms 
        // to INDEXED_SPEAKER_SHOT_POS, change state to IDLE, then exit.
        m_intakeSubsystem.cancelIntake();
        m_innerArmSubsystem.gotoVerticalUpPos();
        gotoPosition(MAC.INDEXED_SPEAKER_SHOT_POS);
        changeNoteStateTo(Repetoire.NOTE_HANDLER_IDLE);
      }
      // either way, exit now
      return;
    }
    // Else rumble an error code.  retrieveNote should only be called due
    // to actual or simulated sensor input, or upon an operator cancel request,
    // so should be well vetted, but just in case...report the unexpected.
    new RumbleCmd(2, .5, 200).schedule();
    System.out.println("RetrieveNote @ NST State = "+m_nowPlaying.toString());
  }

  public void prepForAmpScore() {
    if ((m_nowPlaying == Repetoire.WAIT_FOR_SPECIFIED_GOAL)
        ||
        (m_nowPlaying == Repetoire.NOTE_HANDLER_IDLE)) {
      changeNoteStateTo(Repetoire.PREP_FOR_AMP_GOAL);
      // Ensure the following flags are cleared here. If either needs to be set, 
      // do so after this method is called.
      m_noWaitToScore = false;
      m_isSafeToReturn = false;
    } else {
      new RumbleCmd(2, .5, 200).schedule();
      System.out.println("PrepForAmp @ NST 4tate = "+m_nowPlaying.toString());
    }
  }

  public void prepForIndexedSpeakerScore() {
    if ((m_nowPlaying == Repetoire.WAIT_FOR_SPECIFIED_GOAL)
        ||
        (m_nowPlaying == Repetoire.NOTE_HANDLER_IDLE)) {
      changeNoteStateTo(Repetoire.PREP_FOR_SPEAKER_GOAL);
      m_isDistantSpeakerShot = false;
      // Ensure the m_noWaitToScore flag is cleared here for normal use. 
      // If it needs to be set, do so after this method is called.
      m_noWaitToScore = false;
    } else {
      new RumbleCmd(1, .5, 200).schedule();
      System.out.println("PrepForIndexedSpeaker @ NST State = "+m_nowPlaying.toString());
    }
  }

  // @param voltage = a double value which is either SC.SHOOTER_VOLTAGE_OUT_FAR
  // or SC.SHOOTER_VOLTAGE_OUT_PASS
  public void prepForDistantSpeakerScore(double voltage) {
    if ((m_nowPlaying == Repetoire.WAIT_FOR_SPECIFIED_GOAL)
        ||
        (m_nowPlaying == Repetoire.NOTE_HANDLER_IDLE)) {
      changeNoteStateTo(Repetoire.PREP_FOR_SPEAKER_GOAL);
      m_isDistantSpeakerShot = true;
      m_shooterSubsystem.setDistantShotVoltageOut(voltage);
      // Ensure the m_noWaitToScore flag is cleared here for normal use. 
      // If it needs to be set, do so after this method is called.
      m_noWaitToScore = false;
    } else {
      new RumbleCmd(2, .5, 200).schedule();
      System.out.println("PrepForDistantSpeaker @ NST State = "+m_nowPlaying.toString());
    }
  }

  public void scoreNote() {
    System.out.println("ScoreNote @ NST State = "+m_nowPlaying.toString());
    if (m_nowPlaying == Repetoire.WAIT_TO_SCORE_SPEAKER) {
      changeNoteStateTo(Repetoire.SCORE_SPEAKER);
    } else if (m_nowPlaying == Repetoire.WAIT_TO_SCORE_AMP) {
      changeNoteStateTo(Repetoire.SCORE_AMP);
    } else if ((m_nowPlaying == Repetoire.RETURN_FROM_AMP) 
               &&
               (! m_isSafeToReturn)) {
        m_isSafeToReturn = true;
    } else {
      new RumbleCmd(2, .5, 400).schedule();
      System.out.println("ScoreNote @ NST State = "+m_nowPlaying.toString());
    }
  }

  // Since shooter wheels do not stop in Autonomous, need this to stop the shooter
  public void teleopStart() {
    m_shooterSubsystem.cancelShooter();
  }

  public void discardNote() {
    // Previously, this method would try to get rid of a note in any state, and
    // then park the arms inside the robot.
    // Now, it will be limited to ejecting a note in those states where it may help.
    // For instance, a note has been picked up awkwardly - wrinkled, by only one 
    // finger instead of two, etc., or the note jammed in the shooter (in which case
    // the shooter will be stopped, the intake will be turned on full speed for
    // 2 seconds to try and pull the note out of the shooter, then it will be
    // ejected, and finally, to get rid of a note ready to score in the amp. All
    // will then pull the arms back into the robot in IDLE state, except in the
    // AMP shot case the operator will need to pull RT first, as in a normal arm
    // return.
    switch (m_nowPlaying) {
      case WAIT_FOR_NOTE:
        m_intakeSubsystem.ejectNote();
        // Change currentSeqNo to 20 so that the intake
        // can be reactivated once the ejection completes.
        changeSeqNoTo(20);
        break;  

      // In the following two cases the arms are in motion. As long as the arms are 
      // outside of the robot, go ahead and start the intakeSubsystem eject 
      // in case that helps upblock a jam. At the end of the arm motion(s),
      // the MasterArmSequencer should take care of any related intake action needed.
      case PREP_TO_GET_NOTE:
        if (m_currentSeqNo >= 3) {
          // Outide the robot frame. Start eject, but do not change state!
          // When seqNo reaches the normal endpoint, m_intakeSubsystem.acquireNote()
          // will be called, and it may override the ejection. Either way,
          // the operator can then press ALT-RT again to eject, then re-try, if needed.
          m_intakeSubsystem.ejectNote();
        }
        break;

      case RETRIEVE_NOTE:
        if (m_currentSeqNo <= 4) {
          // Not yet within the robot frame. Eject now, but again, no state change.
          m_intakeSubsystem.ejectNote();
        } 
        break;

      case WAIT_FOR_SPECIFIED_GOAL:
        // Note is held vertically in the inner arms.
        changeSeqNoTo(20);
        break;

        case WAIT_TO_SCORE_SPEAKER:
        case SCORE_SPEAKER:
        // Assume that note has jammed in the shooter. Try to recover 
        changeSeqNoTo(20);
        break;

      case PREP_FOR_AMP_GOAL:
      case WAIT_TO_SCORE_AMP:
      case SCORE_AMP:
        // Don't bother trying to recover from PREP_FOR_AMP_SCORE,
        // WAIT_FOR_AMP_SCORE, or SCORE_AMP - the operator can just
        // wait for movements to stop, then press RT twice to get rid of
        // any Note and return.
        break;
      
      case PREP_FOR_WAVING:
      case WAVING_AT_CROWD:
      default:
        // Nothing to do for these states. To cancel waving, use ALT-X (cancelNoteAction())
        break;
    }
  }

  /**********************************************************
   * cancelNoteAction() is intended primarily to stop motors.
   * For handling an unwanted note, use discardNote()
   **********************************************************/
  public void cancelNoteAction() {
    switch (m_nowPlaying) {
      case NOTE_HANDLER_IDLE:
      case WAIT_FOR_SPECIFIED_GOAL:
        break;

      case PREP_TO_GET_NOTE:
        // retrieveNote checks for sub states of this NoteState, so safe to
        // call it without any additional filters
        if (m_currentSeqNo <= 2) {
          // Not yet fully out of "holster". Return master arm to idle pos,
          // leave inner arm where it is (probably vertical), and return to IDLE
          gotoPosition(MAC.INDEXED_SPEAKER_SHOT_POS);
          changeNoteStateTo(Repetoire.NOTE_HANDLER_IDLE);
          m_intakeSubsystem.cancelIntake();
        } else {
          retrieveNote();
        }
        break;

      case WAIT_FOR_NOTE:
        // "Holster" the arms, regardless of whether a note is held or not
        retrieveNote();
        break;
        
      case PREP_FOR_SPEAKER_GOAL:
      case WAIT_TO_SCORE_SPEAKER:
      // Here we can cancel the shot but no reason not to hold onto the note.
      // The operator can always shoot it if they wany to get rid of it.
        m_shooterSubsystem.cancelShooter();
        changeNoteStateTo(Repetoire.WAIT_FOR_SPECIFIED_GOAL);
        break;

      case PREP_FOR_AMP_GOAL:
        // no ideal way to recover with arms extended. Best is just 
        // to dump the Note, but have to wait until prep is complete. 
        // After the no wait eject, still require that operator
        // trigger the arm return when safe, rather than automatically.
        m_noWaitToScore = true;
        break;
      
      case WAIT_TO_SCORE_AMP:
        // no ideal way to recover. Best is just to dump the Note,
        // then let operator trigger a return when safe
        changeNoteStateTo(Repetoire.SCORE_AMP);
        m_noWaitToScore = true;
        break;

      case RETRIEVE_NOTE:
      case SCORE_SPEAKER:
      case SCORE_AMP:
      case RETURN_FROM_AMP:
        // do nothing, let whichever process is active finish normally
        break;

      case PREP_FOR_WAVING:
        m_cancelWave = true;
        break;

      case WAVING_AT_CROWD:
        stopWavingAtCrowd();
        break;
        
      case DEBUG_HOLD:
      default:
        // Nothing to do
        break;
    }
  }
  
  /*****************************************************
   * General Utility routines, and supporting methods for
   * parade activities like "waving at crowds".
   * **************************************************/
  
   // waveAtCrowd will raise the masterArm to the Amp score
   // position, then ocillate the inner arm about the 0 position
   // (horizontal forward) at the rate (in units of max percent
   // output) and rotation (symetrically away from 0, + and -)
   // specified by the caller. Be sure to call stopWavingAtCrowd() in order
   // to restore the inner Arm default movement and allow the masterArm
   // to be lowered. In fact, call stopWavingAtCrowd() whenever Disable is
   // pressed and use state information to decide whether any action is needed.
   public void startWavingAtCrowd(double speed, double rotation) {
    // the folloing vetting is redundant, but won't hurt anything.
    // Parade code is not time or resource critical!

    m_cancelWave = false;

    m_waveSpeed = Math.abs(speed);
    if (m_waveSpeed > 0.9) {
      m_waveSpeed = 0.9;
    }

    m_waveMagnitude = Math.abs(rotation);
    if (m_waveMagnitude > 30.0/360.0) {
      m_waveMagnitude = 30.0/360.0;
    }

    if ((m_nowPlaying == Repetoire.NOTE_HANDLER_IDLE)
        ||
        (m_nowPlaying == Repetoire.WAIT_FOR_SPECIFIED_GOAL)) {
      changeNoteStateTo(Repetoire.PREP_FOR_WAVING);
    } else if ((m_nowPlaying == Repetoire.WAVING_AT_CROWD)
               ||
               (m_nowPlaying == Repetoire.PREP_FOR_WAVING)) {
      m_waveSpeed = speed;
      m_waveMagnitude = rotation;
      m_waveDirection = 1.0;
    } else {
      System.out.println(m_nowPlaying.toString()+" not valid for switch to Waving");
    }
  }

  public void stopWavingAtCrowd() {
    if (m_nowPlaying == Repetoire.WAVING_AT_CROWD) {
      changeNoteStateTo(Repetoire.RETURN_FROM_AMP);
    } else if (m_nowPlaying == Repetoire.PREP_FOR_WAVING) {
      m_cancelWave = true;
    }
  }

  public boolean isWavingAtCrowd() {
    return (m_nowPlaying == Repetoire.PREP_FOR_WAVING) || (m_nowPlaying == Repetoire.WAVING_AT_CROWD);
  }

  /*********************************************
   * Status utilities to assist with Autonomous 
   *********************************************/
  // isIdle is typically used to ask if the shooter has completed a previously requested shot.
  public boolean isIdle() {
    return m_nowPlaying == Repetoire.NOTE_HANDLER_IDLE;
  }

  public boolean isIntakeDoneDeploying() {
    return ((m_nowPlaying == Repetoire.WAIT_FOR_NOTE)        // Expected result of deployment
            ||
            (m_nowPlaying != Repetoire.PREP_TO_GET_NOTE));   // "Cover all the bases" logic
  }

  public boolean isIntakeRetrieved() {
    return ((m_nowPlaying == Repetoire.WAIT_FOR_SPECIFIED_GOAL) // Expected result of retrieval
            ||
            (m_nowPlaying != Repetoire.RETRIEVE_NOTE));         // "Cover all the bases" logic
  }

  public boolean isReadyToScoreSpeaker() {
    return m_nowPlaying == Repetoire.WAIT_TO_SCORE_SPEAKER;
  }

  public boolean isReadyToScoreAmp() {
    return m_nowPlaying == Repetoire.WAIT_TO_SCORE_AMP;
  }

  /* **************************************************
   * getShooterSubsystem makes ShooterSubsystem handle 
   * available to RobotContainer for tuning aim and
   * velocity - not needed for competition
   * ***************************************************/
   // public ShooterSubsystem getShooterSubsystem() {
   //   return m_shooterSubsystem;
   // }

   // closeRecording is called from RobotContainer via
   // ALT+Start button binding - this can ensure that the
   // log file (if any) is closed cleanly 
  public void closeRecording() {
    if (NOTE_LOGGING_ACTIVE) {
      m_fileRecorder.closeFileRecorder();
    }
    if (Robot.isCtreSignalLoggerActive()) {
      Robot.stopCtreSignalLogger();
    }
  }

  /************************************************************************
   * Utilities for slightly adjusting position of MasterArm, inner arm,
   * and shooter aim for use when developing/tweaking standard setpoints, 
   * and potentially useful during match play if the robot gets out of 
   * alignment, but that would be an emergency manual adjustment per note
   * handled, very inefficient.
   * All take positions in Rotation units.
   * Redundancies in setting m_currentMasterArmSetpoint is a side effect of
   * using the normal gotoPosition() routine after adjusting the setpoint,
   * and sharing the SW LimitSwitch check.
   ************************************************************************/
  public double limitMasterArmPos(double position) {
    if (position > MAC.MAX_MASTER_ARM_SOFT_LIMIT) {
      return MAC.MAX_MASTER_ARM_SOFT_LIMIT;
    } else if (position < MAC.MIN_MASTER_ARM_SOFT_LIMIT) {
      return MAC.MIN_MASTER_ARM_SOFT_LIMIT;
    } else {
      return position;
    }
  }

  public void adjustMasterArmSetpointUp() {
    adjustMasterArmSetpoint(1.0);
  }
   
  public void adjustMasterArmSetpointDown() {
    adjustMasterArmSetpoint(-1.0);
  }

  // @param direction should be +1 or -1 ONLY. It is used as a factor
  // to increase or decrese the current position by a fixed step (1/360)
  // per call.
  public void adjustMasterArmSetpoint(double direction) {
    // Allow setpoint adjustment in any state, as long as direction is valid
    if (Math.abs(direction) != 1.0) {
      System.out.println("MA adjust ignored - invalid dir arg "+direction);
    } else {
      m_currentMasterArmSetpoint = limitMasterArmPos(m_currentMasterArmSetpoint + (direction / 360));
      gotoPosition(m_currentMasterArmSetpoint);
    } 
  }

  public void adjustInnerArmSetpointUp() {
    adjustInnerArmSetpoint(1.0);
  }
  
  public void adjustInnerArmSetpointDown() {
    adjustInnerArmSetpoint(-1.0);
  }
  
  public void adjustInnerArmSetpoint(double direction) {
    if (Math.abs(direction) != 1.0) {
      System.out.println("IA adjust req ignored: invalid direction argument "+direction);
    } else {
      m_innerArmSubsystem.adjustInnerArmSetpoint(direction, DEBUG_ON);
    }
  }

  public void adjustShooterAimUp() {
    adjustShooterAim(1.0);
  }

  public void adjustShooterAimDown() {
    adjustShooterAim(-1.0);
  }

  public void adjustShooterAim(double direction) {
    m_shooterSubsystem.adjustShooterAim(direction);
  }

  /*************************************************************
   * @param setpoint
   * gotoPosition() method directs MasterArm to a desired 
   * position. The setpoint argument is in absolute 
   * rotation units, and will generally be limited 
   * to those pre-defined in NotableConstants.java.
   **************************************************************/
  public void gotoPosition(double setpoint) {
    m_currentMasterArmSetpoint = setpoint;
    m_masterArmMotor.setControl(m_masterArmMotionMagicCtrl.withPosition(setpoint));
    m_startTime = System.currentTimeMillis();
    if (NOTE_LOGGING_ACTIVE) {
      m_fileRecorder.recordReqEvent("MA",
                                    NoteRequest.MOVE_MASTER_ARM,
                                    m_currentMasterArmSetpoint,
                                    m_startTime,
                                    m_nowPlaying.toString(),
                                    m_currentSeqNo);
    }
  }

  /********************************************************************************
   * Methods which return or check the current MasterArm position.
   ********************************************************************************/
  // getAbsMasterArmPos returns the MasterArmEncoder sensor position
  // which is in absolute rotations, with origin of 0 when horizontal,
  // corrected for magnet offset (the magnet offset must be measured,
  // then stored in NotableConstants.java).
   public double getAbsMasterArmPos() {
    return(m_masterArmEncoder.getAbsolutePosition().getValueAsDouble());
  }

  // isMasterArmAt can be called for any MasterArm position that has previously 
  // been sent to the motor controller as a setpoint. It returns true if the 
  // current Arm position is within an allowable delta of the setpoint, or if 
  // the arrival timeout has expired (every requested move records a startTime 
  // from which a timeout can be calculated). Otherwise it returns false.
  private boolean isMasterArmAt(double position, long timeoutDuration) {
    m_positionError = position - getAbsMasterArmPos();
    m_elapsedTime = System.currentTimeMillis() - m_startTime;

    if (Math.abs(m_positionError) < MAC.ALLOWED_MASTER_ARM_POS_ERROR) {
      m_fileRecorder.recordMoveEvent("MA",
                                     NoteEvent.SETPOINT_REACHED,
                                     position,
                                     m_positionError,
                                     System.currentTimeMillis(),
                                     m_elapsedTime,
                                     m_nowPlaying.toString(),
                                     m_currentSeqNo);
      return true;
    }
    if (m_elapsedTime <= timeoutDuration) {
      // general timeout has not yet expired.
      return false;
    } else {
      // timeout has occured. Log event and force assumption that MasterArm is at 
      // the requested setpoint. This avoids "hanging" the state machine due to
      // a slow PID (hopefully the position is close enough).
      if (NOTE_LOGGING_ACTIVE) {
        m_fileRecorder.recordMoveEvent( "MA",
                                        NoteEvent.TIMEOUT_OCCURED,
                                        position,
                                        m_positionError,
                                        System.currentTimeMillis(),
                                        m_elapsedTime,
                                        m_nowPlaying.toString(),
                                        m_currentSeqNo);
      }
      return true;
    }
  }

  // This method returns the uncorrected CANcoder Absolute position. The valid range of one
  // revolution is -.5 to + .5, so translate to baseband if needed.
  public double getRawMasterArmPos() {
    m_temp = getAbsMasterArmPos() - MAC.MASTER_ARM_ENCODER_MAGNET_OFFSET;
    if (Math.abs(m_temp) > .5) {
      m_temp = m_temp - Math.copySign(1.0, m_temp);
    } 
    return m_temp;
  }

  /*******************************************************************
   * Setup and Config routines
   ******************************************************************/
  private void configMasterArmMotor() {
    var closedLoopConfig = new ClosedLoopRampsConfigs().withDutyCycleClosedLoopRampPeriod(MAC.MASTER_ARM_CLOSED_LOOP_RAMP_PERIOD)
                                                       .withVoltageClosedLoopRampPeriod(MAC.MASTER_ARM_CLOSED_LOOP_RAMP_PERIOD)
                                                       .withTorqueClosedLoopRampPeriod(0);
    var feedbackConfig = new FeedbackConfigs().withFeedbackSensorSource(FeedbackSensorSourceValue.FusedCANcoder)
                                              .withFeedbackRemoteSensorID(MAC.MASTER_ARM_ENCODER_ID)
                                              .withSensorToMechanismRatio(MAC.MASTER_ARM_ENCODER_TO_AXLE_RATIO)
                                              .withRotorToSensorRatio(MAC.MASTER_ARM_ROTOR_TO_ENCODER_RATIO);
    var motorOutputConfig = new MotorOutputConfigs().withNeutralMode(MAC.MASTER_ARM_MOTOR_NEUTRAL_MODE)
                                                    .withInverted(MAC.MASTER_ARM_MOTOR_INVERT)
                                                    .withPeakForwardDutyCycle(MAC.MASTER_ARM_OUTPUT_LIMIT_FACTOR)
                                                    .withPeakReverseDutyCycle(-MAC.MASTER_ARM_OUTPUT_LIMIT_FACTOR);
    var currentLimitConfig = new CurrentLimitsConfigs().withSupplyCurrentLimit(MAC.MASTER_ARM_SUPPLY_CURRENT_LIMIT)
                                                       .withSupplyCurrentThreshold(MAC.MASTER_ARM_SUPPLY_CURRENT_THRESHOLD)
                                                       .withSupplyTimeThreshold(MAC.MASTER_ARM_SUPPLY_TIME_THRESHOLD)
                                                       .withSupplyCurrentLimitEnable(MAC.MASTER_ARM_ENABLE_CURRENT_LIMIT)
                                                       .withStatorCurrentLimit(MAC.MASTER_ARM_STATOR_CURRENT_LIMIT)
                                                       .withStatorCurrentLimitEnable(MAC.MASTER_ARM_ENABLE_STATOR_CURRENT_LIMIT);
    var swLimitConfig = new SoftwareLimitSwitchConfigs().withForwardSoftLimitEnable(MAC.ENABLE_MASTER_ARM_SOFT_LIMITS)
                                                        .withForwardSoftLimitThreshold(MAC.MAX_MASTER_ARM_SOFT_LIMIT)
                                                        .withReverseSoftLimitEnable(MAC.ENABLE_MASTER_ARM_SOFT_LIMITS)
                                                        .withReverseSoftLimitThreshold(MAC.MIN_MASTER_ARM_SOFT_LIMIT);
    Slot0Configs pid0Config = new Slot0Configs().withKP(MAC.MASTER_ARM_KP)
                                                .withKI(MAC.MASTER_ARM_KI)
                                                .withKD(MAC.MASTER_ARM_KD)
                                                .withKS(MAC.MASTER_ARM_KS)
                                                .withKV(MAC.MASTER_ARM_KV)
                                                .withKA(MAC.MASTER_ARM_KA)
                                                .withKG(MAC.MASTER_ARM_KG).withGravityType(GravityTypeValue.Arm_Cosine);
    var motionMagicConfig = new MotionMagicConfigs().withMotionMagicCruiseVelocity(4.0)
                                                    .withMotionMagicAcceleration(8.0)
                                                    .withMotionMagicJerk(25);
    var masterArmConfig = new TalonFXConfiguration().withFeedback(feedbackConfig)
                                                    .withMotorOutput(motorOutputConfig)
                                                    .withCurrentLimits(currentLimitConfig)
                                                    .withSoftwareLimitSwitch(swLimitConfig)
                                                    .withClosedLoopRamps(closedLoopConfig)
                                                    .withSlot0(pid0Config)
                                                    .withMotionMagic(motionMagicConfig);
    StatusCode status = m_masterArmMotor.getConfigurator().apply(masterArmConfig);

    if (! status.isOK() ) {
        System.out.println("Failed to apply MASTER_ARM configs. Error code: "+status.toString());
    }
  }

  private void configAbsMasterArmCANCoder() {        
    var magnetSensorConfigs = new MagnetSensorConfigs().withAbsoluteSensorRange(MAC.MASTER_ARM_CANCODER_RANGE)
                                                       .withSensorDirection(MAC.MASTER_ARM_CANCODER_DIR)
                                                       .withMagnetOffset(MAC.MASTER_ARM_ENCODER_MAGNET_OFFSET);
    var ccConfig = new CANcoderConfiguration().withMagnetSensor(magnetSensorConfigs);
    StatusCode status = m_masterArmEncoder.getConfigurator().apply(ccConfig);
    if (! status.isOK() ) {
        System.out.println("Failed to apply MASTER CANcoder configs. Error code: "+status.toString());
    }
  }

  /********************************************************************
   * Publishing setup and operation
   ********************************************************************/
  private void setupMasterArmPublishing() {
    // Get or establish the "Note Handlers" tab
    ShuffleboardTab sbt = Shuffleboard.getTab("Note Handlers");

    // Setup the "Master Arm" data list
    ShuffleboardLayout sbLayout = sbt.getLayout("Master Arm", BuiltInLayouts.kList)
                                      .withPosition(MAC.MASTER_ARM_DATA_COL,
                                                    MAC.MASTER_ARM_DATA_ROW)
                                      .withSize(2, MAC.MASTER_ARM_DATA_LIST_HGT)
                                      .withProperties(Map.of("Label position", "LEFT"));
    sbLayout.add("Ids ", F.df1.format(MAC.MASTER_ARM_FALCON_ID)+"  "+F.df1.format(MAC.MASTER_ARM_ENCODER_ID));
    sbLayout.add("Offset", F.df1.format(MAC.MASTER_ARM_ENCODER_MAGNET_OFFSET));
    m_rawAxlePosEntry     = sbLayout.add("Raw_P", "0").getEntry();
    m_absAxlePosEntry     = sbLayout.add("Abs_P", "0").getEntry();
    m_armSetpointPosEntry = sbLayout.add("SetPt", "0").getEntry();
    m_armPIDOutEntry      = sbLayout.add("PID Out", "0").getEntry();
    m_AxleVelocityEntry   = sbLayout.add("Axle_V", "0").getEntry();
    m_falconAmpsEntry     = sbLayout.add("Amps", "0").getEntry();

    // Setup the "Note States" data list
    sbLayout = sbt.getLayout("Note States", BuiltInLayouts.kList)
                                      .withPosition(MAC.MASTER_ARM_DATA_COL + 4,
                                                    MAC.MASTER_ARM_DATA_ROW)
                                      .withSize(2, MAC.MASTER_ARM_DATA_LIST_HGT)
                                      .withProperties(Map.of("Label position", "LEFT"));
                                        m_AxleVelocityEntry   = sbLayout.add("Axle Vel", "0").getEntry();
    m_playingNowEntry = sbLayout.add("Playing ", "").getEntry();
    m_pendingNoteEntry = sbLayout.add("Pending ", "").getEntry();
    m_seqNoEntry = sbLayout.add("SeqNo ", 0).getEntry();
    m_pendingSeqNoEntry = sbLayout.add("Pend SeqNo ", 0).getEntry();
    m_isDistantShotEntry = sbLayout.add("DistShot ", "No").getEntry();

    // Note that the InnerArm shares space on the "Note Handlers" Tab
  }

  private void publishMasterArmData() {
    m_rawAxlePosEntry.setString(F.df4.format(getRawMasterArmPos()));
    m_absAxlePosEntry.setString(F.df3.format(getAbsMasterArmPos()));
    m_armSetpointPosEntry.setString(F.df3.format(m_currentMasterArmSetpoint));
    m_armPIDOutEntry.setString(F.df3.format(m_masterArmMotor.getClosedLoopOutput().getValueAsDouble()));
    m_AxleVelocityEntry.setString(F.df3.format(m_masterArmEncoder.getVelocity().getValueAsDouble()));
    m_falconAmpsEntry.setString(F.df3.format(m_masterArmMotor.getSupplyCurrent().getValueAsDouble()));
    m_playingNowEntry.setString(m_nowPlaying.toString());
    m_pendingNoteEntry.setString(m_pendingNote.toString());
    m_seqNoEntry.setInteger(m_currentSeqNo);
    m_pendingSeqNoEntry.setInteger(m_pendingSeqNo);
    m_isDistantShotEntry.setString(m_isDistantSpeakerShot ? "Yes" : "No");

    // Shuffleboard is notoriously fickle with programatically generated tabs,
    // sometimes it stops the live data updates.
    // Post any critical data to SmartDashboard Tab in parallel
    m_avgMasterRawAbsPos = ((m_avgMasterRawAbsPos * .95) + (getRawMasterArmPos() * .05));
    SmartDashboard.putNumber("MasterAvgRawAbsPos ", m_avgMasterRawAbsPos);
    SmartDashboard.putNumber("MasterCorrAbsPos ", getAbsMasterArmPos());
  } 

  /***************************************************************
   * periodic stuff - called once per loop
   ***************************************************************/
  public void periodic() {
    masterArmNoteHandlingSequencer();
    publishMasterArmData();
  }

   /*********************************************************************
   * Master Arm Sequencer
   * 
   * Sequences not just the master arm movements, but the entire suite  of
   * Note Handler subsystems.
   * Called once per loop from periodic()
   *********************************************************************/

  public void masterArmNoteHandlingSequencer() {
    switch (m_nowPlaying) {
      case NOTE_HANDLER_IDLE:
        // Nothing to do - will exit via button press instant command
        break;

      case PREP_TO_GET_NOTE:
        processNoteAcquisitionPrep();
        break;
      
      case WAIT_FOR_NOTE:
        processWaitForNote();           // check for eject note
        break;

      case RETRIEVE_NOTE:
        processRetrievingNote();
        break;

      case WAIT_FOR_SPECIFIED_GOAL:
        processWaitForSpecifiedGoal();   // Check for eject note
        break;

      case PREP_FOR_SPEAKER_GOAL:
        processSpeakerScorePrep();
        break;

      case WAIT_TO_SCORE_SPEAKER:
        if (m_noWaitToScore) {
          changeNoteStateTo(Repetoire.SCORE_SPEAKER);
          m_noWaitToScore = false;
        } else if (m_currentSeqNo == 20) {
          // need to discard the note.
          changeNoteStateTo(Repetoire.SCORE_SPEAKER);
          changeSeqNoTo(20);
        }
        // else nothing to do - will exit via operator button press
        break;

      case SCORE_SPEAKER:
        processScoringSpeaker();
        break;

      case PREP_FOR_AMP_GOAL:
      case PREP_FOR_WAVING:
        processAmpScorePrep();
        break;

      case WAIT_TO_SCORE_AMP:
        if (m_noWaitToScore) {
          changeNoteStateTo(Repetoire.SCORE_AMP);
          m_noWaitToScore = false;
        }
        // else nothing to do - will exit via button press instant command
        break;

      case SCORE_AMP:
        processScoringAmp();
        break;

      case RETURN_FROM_AMP:
        processReturnFromAmp();
        break;

      case WAVING_AT_CROWD:
        processWavingAtCrowd();
        break;

      case DEBUG_HOLD:
        break;

      default:
        System.out.println("Invalid NST state in MasterArmSubsystem"+m_nowPlaying.toString());
        break;
    }
  }

  /****************************************************************
   * Methods for changing state and SeqNo
   * Both change methods support single stepping though States
   * and/or SeqNos if DEBUG_ON is true
   * Both also log to File all changes
   ***************************************************************/
  // setter for new Note state
  public void changeNoteStateTo(Repetoire newState) {
    if ((! DEBUG_ON) || (m_nowPlaying == Repetoire.DEBUG_HOLD)) {
      if (NOTE_LOGGING_ACTIVE) {
        m_fileRecorder.recordStateChange( System.currentTimeMillis(),
                                          m_nowPlaying.toString(),
                                          newState.toString(),
                                          m_currentSeqNo);
      }
      m_nowPlaying = newState;
    } else {
      if (NOTE_LOGGING_ACTIVE) {
        m_fileRecorder.recordStateChange( System.currentTimeMillis(),
                                          m_nowPlaying.toString(),
                                          Repetoire.DEBUG_HOLD.toString(),
                                          m_currentSeqNo );
      }
      m_pendingNote = newState;
      m_nowPlaying = Repetoire.DEBUG_HOLD;
    }
    resetSeqNo();   
  }

  private void changeSeqNoTo(int newSeqNo) {
    // If DEBUG is off, or if ON but we're already in a debug hold state, just
    // transition normally, no need to store pending seqNo.
    if ((! DEBUG_ON) || (m_currentSeqNo == 99)) {
      if (NOTE_LOGGING_ACTIVE) {
        m_fileRecorder.recordSeqNoChange( System.currentTimeMillis(),
                                          m_nowPlaying.toString(),
                                          m_currentSeqNo,
                                          newSeqNo );
      }
      m_currentSeqNo = newSeqNo;
    } else {
      // DEBUG is ON, so store new state as pending, and switch to the debug hold seqMo code.
      if (NOTE_LOGGING_ACTIVE) {
        m_fileRecorder.recordSeqNoChange( System.currentTimeMillis(),
                                          m_nowPlaying.toString(),
                                          m_currentSeqNo,
                                          99);
      }
      m_pendingSeqNo = newSeqNo;
      m_currentSeqNo = 99;
    }
  }

  public void resetSeqNo() {
    m_currentSeqNo = 1;
    m_pendingSeqNo = m_currentSeqNo;
  }

  public void stepPastDebugHold() {
    if (m_nowPlaying == Repetoire.DEBUG_HOLD) {
      changeNoteStateTo(m_pendingNote);
    }
    if (m_currentSeqNo == 99) {
      changeSeqNoTo(m_pendingSeqNo);
    }
  }
  
  /*****************************************
   * PROCESS ACQUIRE NOTE PREP
   *****************************************/
  private void processNoteAcquisitionPrep() {
    //1. do any validity checks? For now, assume a clean start from Idle
    switch (m_currentSeqNo) {
      case 1:
        m_innerArmSubsystem.gotoVerticalUpPos();          // Set innerArm moving, then wait for it
        changeSeqNoTo(2);
        break;

      case 2:
        if (m_innerArmSubsystem.innerArmIsVerticalUp()) {
          //System.out.println("MA Received a True response");  
          gotoPosition(MAC.LOW_SAFE_TO_ROTATE_OUT_POS);   // Set masterArm moving, then wait for it
          changeSeqNoTo(3);
        } else {
          //System.out.println("MA Recieved a False response");
        }
        break;

      case 3:
        if (isMasterArmAt(MAC.LOW_SAFE_TO_ROTATE_OUT_POS, MAC.ALLOWED_MILLIS_MA_SMALL_MOVE)) {
          m_innerArmSubsystem.gotoBumperContactPos();     // Set innerArm moving, then wait for it
          changeSeqNoTo(4);
        }
        break;

      case 4:
        if (m_innerArmSubsystem.innerArmIsAtBumperContactPos()) {
          gotoPosition(MAC.NOTE_PICKUP_POS);       // set masterArm moving to pickup Pos, then wait for it
          changeSeqNoTo(5);
        }
        break;

      case 5:
        if (isMasterArmAt(MAC.NOTE_PICKUP_POS, MAC.ALLOWED_MILLIS_MA_SMALL_MOVE)) {
          m_innerArmSubsystem.gotoNotePickupPos();        // set innerArm moving to pickup Pos, then wait for it
          changeSeqNoTo(6);
        }
        break;

      case 6:
        if (m_innerArmSubsystem.innerArmIsAtPickupPos()) {
          m_intakeSubsystem.acquireNote();                // Start up the intake
          changeNoteStateTo(Repetoire.WAIT_FOR_NOTE);        // and change NST state to wait for note
        }
        break;

      case 99:            // Debug hold - a code SeqNo
        break;

      default:
          System.out.println("Invalid SeqNo in processNoteAcquisitionPrep: "+m_currentSeqNo);
          break;
    }
  }

  /********************************************
   * PROCESS WAIT FOR NOTE
   ********************************************/
  private void processWaitForNote() {
    switch (m_currentSeqNo) {
      case 1:
        if (m_intakeSubsystem.isNoteAcquired()) {
          retrieveNote();
        };
        break;

      case 20:
        m_intakeSubsystem.ejectNote();
        changeSeqNoTo(21);
        break;

      case 21:
        if (m_intakeSubsystem.isIntakeIdle()) {
          // Eject is complete, so fire up the intake again, stay in WAIT_FOR_NOTE state,
          // got back to SeqNo 1 
          m_intakeSubsystem.acquireNote();
          changeSeqNoTo(1);
        }
        break;

      case 99:
        break;

      default:
          System.out.println("Invalid SeqNo in processWaitForNoe: "+m_currentSeqNo);
          break;
    }
  }

  /********************************************
   * PROCESS RETRIEVE NOTE
   ********************************************/
  private void processRetrievingNote() {
    switch (m_currentSeqNo) {
      case 1:
        m_innerArmSubsystem.gotoBumperContactPos();             // set innerArm moving, then wait for it
        changeSeqNoTo(2);
        break;

      case 2:
        if (m_innerArmSubsystem.innerArmIsAtBumperContactPos()) {
          gotoPosition(MAC.LOW_SAFE_TO_ROTATE_OUT_POS);       // set masterArm moving to the rotate out position 
                                                              // (where we will actually rotate inwards to vertical), 
                                                              // then wait for it
          changeSeqNoTo(3);
        }
        break;

      case 3:
        if (isMasterArmAt(MAC.LOW_SAFE_TO_ROTATE_OUT_POS, MAC.ALLOWED_MILLIS_MA_SMALL_MOVE)) {
          m_innerArmSubsystem.gotoVerticalUpPos();                // set innerArm moving, then wait for it
          changeSeqNoTo(4);
        }
        break;

      case 4:
        if (m_innerArmSubsystem.innerArmIsVerticalUp()) {
          gotoPosition(MAC.LOW_SAFE_TO_ROTATE_IN_POS);          // set masterArm moving, then wait for it
          changeSeqNoTo(5);
        }
        break;

      case 5:
        if (isMasterArmAt(MAC.LOW_SAFE_TO_ROTATE_IN_POS, MAC.ALLOWED_MILLIS_MA_SMALL_MOVE)) {
          //m_innerArmSubsystem.gotoSpeakerShotPos(false);  // set innerArm moving, then wait for it
          // We could eliminate this step now that default inner arm position is VERTICAL,
          // but leave it in case we revert to indexed speaker shot position
          m_innerArmSubsystem.gotoVerticalUpPos();            // set innerArm moving, then wait for it
          changeSeqNoTo(6);
        }
        break;

      case 6:
        // if (m_innerArmSubsystem.innerArmIsAtIndexedSpeakerPos()) {
        if (m_innerArmSubsystem.innerArmIsVerticalUp()) {
          gotoPosition(MAC.INDEXED_SPEAKER_SHOT_POS);       // set masterArm moving, then wait for it
          //m_innerArmSubsystem.gotoSpeakerShotPos(m_isDistantSpeakerShot);
          changeSeqNoTo(7);
        }
        break;

      case 7:
        if (isMasterArmAt(MAC.INDEXED_SPEAKER_SHOT_POS, MAC.ALLOWED_MILLIS_MA_SMALL_MOVE)) {
          changeNoteStateTo(Repetoire.WAIT_FOR_SPECIFIED_GOAL);
        }
        break;

      case 99:
        break;
        
      default:
        System.out.println("Invalid SeqNo in processRetrieveNote: "+m_currentSeqNo);
        break;
    }
  }

  /************************************************
   * PROCESS WAIT FOR SPECIFIED GOAL
   ************************************************/
  public void processWaitForSpecifiedGoal() {
    switch (m_currentSeqNo) {
      case 1:
        // Do nothing, just wait with inner arms vertical up for external trigger
        break;

      // SeqNo 20 is only used when a discardNote() request is made.
      case 20:
        // Need to eject - first go Horiz out
        m_innerArmSubsystem.gotoHorizontalForwardPos();
        changeSeqNoTo(21);
        break;

      case 21:
        if (m_innerArmSubsystem.innerArmIsHorizontalForwardPos()) {
          m_intakeSubsystem.ejectNote();
          changeSeqNoTo(22);
        }
        break;

      case 22:
        if (m_intakeSubsystem.isIntakeIdle()) {
          m_innerArmSubsystem.gotoVerticalUpPos();
          changeSeqNoTo(23);
        }
        break;

      case 23:
        if (m_innerArmSubsystem.innerArmIsVerticalUp()) {
          changeNoteStateTo(Repetoire.NOTE_HANDLER_IDLE);
        }
        break;

      case 99:
        break;

      default:
        break;
    }
  }

  /*****************************************
   * PROCESS SPEAKER SCORE PREP
   *****************************************/
  private void processSpeakerScorePrep() {
    switch (m_currentSeqNo) {
      case 1:
        // in this instance, we can do a lot in parallel, then wait for all to complete
        m_shooterSubsystem.prepareToShoot(m_isDistantSpeakerShot);
        m_innerArmSubsystem.gotoSpeakerShotPos(m_isDistantSpeakerShot);
        if (m_isDistantSpeakerShot) {
          gotoPosition(MAC.DISTANT_SPEAKER_SHOT_POS);
        } else {
          gotoPosition(MAC.INDEXED_SPEAKER_SHOT_POS);
        }
        m_masterArmIsReadyToShoot = false;
        m_innerArmIsReadyToShoot = false;
        changeSeqNoTo(2);
        m_startTime = System.currentTimeMillis();
        break;

      case 2:
        // If shooter system is ready, and both Arms are ready, advance NST State. 
        // Note that all of the polled systems have their own timeouts to ensure no 
        // infinite hangups during a match, but just in case check for a 2 sec 
        // master timeout in this seqNo.
        if (m_isDistantSpeakerShot) {
          if (! m_masterArmIsReadyToShoot) {
            if (isMasterArmAt(MAC.DISTANT_SPEAKER_SHOT_POS, MAC.ALLOWED_MILLIS_MA_SMALL_MOVE)) {
              m_masterArmIsReadyToShoot = true;
            }
            if (! m_innerArmIsReadyToShoot) {
              if (m_innerArmSubsystem.innerArmIsAtDistantSpeakerPos()) {
                m_innerArmIsReadyToShoot = true;
              }
            }
          }
        } else {
          if (! m_masterArmIsReadyToShoot) {
            if (isMasterArmAt(MAC.INDEXED_SPEAKER_SHOT_POS, MAC.ALLOWED_MILLIS_MA_SMALL_MOVE)) {
              m_masterArmIsReadyToShoot = true;
            }
            if (! m_innerArmIsReadyToShoot) {
              if (m_innerArmSubsystem.innerArmIsAtIndexedSpeakerPos()) {
                m_innerArmIsReadyToShoot = true;
              }
            }
          }
        }
        m_elapsedTime = System.currentTimeMillis() - m_startTime;
        if ((m_elapsedTime > 2000) || (m_shooterSubsystem.isReadyToShoot() 
                                       && 
                                       m_masterArmIsReadyToShoot 
                                       && 
                                       m_innerArmIsReadyToShoot)) {
          if (m_noWaitToScore) {
            changeNoteStateTo(Repetoire.SCORE_SPEAKER); 
          } else {
            changeNoteStateTo(Repetoire.WAIT_TO_SCORE_SPEAKER); 
          } 
        }          
        break;

      // SeqNo 20 is only set when a discardNote() request is made.
      // In this state the shooter motor is already on, and perhaps somehthing
      // is jammed in the mechanism. Try to recover by turning off the
      // shooter, turning on the intake at full speed, sending the inner
      // arm horizontal out, then discard and return. All of this is duplicated
      // in the Repetoire.SCORE_SPEAKER state, so just change state and then
      // set the SeqNo back to 20.
      case 20:
        changeNoteStateTo(Repetoire.SCORE_SPEAKER);
        changeSeqNoTo(20);
        break;

      case 99:
        break;

      default:
        System.out.println("Invalid SeqNo in processSpeakerScorePrep: "+m_currentSeqNo);
        break;
    }
  }

  /**************************************
   * PROCESS SCORING AT SPEAKER
   **************************************/
  private void processScoringSpeaker() {
    switch (m_currentSeqNo) {
      case 1: 
        m_intakeSubsystem.ejectNote();
        m_shooterSubsystem.shotInitiated();
        changeSeqNoTo(2);
        m_startTime = System.currentTimeMillis();
        break;

      case 2:
        if (m_intakeSubsystem.isIntakeIdle() 
            &&
            m_shooterSubsystem.isShotDetected()) {
          gotoPosition(MAC.INDEXED_SPEAKER_SHOT_POS);
          m_innerArmSubsystem.gotoVerticalUpPos();
          changeNoteStateTo(Repetoire.NOTE_HANDLER_IDLE);
          m_shooterSubsystem.cancelShooter();       // If autonomous, this is ignored
        }
        break;

      // SeqNo 20 is only set when a discardNote() request is made.
      // In this state the shooter motor is on, and perhaps somehthing
      // is jammed in the mechanism. Try to recover by turning off the
      // shooter, turning on the intake at full speed for 2 seconds, sending 
      // the inner arm horizontal out, then discard and return.
      case 20:
        m_shooterSubsystem.cancelShooter();
        m_intakeSubsystem.acquireNote();
        m_innerArmSubsystem.gotoVerticalUpPos();
        gotoPosition(MAC.LOW_SAFE_TO_ROTATE_OUT_POS);   //This sets m_startTime
        changeSeqNoTo(21);
        break;

      case 21:
        if (m_innerArmSubsystem.innerArmIsVerticalUp()
            &&
            isMasterArmAt(MAC.LOW_SAFE_TO_ROTATE_OUT_POS, MAC.ALLOWED_MILLIS_MA_SMALL_MOVE)) {
          m_innerArmSubsystem.gotoHorizontalForwardPos();
          changeSeqNoTo(22);
        }
        break;

      case 22:
        if (m_innerArmSubsystem.innerArmIsHorizontalForwardPos()) {
          m_intakeSubsystem.ejectNote();
          changeSeqNoTo(23);
        }
        break;

      case 23:
        if (m_intakeSubsystem.isIntakeIdle()) {
          m_innerArmSubsystem.gotoVerticalUpPos();
          changeSeqNoTo(24);
        }
        break;

      case 24:
        if (m_innerArmSubsystem.innerArmIsVerticalUp()) {
          gotoPosition(MAC.INDEXED_SPEAKER_SHOT_POS);
          changeNoteStateTo(Repetoire.NOTE_HANDLER_IDLE);
        }
        break;

      case 99:
        break;

      default:
        System.out.println("Invalid SeqNo in processScoringSpeaker: "+m_currentSeqNo);
        break;
    }
  }

/*****************************************
 * PROCESS PREP FOR AMP SCORE
 *****************************************/
  private void processAmpScorePrep() {
   switch (m_currentSeqNo) {
      case 1: 
        m_innerArmSubsystem.gotoVerticalUpPos();                // set innerArm moving, then wait for it
        changeSeqNoTo(2);
        break;

      case 2:
        if (m_innerArmSubsystem.innerArmIsVerticalUp()) {
          gotoPosition(MAC.MID_SAFE_TO_ROTATE_POS);       // set masterArm moving, then wait for it
          changeSeqNoTo(3);
        }
        break;

      case 3:
        if (isMasterArmAt(MAC.MID_SAFE_TO_ROTATE_POS, MAC.ALLOWED_MILLIS_MA_SMALL_MOVE)) {
          m_innerArmSubsystem.gotoHorizontalBackPos();            // set innerArm moving, then wait for it
          changeSeqNoTo(4);
        }
        break;

      case 4:
        if (m_innerArmSubsystem.innerArmIsHorizontalBackPos()) {
          gotoPosition(MAC.HIGH_SAFE_TO_ROTATE_POS);    // set masterArm moving, then wait for it
          changeSeqNoTo(5);
        }
        break;

      case 5:
        if (isMasterArmAt(MAC.HIGH_SAFE_TO_ROTATE_POS, MAC.ALLOWED_MILLIS_MA_LARGE_MOVE)) {
          m_innerArmSubsystem.gotoAmpShotPos();                   // set innerArm moving, then wait for it
          changeSeqNoTo(6);
        }
        break;

      case 6:
        if (m_innerArmSubsystem.innerArmIsAtAmpScoringPos()) {
          gotoPosition(MAC.AMP_SHOT_POS);                   // set masterArm moving, then wait for it
          changeSeqNoTo(7);
        }
        break;

      case 7:
        if (isMasterArmAt(MAC.AMP_SHOT_POS, MAC.ALLOWED_MILLIS_MA_SMALL_MOVE)) {
          if (m_nowPlaying == Repetoire.PREP_FOR_AMP_GOAL) {
            changeNoteStateTo(Repetoire.WAIT_TO_SCORE_AMP);
          } else if (m_nowPlaying == Repetoire.PREP_FOR_WAVING) {
            if (m_cancelWave) {
              changeNoteStateTo(Repetoire.RETURN_FROM_AMP);
              m_cancelWave = false;
            } else {
              changeNoteStateTo(Repetoire.WAVING_AT_CROWD);
            }
          }
        }
        break;

      case 99:
        break;

        default:
          System.out.println("Invalid SeqNo in processAmpScorePrep: "+m_currentSeqNo);
          break;
    }
  }

  /***********************************
   * PROCESS SCORING AT AMP
   ***********************************/
  private void processScoringAmp() {
    switch (m_currentSeqNo) {
      case 1: 
        m_intakeSubsystem.ejectNote();
        m_startTime = System.currentTimeMillis();
        changeSeqNoTo(2);
        break;

      case 2:
        if (m_intakeSubsystem.isIntakeIdle()) {
          changeNoteStateTo(Repetoire.RETURN_FROM_AMP);
        } else if ((System.currentTimeMillis() - m_startTime) > 1000) {
          changeNoteStateTo(Repetoire.RETURN_FROM_AMP);
        }
        m_isSafeToReturn = false;
        break;

      case 99:
        break;

      default:
        System.out.println("Invalid SeqNo in processScoringAmp: "+m_currentSeqNo);
        break;
    }
  }
  
  /*****************************
   * PROCESS RETURN FROM AMP
   *****************************/
  private void processReturnFromAmp() {
   switch (m_currentSeqNo) {
    case 1:
      if (m_isSafeToReturn) {
        gotoPosition(MAC.HIGH_SAFE_TO_ROTATE_POS);
        changeSeqNoTo(2);
      }
      break;

    case 2:
      if (isMasterArmAt(MAC.HIGH_SAFE_TO_ROTATE_POS, MAC.ALLOWED_MILLIS_MA_SMALL_MOVE)) {
        m_innerArmSubsystem.gotoHorizontalBackPos();
        changeSeqNoTo(3);
      }
      break;

    case 3:
      if (m_innerArmSubsystem.innerArmIsHorizontalBackPos()) {
        gotoPosition(MAC.MID_SAFE_TO_ROTATE_POS);
        changeSeqNoTo(4);
      }
      break;

    case 4:
      if (isMasterArmAt(MAC.MID_SAFE_TO_ROTATE_POS, MAC.ALLOWED_MILLIS_MA_LARGE_MOVE)) {
        m_innerArmSubsystem.gotoVerticalUpPos();
        changeSeqNoTo(5);
      }
      break;

    case 5:
      if (m_innerArmSubsystem.innerArmIsVerticalUp()) {
        gotoPosition(MAC.INDEXED_SPEAKER_SHOT_POS);
        changeSeqNoTo(6);
      }
      break;

    case 6:
      if (isMasterArmAt(MAC.INDEXED_SPEAKER_SHOT_POS, MAC.ALLOWED_MILLIS_MA_SMALL_MOVE)) {
         changeNoteStateTo(Repetoire.NOTE_HANDLER_IDLE);
      }
      break;

    case 99:
      break;
      
    default:
      System.out.println("Invalid SeqNo in processReturnFromAmp: "+m_currentSeqNo);
      break;
    }
  }

  /******************************************************
   * processWavingAtCrowd()
   ******************************************************/
  public void processWavingAtCrowd() {
    if (((m_waveDirection == 1.0) && (m_innerArmSubsystem.getAbsInnerArmPos() > m_waveMagnitude))
        ||
        ((m_waveDirection == -1.0) && (m_innerArmSubsystem.getAbsInnerArmPos() < -m_waveMagnitude))) {
      m_waveDirection *= -1.0;
    }
    m_innerArmSubsystem.drive(m_waveDirection * m_waveSpeed);
  }

   /******************************************************
   * Note Sensor simulation support method
   * ****************************************************/
  public void simulateNoteAcquired() {
    m_intakeSubsystem.triggerNoteAcquired();
    if (m_nowPlaying == Repetoire.WAIT_FOR_NOTE) {
      retrieveNote();
    }
  }
}

