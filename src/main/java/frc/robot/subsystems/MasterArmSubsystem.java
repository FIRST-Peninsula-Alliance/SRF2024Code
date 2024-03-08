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
// import edu.wpi.first.wpilibj.RobotState;    // needed if Test mode code is uncommented
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.FileRecorder;
import frc.lib.util.FileRecorder.NoteEvent;
import frc.robot.Constants;
import frc.robot.Constants.F;
import frc.robot.NotableConstants.MAC;
import frc.robot.commands.RumbleCmd;

public class MasterArmSubsystem extends SubsystemBase {
  // Change DEBUG_ON to false to turn off both tracking and manually 
  // single stepping though state changes. The latter allows 
  // observing the effects of each change before moving on.

  private static boolean DEBUG_ON = false;
  private static boolean LOGGING_ON = true;

  public long m_startTime;      // use to measure timeouts on Arm movement
  public long m_elapsedTime;

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
    RECOVER_FROM_BAD_NOTE_PICKUP,
    DEBUG_HOLD;
  }

  private Repetoire m_nowPlaying;
  private Repetoire m_pendingNote;

  private double m_currentMasterArmSetpoint;
  private double m_maNotePickupPosSetpoint = MAC.NOTE_PICKUP_POS;
  private int m_currentSeqNo;               // ...SeqNo's are used to step through m_nowPlaying SubStates
  private int m_pendingSeqNo;
  private boolean m_isDistantSpeakerShot;   // flag keeps track of which speaker shot is called for
  private boolean m_isSafeToReturn;
  private boolean m_noWaitToScore;
  private boolean m_armsAreReadyToShoot;
  private double m_avgMasterRawAbsPos;
  
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
  public FileRecorder m_fileRecorder = LOGGING_ON ? new FileRecorder("NoteData") : null;
  public InnerArmSubsystem m_innerArmSubsystem = new InnerArmSubsystem(()-> getCurrentStateName(),
                                                                       ()-> getCurrentSeqNo(),
                                                                       m_fileRecorder);
  public IntakeSubsystem   m_intakeSubsystem  = new IntakeSubsystem(()-> getCurrentStateName(),
                                                                    ()-> getCurrentSeqNo(),
                                                                    m_fileRecorder);
  public ShooterSubsystem  m_shooterSubsystem = new ShooterSubsystem(()-> getCurrentStateName(),
                                                                     ()-> getCurrentSeqNo(),
                                                                     m_fileRecorder);

  // Local motors and sensors 
  private TalonFX m_masterArmMotor = new TalonFX(MAC.MASTER_ARM_FALCON_ID, Constants.CANIVORE_BUS_NAME);
  private CANcoder m_masterArmEncoder = new CANcoder(MAC.MASTER_ARM_ENCODER_ID, Constants.CANIVORE_BUS_NAME);
  private final MotionMagicVoltage m_masterArmMotionMagicCtrl = 
                                 new MotionMagicVoltage(0.0).withSlot(0);

  // Network Table entries for publishing MasterArmSubsystem data
  private GenericEntry m_absAxlePosEntry;
  private GenericEntry m_falconRotorPosEntry;
  private GenericEntry m_armSetpointPosEntry;
  private GenericEntry m_armNotePickupPosEntry;
  private GenericEntry m_armPIDOutEntry;
  private GenericEntry m_AxleVelocityEntry;
  private GenericEntry m_falconAmpsEntry;
  /*
  private GenericEntry m_masterArmSetpoint1Entry;
  private GenericEntry m_masterArmSetpoint2Entry;
  private GenericEntry m_innerArmSetpoint1Entry;
  private GenericEntry m_innerArmSetpoint2Entry;
 */

  private GenericEntry m_playingNowEntry;
  private GenericEntry m_pendingNoteEntry;
  private GenericEntry m_seqNoEntry;
  private GenericEntry m_pendingSeqNoEntry;
  private GenericEntry m_isDistantShotEntry;

  private double m_positionError;

  /******************************************************
   * Constructor for a new MasterArmSubsystem. 
   ******************************************************/
  public MasterArmSubsystem() {
    configAbsMasterArmCANCoder();
    configMasterArmMotor();
  //  m_intakeSubsystem.setDefaultCommand(DefaultIntakeCmd(m_intakeSubsystem, ()->RobotContainer.getHidXboxCtrl().getLeftY()));
    changeNoteStateTo(Repetoire.NOTE_HANDLER_IDLE);
    m_pendingNote = m_nowPlaying;
    m_innerArmSubsystem.gotoVerticalPos();
    m_isDistantSpeakerShot = false;
    m_noWaitToScore = false;
    m_isSafeToReturn = false;
    setupMasterArmPublishing();
    m_avgMasterRawAbsPos = getAbsMasterArmPos() - MAC.MASTER_ARM_ENCODER_MAGNET_OFFSET;
    gotoPosition(MAC.INDEXED_SPEAKER_SHOT_POS);
  }

  /***********************************************
   * State status access methods
   ***********************************************/

  public String getCurrentStateName() {
    return m_nowPlaying.toString();
  }

  public int getCurrentSeqNo() {
    return m_currentSeqNo;
  }

  /**************************************************************
   * Higher level Note handling kickoff and state change methods
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
      // Normal state for this method to be called in, but if called from Cancel,
      // assume a note might still be present, set intake accordingly, and exit
      changeNoteStateTo(Repetoire.RETRIEVE_NOTE);
      m_intakeSubsystem.holdNote();
      return;
    }
    if (m_nowPlaying == Repetoire.PREP_TO_GET_NOTE) {
      // Normal state for this method to be call from Cancel
      // If prep has progressed past bumpers, do normal retrieve,
      if (m_currentSeqNo > 2) {
        changeNoteStateTo(Repetoire.RETRIEVE_NOTE);
        m_intakeSubsystem.holdNote();
      } else {
        // else turn off intake (just to be sure it is off) and set position
        // to stay within bumpers (inner arms should still be vertical), then exit.
        m_innerArmSubsystem.gotoSpeakerShotPos(false);
        gotoPosition(MAC.INDEXED_SPEAKER_SHOT_POS);
        changeNoteStateTo(Repetoire.NOTE_HANDLER_IDLE);
        m_intakeSubsystem.cancelIntake();
      }
      // either way, exit now
      return;
    }
    // Else rumble an error code.  retrieveNote is only caled due to actual or  
    // simulated sensor input, or upon other expected state changes, so 
    // should be well vetted, but just in case...
    new RumbleCmd(2, .5, 200).schedule();
    System.out.println("RetrieveNote @ NST State = "+m_nowPlaying.toString());
  }

  public void prepForAmpScore() {
    if (m_nowPlaying == Repetoire.WAIT_FOR_SPECIFIED_GOAL) {
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
        (m_nowPlaying == Repetoire.NOTE_HANDLER_IDLE)) {  // Allow for Unknown Note status
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

  public void prepForDistantSpeakerScore() {
    if ((m_nowPlaying == Repetoire.WAIT_FOR_SPECIFIED_GOAL)
        ||
        (m_nowPlaying == Repetoire.NOTE_HANDLER_IDLE)) {  // Allow for Unknown Note status
      changeNoteStateTo(Repetoire.PREP_FOR_SPEAKER_GOAL);
      m_isDistantSpeakerShot = true;
      // Ensure the m_noWaitToScore flag is cleared here for normal use. 
      // If it needs to be set, do so after this method is called.
      m_noWaitToScore = false;
    } else {
      new RumbleCmd(2, .5, 200).schedule();
      System.out.println("PrepForDistantSpeaker @ NST State = "+m_nowPlaying.toString());
    }
  }

  public void scoreNote() {
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
      System.out.println("PrepForDistantSpeaker @ NST State = "+m_nowPlaying.toString());
    }
  }

  public void discardNote() {
    // Previously, this method woudl try to get rid of a note in any state, and
    // then park the arms inside the robot.
    // Now, it will be limited to ejecting a note that has been picked up awkwardly -
    // e.g. wrinkled, by only one finger instead of two, etc., and the intake will
    // be left deployed, so that the operator can quickly re-try to acquire the Note.
    switch(m_nowPlaying) {
      case WAIT_FOR_NOTE:
        m_intakeSubsystem.ejectNote();
        // Change state to RECOVER_FROM_BAD_NOTE_PICKUP so that the intake
        // can be reactivated once the ejection completes.
        changeNoteStateTo(Repetoire.RECOVER_FROM_BAD_NOTE_PICKUP);
        break;
        
      // In the following two cases the arms are in motion. As long as the arms are 
      // outside of the robot, go ahead and start the intakeSubsystem eject 
      // anyway, in case that helps upblock a jam. At the end of the arm motion(s),
      // the MasterArmSequencer should take care of any related intake action needed.
      case PREP_TO_GET_NOTE:
        if (m_currentSeqNo >= 3) {
          // Now outide the robot frame. Start eject,but do not change state
          // When seqNo reaches the endpoint, m_intakeSubsystem.AcquireNote()
          // will be called and it will override the ejection.
          m_intakeSubsystem.ejectNote();
        }
        break;

      case RETRIEVE_NOTE:
        if (m_currentSeqNo <= 4) {
          // Not yet within the robot frame. Eject now, but again, no state change.
          m_intakeSubsystem.ejectNote();
        } 
        break;

      default:
        // Nothing to do except for the above states
        break;
    }
  }

  public void cancelNoteAction() {
    switch(m_nowPlaying) {
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
        // Afterwards the no wait eject, still require that operator
        // trigger the arm return when safe, rather than automatically.
        m_noWaitToScore = true;
        break;
      
      case WAIT_TO_SCORE_AMP:
        // no ideal way to recover. Best is just to dump the Note,
        // then let operator trigger a return when safe
        m_noWaitToScore = true;
        changeNoteStateTo(Repetoire.SCORE_AMP);
        break;

      case RETRIEVE_NOTE:
      case SCORE_SPEAKER:
      case SCORE_AMP:
      case RETURN_FROM_AMP:
        // do nothing, let whichever process is active finish normally
        break;

      case RECOVER_FROM_BAD_NOTE_PICKUP:
        // TBD. Fow now, do nothing.
      case DEBUG_HOLD:
      default:
        // Nothing to do
        break;
    }
  }

  /*********************************************
   * Status utilities to assist with Autonomous 
   *********************************************/
  public boolean isIdle() {
    return m_nowPlaying == Repetoire.NOTE_HANDLER_IDLE;
  }

  public boolean isReadyToScoreSpeaker() {
    return m_nowPlaying == Repetoire.WAIT_TO_SCORE_SPEAKER;
  }

  public boolean isReadyToScoreAmp() {
    return m_nowPlaying == Repetoire.WAIT_TO_SCORE_AMP;
  }

  /*******************************************************
   * Arm movement and Shooter Test and Debug support
   *******************************************************/
  /* Comment out test software for competition

  public void setTestAim(boolean isDistantShot) {
    if (RobotState.isEnabled() && RobotState.isTest()) {
      m_shooterSubsystem.toggleShooterAimTest();
    }
  }

  public void toggleTestShooter() {
    if (RobotState.isEnabled() && RobotState.isTest()) {
      m_shooterSubsystem.toggleTestShooter();
    }
  }

  public enum TP_Failures {
    notTestMode,
    invalidTargetArm,
    invalidSetpoint
  }

  public void gotoTestPosition(int whichArm, double setpoint) {
    if (RobotState.isTest()) {
      if (whichArm == 1) {
        if ((setpoint <= MAC.MAX_MASTER_ARM_ROTATIONS) 
            &&
            (setpoint >= MAC.MIN_MASTER_ARM_ROTATIONS))  {
          gotoPosition(setpoint);
        } else {
          ReportFailedSetpointRequest("MA ", setpoint, TP_Failures.invalidSetpoint);
        }
      } else if (whichArm == 2) {
        // No limits on Inner arm, except for logical ones.
        // So, don't let arm spin more than twice.
        if ((setpoint <= 2.0) && (setpoint > -2.0)) {
          m_innerArmSubsystem.gotoPosition(setpoint);
        } else {
          ReportFailedSetpointRequest("IA ", setpoint, TP_Failures.invalidSetpoint);
        }
      } else {
        ReportFailedSetpointRequest("Unknown "+whichArm, setpoint, TP_Failures.invalidTargetArm);      
      }
    } else {
      ReportFailedSetpointRequest("", setpoint, TP_Failures.notTestMode);    
    }
  }
  
  public void ReportFailedSetpointRequest(String whichArm, double setpoint, TP_Failures code) {
    String tempBuffer;
    tempBuffer = whichArm+" Setpoint failed due to "+code.toString()+
                 ". Robot State = "+(RobotState.isEnabled() ? "Enabled, " : "Disabled, ");
    if (RobotState.isTeleop()) {
      tempBuffer = tempBuffer + "In TeleOp. ";
    } else if (RobotState.isAutonomous()) {
      tempBuffer = tempBuffer + "In Autonomous. ";
    } else if (RobotState.isTest()) {
      tempBuffer = tempBuffer + "In Test. ";
    }
    tempBuffer = tempBuffer + "Setpoint = "+setpoint;
    System.out.println(tempBuffer);
  }

  public void gotoMASetpoint1() {
    //gotoTestPosition(1, m_masterArmSetpoint1Entry.getDouble(MAC.MASTER_ARM_HORIZ_POS));
    gotoTestPosition(1, SmartDashboard.getNumber("MA SP1", MAC.MASTER_ARM_HORIZ_POS));
  }

  public void gotoMASetpoint2() {
 //   gotoTestPosition(1, m_masterArmSetpoint2Entry.getDouble(MAC.LOW_SAFE_TO_ROTATE_OUT_POS));
    gotoTestPosition(1, SmartDashboard.getNumber("MA SP2", MAC.LOW_SAFE_TO_ROTATE_OUT_POS));
  }

  public void gotoIASetpoint1() {
    //gotoTestPosition(2, m_innerArmSetpoint1Entry.getDouble(IAC.VERTICAL_POS));
    gotoTestPosition(2, SmartDashboard.getNumber("IA SP1", IAC.VERTICAL_POS));
  }

  public void gotoIASetpoint2() {
    //gotoTestPosition(2, m_innerArmSetpoint2Entry.getDouble(IAC.BUMPER_CONTACT_POS));
    gotoTestPosition(2, SmartDashboard.getNumber("IA SP2", IAC.HORIZONTAL_FORWARD_POS));
  }
  */

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
    System.out.println("MA Setpoint ="+setpoint);
  }

  /************************************************************************
   * Utilities for slightly adjusting position of MasterArm for use
   * when developing/tweaking standard setpoints. Perhaps potentially
   * useful during match play if the robot gets out of alignment, but that 
   * would be an emergency use, very inefficient.
   * All take positions in Rotation units.
   * Redundancies in setting m_currentMasterArmSetpoint is a side effect of
   * using the normal gotoPosition() routine after adjusting the setpoint,
   * and sharing the SW LimitSwitch check.
   ************************************************************************/
  public double limitMasterArmPos(double position) {
    if (position > MAC.MAX_MASTER_ARM_ROTATIONS) {
      return MAC.MAX_MASTER_ARM_ROTATIONS;
    } else if (position < MAC.MIN_MASTER_ARM_ROTATIONS) {
      return MAC.MIN_MASTER_ARM_ROTATIONS;
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
    // TODO - fix bug - this is being called on button presses, 
    // but setpoint is not changing. Works for the inneer arm, but 
    // not the MasterArm.
    // Is something resetting the action each time, or ?? 

    // If DEBUG_ON, allow incrementing/decrementing always, and
    // apply it to the current setpoint.
    // Otherwise, only allow incrementing/decrementing when in
    // WAIT_FOR_NOTE state, and apply the adjustment to a variable
    // holding the Note pickup position setpoint, which adjustment
    // will persist until the next re-boot. This applies for both
    // inner and Master Arms.
    if (Math.abs(direction) != 1.0) {
      System.out.println("MA adjust ignored - invalid dir arg "+direction);
    } else if (DEBUG_ON) {
      m_currentMasterArmSetpoint = m_currentMasterArmSetpoint + (1 / 360) * direction;
      m_currentMasterArmSetpoint = limitMasterArmPos(m_currentMasterArmSetpoint);
      gotoPosition(m_currentMasterArmSetpoint);
      System.out.println("Adjusting all MA setpoints in Debug");
    } else if (m_nowPlaying == Repetoire.WAIT_FOR_NOTE) {
        m_maNotePickupPosSetpoint = m_maNotePickupPosSetpoint + (1 / 360) * direction;
        m_maNotePickupPosSetpoint = limitMasterArmPos(m_maNotePickupPosSetpoint);
        gotoPosition(m_maNotePickupPosSetpoint);
        System.out.println("Adjust MA Note Pickup Setpoint "+ m_maNotePickupPosSetpoint);
    } else {
      System.out.println("Adjust MA req ignored: not DEBUG_ON or WAIT_FOR_NOTE");
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
    } else if (DEBUG_ON || m_nowPlaying == Repetoire.WAIT_FOR_NOTE) {
        m_innerArmSubsystem.adjustInnerArmSetpoint(direction, DEBUG_ON);
        System.out.println("m_innerArmSubsystem.adjust... called");
    } else {
      System.out.println("IA adjust ignored: not DEBUG_ON or WAIT_FOR_NOTE");
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
      m_fileRecorder.recordMoveEvent( "MA",
                                      NoteEvent.TIMEOUT_OCCURED,
                                      position,
                                      m_positionError,
                                      System.currentTimeMillis(),
                                      m_elapsedTime,
                                      m_nowPlaying.toString(),
                                      m_currentSeqNo);
      return true;
    }
  }

  /*******************************************************************
   * Setup and Config routines
   ******************************************************************/
  private void configMasterArmMotor() {
    /*
    var openLoopConfig = new OpenLoopRampsConfigs().withDutyCycleOpenLoopRampPeriod(0)
                                                   .withVoltageOpenLoopRampPeriod(MAC.MASTER_ARM_OPEN_LOOP_RAMP_PERIOD)
                                                   .withTorqueOpenLoopRampPeriod(0);
    */
    var closedLoopConfig = new ClosedLoopRampsConfigs().withDutyCycleClosedLoopRampPeriod(0)
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
                                                    //.withDutyCycleNeutralDeadband(.001);
    var currentLimitConfig = new CurrentLimitsConfigs().withSupplyCurrentLimit(MAC.MASTER_ARM_CONT_CURRENT_LIMIT)
                                                       .withSupplyCurrentThreshold(MAC.MASTER_ARM_PEAK_CURRENT_LIMIT)
                                                       .withSupplyTimeThreshold(MAC.MASTER_ARM_PEAK_CURRENT_DURATION)
                                                       .withSupplyCurrentLimitEnable(MAC.MASTER_ARM_ENABLE_CURRENT_LIMIT);
    var swLimitConfig = new SoftwareLimitSwitchConfigs().withForwardSoftLimitEnable(true)
                                                        .withForwardSoftLimitThreshold(MAC.MAX_MASTER_ARM_ROTATIONS)
                                                        .withReverseSoftLimitEnable(true)
                                                        .withReverseSoftLimitThreshold(MAC.MIN_MASTER_ARM_ROTATIONS);
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
                                                    //.withOpenLoopRamps(openLoopConfig)
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

  // Called on ALT+Start button press. BE SURE calibration stick is in place
  // before pressing!
  public void resetInnerArmMagnetOffset() {
    m_innerArmSubsystem.resetMagnetOffset();
  }

  /********************************************************************
   * Publishing setup and operation
   ********************************************************************/
  private void setupMasterArmPublishing() {
    // Get or establish the "Note Handlers" tab
    ShuffleboardTab sbt = Shuffleboard.getTab("Note Handlers");

/*
    Only uncomment the following when testing closed loop setpoint operations

    ShuffleboardLayout sbTestSetpointsLayout = 
                                  sbt.getLayout("Test Setpoints", BuiltInLayouts.kList)
                                     .withPosition(NotableConstants.TEST_SETPOINTS_DATA_COL,
                                                   NotableConstants.TEST_SETPOINTS_DATA_ROW)
                                     .withSize(2, NotableConstants.TEST_SETPOINTS_DATA_LIST_HGT)
                                     .withProperties(Map.of("Label position", "LEFT"));

    m_masterArmSetpoint1Entry = sbTestSetpointsLayout.add("MA Test SP1", MAC.MASTER_ARM_HORIZ_POS).getEntry();
    m_masterArmSetpoint2Entry = sbTestSetpointsLayout.add("MA Test SP2", MAC.HIGH_SAFE_TO_ROTATE_DOWN_AND_OUT_POS).getEntry();
    m_innerArmSetpoint1Entry  = sbTestSetpointsLayout.add("IA Test SP1", IAC.HORIZONTAL_FORWARD_POS).getEntry();
    m_innerArmSetpoint2Entry  = sbTestSetpointsLayout.add("IA Test SP2", IAC.VERTICAL_POS).getEntry();
*/
    // Setup the "Master Arm" data list
    ShuffleboardLayout sbLayout = sbt.getLayout("Master Arm", BuiltInLayouts.kList)
                                      .withPosition(MAC.MASTER_ARM_DATA_COL,
                                                    MAC.MASTER_ARM_DATA_ROW)
                                      .withSize(2, MAC.MASTER_ARM_DATA_LIST_HGT)
                                      .withProperties(Map.of("Label position", "LEFT"));
    sbLayout.add("Ids ", F.df1.format(MAC.MASTER_ARM_FALCON_ID)+"  "+F.df1.format(MAC.MASTER_ARM_ENCODER_ID));
    sbLayout.add("Offset", F.df1.format(MAC.MASTER_ARM_ENCODER_MAGNET_OFFSET));
    m_absAxlePosEntry     = sbLayout.add("Abs Pos", "0").getEntry();
    m_falconRotorPosEntry = sbLayout.add("Motor Pos", "0").getEntry();
    m_armSetpointPosEntry = sbLayout.add("SetPt Pos", "0").getEntry();
    m_armNotePickupPosEntry = sbLayout.add("Note PU", "0").getEntry(); 
    m_armPIDOutEntry      = sbLayout.add("PID Out", "0").getEntry();
    m_AxleVelocityEntry   = sbLayout.add("Axle Vel", "0").getEntry();
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
    m_absAxlePosEntry.setString(F.df3.format(getAbsMasterArmPos()));
    m_falconRotorPosEntry.setString(F.df3.format(m_masterArmMotor.getPosition().getValueAsDouble()));
    m_armSetpointPosEntry.setString(F.df3.format(m_currentMasterArmSetpoint));
    m_armNotePickupPosEntry.setString(F.df3.format(m_maNotePickupPosSetpoint));
    m_armPIDOutEntry.setString(F.df3.format(m_masterArmMotor.getClosedLoopOutput().getValueAsDouble()));
    m_AxleVelocityEntry.setString(F.df3.format(m_masterArmEncoder.getVelocity().getValueAsDouble()));
    m_falconAmpsEntry.setString(F.df3.format(m_masterArmMotor.getSupplyCurrent().getValueAsDouble()));
    m_playingNowEntry.setString(m_nowPlaying.toString());
    m_pendingNoteEntry.setString(m_pendingNote.toString());
    m_seqNoEntry.setInteger(m_currentSeqNo);
    m_pendingSeqNoEntry.setInteger(m_pendingSeqNo);
    m_isDistantShotEntry.setString(m_isDistantSpeakerShot ? "Yes" : "No");

    // Shuffleboard is notoriously fickle, sometimes stop live data updates.
    // Post critical data to SmartDashboard in parallel
    // SmartDashboard.putNumber("MasterEncRelPos ", m_masterArmEncoder.getPosition().getValueAsDouble());
    m_avgMasterRawAbsPos = ((m_avgMasterRawAbsPos * .95) + ((getAbsMasterArmPos()-MAC.MASTER_ARM_ENCODER_MAGNET_OFFSET) * .05));
    SmartDashboard.putNumber("MasterAvgRawAbsPos ", m_avgMasterRawAbsPos);
    SmartDashboard.putNumber("MasterCorrAbsPos ", getAbsMasterArmPos());
    SmartDashboard.putNumber("MasterMotorPos ", m_masterArmMotor.getPosition().getValueAsDouble());
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
   * Sequences not just the master arm movements, but the entire suite of
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
        if (m_intakeSubsystem.isNoteAcquired()) {
          retrieveNote();
        };
        break;

      case RETRIEVE_NOTE:
        processRetrievingNote();
        break;

      case WAIT_FOR_SPECIFIED_GOAL:
        // nothing to do - will exit via button press instant command
        break;

      case PREP_FOR_SPEAKER_GOAL:
        processSpeakerScorePrep();
        break;

      case WAIT_TO_SCORE_SPEAKER:
        if (m_noWaitToScore) {
          changeNoteStateTo(Repetoire.SCORE_SPEAKER);
          m_noWaitToScore = false;
        }
        // else nothing to do - will exit via button press instant command
        break;

      case SCORE_SPEAKER:
        processScoringSpeaker();
        break;

      case PREP_FOR_AMP_GOAL:
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

      case RECOVER_FROM_BAD_NOTE_PICKUP:
        // No change of state or seqNo needed here - just wait for eject to
        // complete, then restart intake belts
        if (m_intakeSubsystem.isIntakeStopped()) {
          m_intakeSubsystem.acquireNote();
          changeNoteStateTo(Repetoire.WAIT_FOR_NOTE);
        }
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
      m_fileRecorder.recordStateChange( System.currentTimeMillis(),
                                        m_nowPlaying.toString(),
                                        newState.toString(),
                                        m_currentSeqNo);
      m_nowPlaying = newState;
    } else {
      m_fileRecorder.recordStateChange( System.currentTimeMillis(),
                                        m_nowPlaying.toString(),
                                        Repetoire.DEBUG_HOLD.toString(),
                                        m_currentSeqNo);
      m_pendingNote = newState;
      m_nowPlaying = Repetoire.DEBUG_HOLD;
    }
    resetSeqNo();   
  }

  private void changeSeqNoTo(int newSeqNo) {
    if ((! DEBUG_ON) || (m_currentSeqNo == 99)) {
      m_fileRecorder.recordSeqNoChange( System.currentTimeMillis(),
                                        m_nowPlaying.toString(),
                                        m_currentSeqNo,
                                        newSeqNo );
      m_currentSeqNo = newSeqNo;
    } else {
      m_fileRecorder.recordSeqNoChange( System.currentTimeMillis(),
                                        m_nowPlaying.toString(),
                                        m_currentSeqNo,
                                        99);
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
        m_innerArmSubsystem.gotoVerticalPos();          // Set innerArm moving, then wait for it
        changeSeqNoTo(2);
        break;

      case 2:
        if (m_innerArmSubsystem.innerArmIsVertical()) {
          System.out.println("MA Received a True response");  
          gotoPosition(MAC.LOW_SAFE_TO_ROTATE_OUT_POS);   // Set masterArm moving, then wait for it
          changeSeqNoTo(3);
        } else {
          System.out.println("MA Recieved a False response");
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
          gotoPosition(m_maNotePickupPosSetpoint);       // set masterArm moving to pickup Pos, then wait for it
          changeSeqNoTo(5);
        }
        break;

      case 5:
        if (isMasterArmAt(m_maNotePickupPosSetpoint, MAC.ALLOWED_MILLIS_MA_SMALL_MOVE)) {
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

      case 99:            // Debug hold coded SeqNo
        break;

      default:
          System.out.println("Invalid SeqNo in processNoteAcquisitionPrep: "+m_currentSeqNo);
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
          m_innerArmSubsystem.gotoVerticalPos();                // set innerArm moving, then wait for it
          changeSeqNoTo(4);
        }
        break;

      case 4:
        if (m_innerArmSubsystem.innerArmIsVertical()) {
          gotoPosition(MAC.LOW_SAFE_TO_ROTATE_IN_POS);          // set masterArm moving, then wait for it
          changeSeqNoTo(5);
        }
        break;

      case 5:
        if (isMasterArmAt(MAC.LOW_SAFE_TO_ROTATE_IN_POS, MAC.ALLOWED_MILLIS_MA_SMALL_MOVE)) {
          //m_innerArmSubsystem.gotoSpeakerShotPos(false);  // set innerArm moving, then wait for it
          // We could eliminate this step now that default inner arm position is VERTICAL,
          // but leave it in case we revert to indexed speaker shot position
          m_innerArmSubsystem.gotoVerticalPos();            // set innerArm moving, then wait for it
          changeSeqNoTo(6);
        }
        break;

      case 6:
        // if (m_innerArmSubsystem.innerArmIsAtIndexedSpeakerPos()) {
        if (m_innerArmSubsystem.innerArmIsVertical()) {
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

  /*****************************************
   * PROCESS SPEAKER SCORE PREP
   *****************************************/
  private void processSpeakerScorePrep() {
    switch(m_currentSeqNo) {
      case 1:
        // in this instance, we can do a lot in parallel, then wait for all to complete
        m_shooterSubsystem.prepareToShoot(m_isDistantSpeakerShot);
        m_innerArmSubsystem.gotoSpeakerShotPos(m_isDistantSpeakerShot);
        if (m_isDistantSpeakerShot) {
          gotoPosition(MAC.DISTANT_SPEAKER_SHOT_POS);
        } else {
          gotoPosition(MAC.INDEXED_SPEAKER_SHOT_POS);
        }
        changeSeqNoTo(2);
        m_startTime = System.currentTimeMillis();
        break;

      case 2:
        // If shooter system is ready, and both Arms are ready, advance NST State. Note that
        // all of the polled systems have their own timeouts to ensure no infinite hangups
        // during a match, but just in case check for a 2 sec timeout in this seqNo.
        if (m_isDistantSpeakerShot) {
          m_armsAreReadyToShoot = m_innerArmSubsystem.innerArmIsAtDistantSpeakerPos()
                                  &&
                                  isMasterArmAt(MAC.DISTANT_SPEAKER_SHOT_POS, MAC.ALLOWED_MILLIS_MA_SMALL_MOVE);
        } else {
          m_armsAreReadyToShoot = m_innerArmSubsystem.innerArmIsAtIndexedSpeakerPos()
                                  &&
                                  isMasterArmAt(MAC.INDEXED_SPEAKER_SHOT_POS, MAC.ALLOWED_MILLIS_MA_SMALL_MOVE);
        }
        m_elapsedTime = System.currentTimeMillis() - m_startTime;
        if ((m_shooterSubsystem.isReadyToShoot() && m_armsAreReadyToShoot)
            ||
            (m_elapsedTime > 2000)) {
          if (m_noWaitToScore) {
            changeNoteStateTo(Repetoire.SCORE_SPEAKER); 
          } else {
            changeNoteStateTo(Repetoire.WAIT_TO_SCORE_SPEAKER); 
          } 
        }          
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
    switch(m_currentSeqNo) {
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
          changeNoteStateTo(Repetoire.NOTE_HANDLER_IDLE);
          m_shooterSubsystem.cancelShooter();
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
   switch(m_currentSeqNo) {
      case 1: 
        m_innerArmSubsystem.gotoVerticalPos();                // set innerArm moving, then wait for it
        changeSeqNoTo(2);
        break;

      case 2:
        if (m_innerArmSubsystem.innerArmIsVertical()) {
          gotoPosition(MAC.LOW_SAFE_TO_ROTATE_OUT_POS);          // set masterArm moving, then wait for it
          changeSeqNoTo(3);
        }
        break;

      case 3:
        if (isMasterArmAt(MAC.LOW_SAFE_TO_ROTATE_OUT_POS, MAC.ALLOWED_MILLIS_MA_SMALL_MOVE)) {
          m_innerArmSubsystem.gotoHorizontalBackPos();            // set innerArm moving, then wait for it
          changeSeqNoTo(4);
        }
        break;

      case 4:
        if (m_innerArmSubsystem.innerArmIsHorizontalBackPos()) {
          gotoPosition(MAC.HIGH_SAFE_TO_ROTATE_DOWN_AND_OUT_POS);    // set masterArm moving, then wait for it
          changeSeqNoTo(5);
        }
        break;

      case 5:
        if (isMasterArmAt(MAC.HIGH_SAFE_TO_ROTATE_DOWN_AND_OUT_POS, MAC.ALLOWED_MILLIS_MA_LARGE_MOVE)) {
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
          changeNoteStateTo(Repetoire.WAIT_TO_SCORE_AMP);
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
    switch(m_currentSeqNo) {
      case 1: 
        m_intakeSubsystem.ejectNote();
        changeSeqNoTo(2);
        m_startTime = System.currentTimeMillis();
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
   switch(m_currentSeqNo) {
    case 1:
      if (m_isSafeToReturn) {
        m_innerArmSubsystem.gotoHorizontalBackPos();
        changeSeqNoTo(2);
      }
      break;

    case 2:
      if (m_innerArmSubsystem.innerArmIsHorizontalBackPos()) {
        gotoPosition(MAC.LOW_SAFE_TO_ROTATE_OUT_POS);
        changeSeqNoTo(3);
      }
      break;

    case 3:
      if (isMasterArmAt(MAC.LOW_SAFE_TO_ROTATE_OUT_POS, MAC.ALLOWED_MILLIS_MA_LARGE_MOVE)) {
        m_innerArmSubsystem.gotoVerticalPos();
        changeSeqNoTo(4);
      }
      break;

    case 4:
      if (m_innerArmSubsystem.innerArmIsVertical()) {
        gotoPosition(MAC.LOW_SAFE_TO_ROTATE_IN_POS);
        changeSeqNoTo(5);
      }
      break;

    case 5:
      if (isMasterArmAt(MAC.LOW_SAFE_TO_ROTATE_IN_POS, MAC.ALLOWED_MILLIS_MA_SMALL_MOVE)) {
        //m_innerArmSubsystem.gotoSpeakerShotPos(false);  // set innerArm moving, then wait for it
        m_innerArmSubsystem.gotoVerticalPos();            // Could delete this step now that innerarm idle
                                                          // position is Vertical, but keep in case
                                                          // of reversion to indexedSpeakerShotPos
        changeSeqNoTo(6);
      }
      break;

    case 6:
      //if (m_innerArmSubsystem.innerArmIsAtIndexedSpeakerPos()) {
      if (m_innerArmSubsystem.innerArmIsVertical()) {
        gotoPosition(MAC.INDEXED_SPEAKER_SHOT_POS);                     // set masterArm moving, then wait for it
        //m_innerArmSubsystem.gotoSpeakerShotPos(m_isDistantSpeakerShot);
        changeSeqNoTo(7);
      }
      break;

    case 7:
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
   * Note Sensor simulation support method
   * ****************************************************/
  public void simulateNoteAcquired() {
    m_intakeSubsystem.simulateNoteAcquired();
    if (m_nowPlaying == Repetoire.WAIT_FOR_NOTE) {
      retrieveNote();
    }
  }
}

