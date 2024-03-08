// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Map;
import java.util.function.IntSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;

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
import frc.robot.Constants.F;
import frc.robot.NotableConstants.IAC;

public class InnerArmSubsystem extends SubsystemBase {
  private TalonFX m_innerArmMotor = new TalonFX(IAC.INNER_ARM_FALCON_ID, Constants.CANIVORE_BUS_NAME);
  private CANcoder m_innerArmCANcoder = new CANcoder(IAC.INNER_ARM_CANCODER_ID, Constants.CANIVORE_BUS_NAME);

  private double m_innerArmSetpoint;
  private double m_innerArmNotePickupPosSetpoint;
  private double m_avgInnerRawAbsPos;
  private double m_avgRawHomePos;
  private double m_magnetOffset = IAC.INNER_ARM_CANCODER_MAGNET_OFFSET;
  private int m_count = 0;
  private boolean m_isDistantSpeakerShot = false;

  private final MotionMagicVoltage m_innerArmMagicCtrl = new MotionMagicVoltage(0.0)
                                                                                .withSlot(0)
                                                                                .withEnableFOC(true);
  private long m_startTime;
  private long m_absPosMeasurementCount = 0;

  private GenericEntry m_magnetOffsetEntry;
  private GenericEntry m_absAxlePosEntry;
  private GenericEntry m_falconPosEntry;
  private GenericEntry m_armSetpointEntry;
  private GenericEntry m_armPIDOutEntry;
  private GenericEntry m_axleVelocityEntry;
  private GenericEntry m_falconAmpsEntry;

  private Supplier<String> m_currentStateName;
  private IntSupplier m_currentSeqNo;
  private double m_positionError;
  private long m_elapsedTime;
  private FileRecorder m_fileRecorder;

  /** Creates a new InnerArmSubsystem. */
  public InnerArmSubsystem(Supplier<String> currentStateName, 
                           IntSupplier currentSeqNo,
                           FileRecorder fileRecorder) {
    m_currentStateName = currentStateName;
    m_currentSeqNo = currentSeqNo;
    m_fileRecorder = fileRecorder;
    configInnerArmCANcoder();
    configInnerArmMotor();
    setupInnerArmPublishing();
    // Set boot-up inner arm setpoint to the safest possible position
    m_innerArmSetpoint = IAC.VERTICAL_POS;
    // Transfer default NOTE_PICKUP_POS to a variable so if needed it can
    // be adjusted at runtime.
    m_innerArmNotePickupPosSetpoint = IAC.NOTE_PICKUP_POS;
    // Initialize avg Inner Arm Cancoder value
    m_avgInnerRawAbsPos = getRawInnerArmPos();
  }

  /************************************************************************
   * Utilities for slightly adjusting position of innerArm (assumed static)
   ************************************************************************/
  public double limitInnerArmPosition(double position) {
    if (position > IAC.MAX_INNER_ARM_LIMIT) {
      return(IAC.MAX_INNER_ARM_LIMIT);
    } else if (position < IAC.MIN_INNER_ARM_LIMIT) {
      return IAC.MIN_INNER_ARM_LIMIT;
    } else {
      return position;
    }
  }

  public void adjustInnerArmSetpoint(double direction, boolean debug_on) {
    // direction, debug, and/or current state (WAITING_FOR_NOTE, if not debug)
    // have already been vetted by MasterArmSubsystem
    double temp;
    if (debug_on) {
      m_innerArmSetpoint = m_innerArmSetpoint + (1.0 / 360.0) * direction;
      m_innerArmSetpoint = limitInnerArmPosition(m_innerArmSetpoint);
      gotoPosition(m_innerArmSetpoint);
    } else {
      temp = m_innerArmNotePickupPosSetpoint + (1/360) * direction;
      System.out.println("IA temp pos = "+temp);
      m_innerArmNotePickupPosSetpoint = limitInnerArmPosition(temp);
      gotoPosition(m_innerArmNotePickupPosSetpoint);
      System.out.println("Adjust IA Note Pickup "+m_innerArmNotePickupPosSetpoint);    
    }
  }

  /*********************************
   * check for reaching setpoints
   *********************************/

  public boolean innerArmIsVertical() {
    return innerArmIsAt(IAC.VERTICAL_POS, IAC.ALLOWED_MILLIS_IA_LARGE_MOVE);
  }

  public boolean innerArmIsAtBumperContactPos() {
    return innerArmIsAt(IAC.BUMPER_CONTACT_POS, IAC.ALLOWED_MILLIS_IA_LARGE_MOVE);
  }

  public boolean innerArmIsAtPickupPos() {
    return innerArmIsAt(IAC.NOTE_PICKUP_POS, IAC.ALLOWED_MILLIS_IA_SMALL_MOVE);
  }

  public boolean innerArmIsAtDistantSpeakerPos() {
    return innerArmIsAt(IAC.DISTANT_SPEAKER_GOAL_POS, IAC.ALLOWED_MILLIS_IA_SMALL_MOVE);
  }
      
  public boolean innerArmIsAtIndexedSpeakerPos() {
    return innerArmIsAt(IAC.INDEXED_SPEAKER_GOAL_POS, IAC.ALLOWED_MILLIS_IA_SMALL_MOVE);
  }

  public boolean innerArmIsHorizontalBackPos() {
    return innerArmIsAt(IAC.HORIZONTAL_BACK_POS, IAC.ALLOWED_MILLIS_IA_LARGE_MOVE);
  }
  
  public boolean innerArmIsAtAmpScoringPos() {
    return innerArmIsAt(IAC.AMP_GOAL_POS, IAC.ALLOWED_MILLIS_IA_LARGE_MOVE);
  }

  // getAbsInnerArmPos returns the InnerArmEncoder sensor position
  // which is in absolute rotations, with origin of 0 when horizontal,
  // already corrected for magnet offset (the offset must be measured,
  // then stored in NotableConstants.java).
  public double getAbsInnerArmPos() {
    return(m_innerArmCANcoder.getAbsolutePosition().getValueAsDouble());
  }

  public boolean innerArmIsAt(double position, long timeoutDuration) {
    m_positionError = position - getAbsInnerArmPos();
    m_elapsedTime = System.currentTimeMillis() - m_startTime;

    if (Math.abs(m_positionError) < IAC.ALLOWED_INNER_ARM_POS_ERROR) {
      m_fileRecorder.recordMoveEvent("IA",
                                     NoteEvent.SETPOINT_REACHED,
                                     position,
                                     m_positionError,
                                     System.currentTimeMillis(),
                                     m_elapsedTime,
                                     m_currentStateName.get(),
                                     m_currentSeqNo.getAsInt());
      return true;
    }

    if (m_elapsedTime <= timeoutDuration) {
      // general timeout has not yet expired.
        return false;
    } else {
      // timeout has occured. Log event and force assume inner arm is at setpoint 
      // in order to avoid "hanging" the state machine (hopefully it is close enough).
      m_fileRecorder.recordMoveEvent( "IA",
                                      NoteEvent.TIMEOUT_OCCURED,
                                      position,
                                      m_positionError,
                                      System.currentTimeMillis(),
                                      m_elapsedTime,
                                      m_currentStateName.get(),
                                      m_currentSeqNo.getAsInt() );
      return true;
    }
  }
  
  /*****************************************
   * goto<various inner arm positions>
   * Called only from Master Arm sequencer.
   * All make use of support routine gotoPosition(),
   * which also stores the current setpoint, 
   * and initializes a motion start time
   * for timeout purposes.
   * Position units are always Rotations
   * (1 rotation == 360 degrees). InnerArm
   * ratio is 20:18 CANcoder:Axle.
   *****************************************/
  public void gotoPosition(double position) {
    m_innerArmSetpoint = position;
    m_innerArmMotor.setControl(m_innerArmMagicCtrl.withPosition(m_innerArmSetpoint));
    m_startTime = System.currentTimeMillis();
    m_fileRecorder.recordReqEvent("IA",
                                  NoteRequest.MOVE_INNER_ARM,
                                  m_innerArmSetpoint,
                                  m_startTime,
                                  m_currentStateName.get(),
                                  m_currentSeqNo.getAsInt());
  }

  public void gotoVerticalPos() {
    gotoPosition(IAC.VERTICAL_POS);
  }

  public void gotoBumperContactPos() {
    gotoPosition(IAC.BUMPER_CONTACT_POS);
  }

  public void gotoNotePickupPos() {
    gotoPosition(m_innerArmNotePickupPosSetpoint);
  }

  public void gotoSpeakerShotPos(boolean isDistantSpeakerShot) {
    m_isDistantSpeakerShot = isDistantSpeakerShot;
    if (m_isDistantSpeakerShot) {
      gotoPosition(IAC.DISTANT_SPEAKER_GOAL_POS);
    } else {
      gotoPosition(IAC.INDEXED_SPEAKER_GOAL_POS);
    }
  }

  public void gotoHorizontalBackPos() {
    gotoPosition(IAC.HORIZONTAL_BACK_POS);
  }
  
  public void gotoAmpShotPos() {
    gotoPosition(IAC.AMP_GOAL_POS);
  }

  /****************************************
   * Configure Hardware methods 
   ****************************************/
  private void configInnerArmMotor() {
    /*
    var openLoopConfig = new OpenLoopRampsConfigs().withDutyCycleOpenLoopRampPeriod(0)
                                                    .withVoltageOpenLoopRampPeriod(IAC.INNER_ARM_OPEN_LOOP_RAMP_PERIOD)
                                                    .withTorqueOpenLoopRampPeriod(0);
    */
    var closedLoopConfig = new ClosedLoopRampsConfigs().withDutyCycleClosedLoopRampPeriod(0)
                                                        .withVoltageClosedLoopRampPeriod(IAC.INNER_ARM_CLOSED_LOOP_RAMP_PERIOD)
                                                        .withTorqueClosedLoopRampPeriod(0);
    var feedbackConfig = new FeedbackConfigs().withFeedbackSensorSource(FeedbackSensorSourceValue.FusedCANcoder)
                                              .withFeedbackRemoteSensorID(IAC.INNER_ARM_CANCODER_ID)
                                              .withSensorToMechanismRatio(IAC.INNER_ARM_CANCODER_TO_AXLE_RATIO)
                                              .withRotorToSensorRatio(IAC.INNER_ARM_ROTOR_TO_CANCODER_RATIO);
    var motorOutputConfig = new MotorOutputConfigs().withNeutralMode(IAC.INNER_ARM_MOTOR_NEUTRAL_MODE)
                                                    .withInverted(IAC.INNER_ARM_MOTOR_INVERT)
                                                    .withPeakForwardDutyCycle(IAC.INNER_ARM_OUTPUT_LIMIT_FACTOR)
                                                    .withPeakReverseDutyCycle(-IAC.INNER_ARM_OUTPUT_LIMIT_FACTOR);
                                                    //.withDutyCycleNeutralDeadband(.001);
    var currentLimitConfig = new CurrentLimitsConfigs().withSupplyCurrentLimit(IAC.INNER_ARM_CONT_CURRENT_LIMIT)
                                                       .withSupplyCurrentThreshold(IAC.INNER_ARM_PEAK_CURRENT_LIMIT)
                                                       .withSupplyTimeThreshold(IAC.INNER_ARM_PEAK_CURRENT_DURATION)
                                                       .withSupplyCurrentLimitEnable(IAC.INNER_ARM_ENABLE_CURRENT_LIMIT);
    Slot0Configs pid0Config = new Slot0Configs().withKP(IAC.INNER_ARM_KP)
                                                .withKI(IAC.INNER_ARM_KI)
                                                .withKD(IAC.INNER_ARM_KD)
                                                .withKS(IAC.INNER_ARM_KS)
                                                .withKV(IAC.INNER_ARM_KV)
                                                .withKA(IAC.INNER_ARM_KA)
                                                .withKG(IAC.INNER_ARM_KG).withGravityType(GravityTypeValue.Arm_Cosine);
    MotionMagicConfigs  motionMagicConfig = new MotionMagicConfigs().withMotionMagicCruiseVelocity(IAC.INNER_ARM_MOTION_MAGIC_VEL)
                                                                    .withMotionMagicAcceleration(IAC.INNER_ARM_MOTION_MAGIC_ACCEL)
                                                                    .withMotionMagicJerk(IAC.INNER_ARM_MOTION_MAGIC_JERK)
                                                                    .withMotionMagicExpo_kA(IAC.INNER_ARM_MOTION_MAGIC_kA)
                                                                    .withMotionMagicExpo_kA(IAC.INNER_ARM_MOTION_MAGIC_kV);
    var innerArmConfig = new TalonFXConfiguration().withFeedback(feedbackConfig)
                                                   .withMotorOutput(motorOutputConfig)
                                                   .withCurrentLimits(currentLimitConfig)
                                                   //.withOpenLoopRamps(openLoopConfig)
                                                   .withClosedLoopRamps(closedLoopConfig)
                                                   .withSlot0(pid0Config)
                                                   .withMotionMagic(motionMagicConfig);
    StatusCode status = m_innerArmMotor.getConfigurator().apply(innerArmConfig);

    if (! status.isOK() ) {
      System.out.println("Failed to apply INNER_ARM configs. Error code: "+status.toString());
    }
  }
  
  private void configInnerArmCANcoder() {
    var magnetSensorConfigs = new MagnetSensorConfigs().withAbsoluteSensorRange(IAC.INNER_ARM_CANCODER_RANGE)
                                                       .withSensorDirection(IAC.INNER_ARM_CANCODER_DIR)
                                                       .withMagnetOffset(m_magnetOffset);
    var ccConfig = new CANcoderConfiguration().withMagnetSensor(magnetSensorConfigs);
    StatusCode status = m_innerArmCANcoder.getConfigurator().apply(ccConfig);
    if (! status.isOK() ) {
      System.out.println("Failed to apply INNER_CANcoder configs. Error code: "+status.toString());
    }
  }

  // This method returns the uncorrected CANcoder Absolute position.
  public double getRawInnerArmPos() {
    return (getAbsInnerArmPos()-m_magnetOffset);
  }

  public boolean innerArmHomeSensorIsTripped() {
    return false;
  }

  // This method only works if an index sensor is added to allow InnerArm position to 
  // be known absolutely, but independently of the CANcoder. Comparison of the two
  // would allow checking that they are still in sync - if not, then this routine could be
  // called. As written, this assumes an index sensor is located at the normal
  // HOME, or IDLE position of the inner arm (IAC.VERTICAL_POS), where it spends aa
  // fair amount of time.
  public void checkMagnetOffset() {
    if (innerArmHomeSensorIsTripped()) {
      if (m_absPosMeasurementCount == 0) {
        m_absPosMeasurementCount = 1;
        m_avgRawHomePos = getRawInnerArmPos();
      } else if (m_absPosMeasurementCount++ < 51) {
        m_avgRawHomePos = (m_avgRawHomePos * .95) + (getRawInnerArmPos() * .05);
      } else if (m_absPosMeasurementCount == 51) {
        // CANcoder has been averaged for a full second while Home
        // sensor is still active, so compare expected reading with current
        // corrected reading. But only do this once per new arrival at sensor.
        if (Math.abs(IAC.VERTICAL_POS - getAbsInnerArmPos()) > .003) {
          m_magnetOffset = -m_avgRawHomePos;
          configInnerArmCANcoder();
        }
      }
    } else {
      // reset the count - did not stay put long enough for a good average
      m_absPosMeasurementCount = 0;
    }
  }

  // This routine is designed to be called manually via an assigned button press,
  // but only when the calibration stick is in place and both Arms are horizontal.
  // Also, wait for average to stabilize - at least 5 seconds. As of now, only 
  // used for InnerArm reset. The MasterArm is much more stable and does not need 
  // frequent re-calibration.
  // This reset is only good until the next RoboRio re-boot, so if happy with the
  // new offset, edit the NotableConstants.java file at the earliest opportunity
  // and change IAC.INNER_ARM_CANCODER_MAGNET_OFFSET to that new offset.
  public void resetMagnetOffset() {
    m_magnetOffset = -m_avgInnerRawAbsPos;
    configInnerArmCANcoder();
  }
    
  /*************************************
   * Publish Inner Arm data methods
   ************************************/

    public void setupInnerArmPublishing() {
    ShuffleboardTab sbt = Shuffleboard.getTab("Note Handlers");
    ShuffleboardLayout sbLayout =  sbt.getLayout("Inner Arm", BuiltInLayouts.kList)
                                      .withPosition(IAC.INNER_ARM_DATA_COL,
                                                    IAC.INNER_ARM_DATA_ROW)
                                      .withSize(2, IAC.INNER_ARM_DATA_LIST_HGT)
                                      .withProperties(Map.of("Label position", "LEFT"));
    sbLayout.add("Ids: ", F.df1.format(IAC.INNER_ARM_FALCON_ID)+"  "+F.df1.format(IAC.INNER_ARM_CANCODER_ID));
    m_magnetOffsetEntry   = sbLayout.add("Offset ", F.df3.format(m_magnetOffset)).getEntry();
    m_absAxlePosEntry     = sbLayout.add("Abs Pos", "0").getEntry();
    m_falconPosEntry      = sbLayout.add("Motor Pos", "0").getEntry();
    m_armSetpointEntry    = sbLayout.add("SetPt Pos", "0").getEntry();
    m_armPIDOutEntry      = sbLayout.add("PID Out", "0").getEntry();
    m_axleVelocityEntry   = sbLayout.add("Axel Vel", "0").getEntry();
    m_falconAmpsEntry     = sbLayout.add("Amps", "0").getEntry();
  }
  
  // This method writes InnerArm data to the dashboard.
  private void publishInnerArmData() {
    // SmartDashboard.putNumber("InnerCanRelPos ", m_innerArmCANcoder.getPosition().getValueAsDouble());
    m_avgInnerRawAbsPos = (m_avgInnerRawAbsPos * .95) + (getRawInnerArmPos() * .05);
    if ((m_count++ % 25) < .001) { // update the average every 1/2 sec to make it easier to copy
      SmartDashboard.putNumber("InnerAvgRawAbsPos ", m_avgInnerRawAbsPos);
    }
    SmartDashboard.putNumber("InnerCorrAbsPos ", getAbsInnerArmPos());
    // SmartDashboard.putNumber("InnerMotorPos ", m_innerArmMotor.getPosition().getValueAsDouble());

    m_magnetOffsetEntry.setString(F.df4.format(m_magnetOffset));
    m_absAxlePosEntry.setString(F.df3.format(getAbsInnerArmPos()));
    m_falconPosEntry.setString(F.df3.format(m_innerArmMotor.getPosition().getValueAsDouble()));
    m_armSetpointEntry.setString(F.df3.format(m_innerArmSetpoint));
    m_armPIDOutEntry.setString(F.df3.format(m_innerArmMotor.getClosedLoopOutput().getValueAsDouble()));
    m_axleVelocityEntry.setString(F.df3.format(m_innerArmCANcoder.getVelocity().getValueAsDouble()));
    m_falconAmpsEntry.setString(F.df3.format(m_innerArmMotor.getSupplyCurrent().getValueAsDouble()));
  }     

  /*************************
   * Periodic method
   *************************/
  @Override
  public void periodic() {
    // This method will be called once per loop
    publishInnerArmData();
  }
}
