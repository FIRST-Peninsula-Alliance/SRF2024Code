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
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
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
  private double m_avgInnerRawAbsPos;
  private double m_magnetOffset = IAC.INNER_ARM_CANCODER_MAGNET_OFFSET;
  private double m_temp;
  private boolean m_isDistantSpeakerShot = false;

  private final MotionMagicVoltage m_innerArmMagicCtrl = new MotionMagicVoltage(0.0)
                                                                                .withSlot(0)
                                                                                .withEnableFOC(true);
  private long m_startTime;

  private GenericEntry m_magnetOffsetEntry;
  private GenericEntry m_rawAxlePosEntry;
  private GenericEntry m_absAxlePosEntry;
  private GenericEntry m_armSetpointEntry;
  private GenericEntry m_armPIDOutEntry;
  private GenericEntry m_axleVelocityEntry;
  private GenericEntry m_falconAmpsEntry;

  private Supplier<String> m_currentStateName;
  private IntSupplier m_currentSeqNo;
  private double m_positionError;
  private long m_elapsedTime;
  private FileRecorder m_fileRecorder;
  private boolean LOGGING_ACTIVE;
 
  /** Creates a new InnerArmSubsystem. */
  public InnerArmSubsystem(Supplier<String> currentStateName, 
                           IntSupplier currentSeqNo,
                           FileRecorder fileRecorder) {
    m_currentStateName = currentStateName;
    m_currentSeqNo = currentSeqNo;
    m_fileRecorder = fileRecorder;
    LOGGING_ACTIVE = m_fileRecorder.isFileRecorderAvail();
    configInnerArmCANcoder();
    configInnerArmMotor();
    setupInnerArmPublishing();
    // Set boot-up inner arm setpoint to the safest possible position
    gotoVerticalUpPos();
    // Initialize avg Inner Arm Cancoder value
    m_avgInnerRawAbsPos = getRawInnerArmPos();
  }

  /************************************************************************
   * Utilities for slightly adjusting position of innerArm (assumed static)
   ************************************************************************/
  public double limitInnerArmPosition(double position) {
    if (position > IAC.MAX_INNER_ARM_SOFT_LIMIT) {
      return(IAC.MAX_INNER_ARM_SOFT_LIMIT);
    } else if (position < IAC.MIN_INNER_ARM_SOFT_LIMIT) {
      return IAC.MIN_INNER_ARM_SOFT_LIMIT;
    } else {
      return position;
    }
  }

  public void adjustInnerArmSetpoint(double direction, boolean debug_on) {
    // direction, debug, and/or current state (WAITING_FOR_NOTE, if not debug)
    // have already been vetted by MasterArmSubsystem
    System.out.println("Adjusting IA "+direction);
    m_innerArmSetpoint = limitInnerArmPosition(m_innerArmSetpoint + (direction / 360.0));
    gotoPosition(m_innerArmSetpoint);
  }

  /*********************************
   * check for reaching setpoints
   *********************************/

  public boolean innerArmIsVerticalUp() {
    return innerArmIsAt(IAC.VERTICAL_UP_POS, IAC.ALLOWED_MILLIS_IA_LARGE_MOVE);
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

  public boolean innerArmIsHorizontalForwardPos() {
    return innerArmIsAt(IAC.HORIZONTAL_FORWARD_POS, IAC.ALLOWED_MILLIS_IA_SMALL_MOVE);
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
      if (LOGGING_ACTIVE) {
        m_fileRecorder.recordMoveEvent( "IA",
                                        NoteEvent.SETPOINT_REACHED,
                                        position,
                                        m_positionError,
                                        System.currentTimeMillis(),
                                        m_elapsedTime,
                                        m_currentStateName.get(),
                                        m_currentSeqNo.getAsInt());
      }
      return true;
    }

    if (m_elapsedTime <= timeoutDuration) {
      // general timeout has not yet expired.
        return false;
    } else {
      // timeout has occured. Log event and force assume inner arm is at setpoint 
      // in order to avoid "hanging" the state machine (hopefully it is close enough).
      if (LOGGING_ACTIVE) {
        m_fileRecorder.recordMoveEvent( "IA",
                                        NoteEvent.TIMEOUT_OCCURED,
                                        position,
                                        m_positionError,
                                        System.currentTimeMillis(),
                                        m_elapsedTime,
                                        m_currentStateName.get(),
                                        m_currentSeqNo.getAsInt() );
      }
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
    if (LOGGING_ACTIVE) {
      m_fileRecorder.recordReqEvent("IA",
                                    NoteRequest.MOVE_INNER_ARM,
                                    m_innerArmSetpoint,
                                    m_startTime,
                                    m_currentStateName.get(),
                                    m_currentSeqNo.getAsInt());
    }
  }

  public void gotoVerticalUpPos() {
    //if (getAbsInnerArmPos() > IAC.VERTICAL_UP_POS) {
    //  m_innerArmSetpoint = IAC.VERTICAL_UP_POS * IAC.INNER_ARM_CANCODER_TO_AXLE_RATIO;
    //} else {
      m_innerArmSetpoint = IAC.VERTICAL_UP_POS;
    //}
    gotoPosition(m_innerArmSetpoint);
  }
  
  public void gotoVerticalDownPos() {
    gotoPosition(IAC.VERTICAL_DOWN_POS);
  }

  public void gotoBumperContactPos() {
    gotoPosition(IAC.BUMPER_CONTACT_POS);
  }

  public void gotoNotePickupPos() {
    gotoPosition(IAC.NOTE_PICKUP_POS);
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

  public void gotoHorizontalForwardPos() {
    gotoPosition(IAC.HORIZONTAL_FORWARD_POS);
  }
  
  public void gotoAmpShotPos() {
    gotoPosition(IAC.AMP_GOAL_POS);
  }

  /****************************************
   * Configure Hardware methods 
   ****************************************/
  private void configInnerArmMotor() {
    var closedLoopConfig = new ClosedLoopRampsConfigs().withDutyCycleClosedLoopRampPeriod(IAC.INNER_ARM_CLOSED_LOOP_RAMP_PERIOD)
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
    var currentLimitConfig = new CurrentLimitsConfigs().withSupplyCurrentLimit(IAC.INNER_ARM_CONT_CURRENT_LIMIT)
                                                       .withSupplyCurrentThreshold(IAC.INNER_ARM_PEAK_CURRENT_LIMIT)
                                                       .withSupplyTimeThreshold(IAC.INNER_ARM_PEAK_CURRENT_DURATION)
                                                       .withSupplyCurrentLimitEnable(IAC.INNER_ARM_ENABLE_CURRENT_LIMIT)
                                                       .withStatorCurrentLimit(IAC.INNER_ARM_STATOR_CURRENT_LIMIT)
                                                       .withStatorCurrentLimitEnable(IAC.INNER_ARM_ENABLE_STATOR_CURRENT_LIMIT);
    var softwareLimitSwitchConfig = new SoftwareLimitSwitchConfigs().withForwardSoftLimitThreshold(IAC.MAX_INNER_ARM_SOFT_LIMIT)
                                                                    .withForwardSoftLimitEnable(IAC.ENABLE_INNER_ARM_SOFT_LIMITS)
                                                                    .withReverseSoftLimitThreshold(IAC.MIN_INNER_ARM_SOFT_LIMIT)
                                                                    .withReverseSoftLimitEnable(IAC.ENABLE_INNER_ARM_SOFT_LIMITS);
    Slot0Configs pid0Config = new Slot0Configs().withKP(IAC.INNER_ARM_KP)
                                                .withKI(IAC.INNER_ARM_KI)
                                                .withKD(IAC.INNER_ARM_KD)
                                                .withKS(IAC.INNER_ARM_KS)
                                                .withKV(IAC.INNER_ARM_KV)
                                                .withKA(IAC.INNER_ARM_KA)
                                                .withKG(IAC.INNER_ARM_KG)
                                                .withGravityType(GravityTypeValue.Arm_Cosine);
    MotionMagicConfigs  motionMagicConfig = new MotionMagicConfigs().withMotionMagicCruiseVelocity(IAC.INNER_ARM_MOTION_MAGIC_VEL)
                                                                    .withMotionMagicAcceleration(IAC.INNER_ARM_MOTION_MAGIC_ACCEL)
                                                                    .withMotionMagicJerk(IAC.INNER_ARM_MOTION_MAGIC_JERK)
                                                                    .withMotionMagicExpo_kA(IAC.INNER_ARM_MOTION_MAGIC_kA)
                                                                    .withMotionMagicExpo_kV(IAC.INNER_ARM_MOTION_MAGIC_kV);
    var innerArmConfig = new TalonFXConfiguration().withFeedback(feedbackConfig)
                                                   .withMotorOutput(motorOutputConfig)
                                                   .withCurrentLimits(currentLimitConfig)
                                                   .withClosedLoopRamps(closedLoopConfig)
                                                   .withSoftwareLimitSwitch(softwareLimitSwitchConfig)
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

  // This method returns the uncorrected CANcoder Absolute position. The valid range of one
  // revolution is -.5 to + .5, so translate to baseband if needed.
  public double getRawInnerArmPos() {
    m_temp = getAbsInnerArmPos() - m_magnetOffset;
    if (Math.abs(m_temp) > .5) {
      m_temp = m_temp - Math.copySign(1.0, m_temp);
    } 
    return m_temp;
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
    m_rawAxlePosEntry     = sbLayout.add("Raw_P", "0").getEntry();
    m_absAxlePosEntry     = sbLayout.add("Abs_P", "0").getEntry();
    m_armSetpointEntry    = sbLayout.add("SetPt", "0").getEntry();
    m_armPIDOutEntry      = sbLayout.add("PID Out", "0").getEntry();
    m_axleVelocityEntry   = sbLayout.add("Axel_V", "0").getEntry();
    m_falconAmpsEntry     = sbLayout.add("Amps", "0").getEntry();
  }
  
  // This method writes InnerArm data to the dashboard.
  private void publishInnerArmData() {
    m_avgInnerRawAbsPos = (m_avgInnerRawAbsPos * .95) + (getRawInnerArmPos() * .05);
    SmartDashboard.putNumber("InnerAvgRawAbsPos ", m_avgInnerRawAbsPos);
    SmartDashboard.putNumber("InnerCorrAbsPos ", getAbsInnerArmPos());

    m_magnetOffsetEntry.setString(F.df4.format(m_magnetOffset));
    m_rawAxlePosEntry.setString(F.df4.format(getRawInnerArmPos()));
    m_absAxlePosEntry.setString(F.df4.format(getAbsInnerArmPos()));
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
    /*
    if (Math.abs(m_innerArmSetpoint - IAC.INDEXED_SPEAKER_GOAL_POS) < .01) {
      //if (m_innerArmCANcoder.getAbsolutePosition().getValueAsDouble() + .355 > .005) {
      //  adjustInnerArmSetpoint(1, false );
      if (m_innerArmCANcoder.getAbsolutePosition().getValueAsDouble() + .355 < -0.005) {
        adjustInnerArmSetpoint(-1, false);
      }
    }
    */
  }
}