// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.text.DecimalFormat;
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
import frc.robot.Constants;
import frc.robot.NotableConstants.IAC;

public class InnerArmSubsystem extends SubsystemBase {
  private TalonFX m_innerArmMotor = new TalonFX(IAC.INNER_ARM_FALCON_ID, Constants.CANIVORE_BUS_NAME);
  private CANcoder m_innerArmCANcoder = new CANcoder(IAC.INNER_ARM_CANCODER_ID, Constants.CANIVORE_BUS_NAME);

  private double m_innerArmSetpoint;
  private double m_innerArmNotePickupPosSetpoint;
  private boolean m_isDistantSpeakerShot = false;

  private final MotionMagicVoltage m_innerArmMagicCtrl = new MotionMagicVoltage(0.0)
                                                                                .withSlot(0)
                                                                                .withEnableFOC(true);
  private long m_startTime;

  private GenericEntry m_absAxlePosEntry;
  private GenericEntry m_falconPosEntry;
  private GenericEntry m_armSetpointEntry;
  private GenericEntry m_armPIDOutEntry;
  private GenericEntry m_axleVelocityEntry;
  private GenericEntry m_falconAmpsEntry;

  // Formatters to control number of decimal places in the published data lists
  DecimalFormat df1 = new DecimalFormat("#.#");
  DecimalFormat df3 = new DecimalFormat("#.###");

  /** Creates a new InnerArmSubsystem. */
  public InnerArmSubsystem() {
    configInnerArmCANcoder();
    configInnerArmMotor();
    setupInnerArmPublishing();
    m_innerArmSetpoint = IAC.INDEXED_SPEAKER_GOAL_POS;
    m_innerArmNotePickupPosSetpoint = IAC.NOTE_PICKUP_POS;
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
    return innerArmIsAt(IAC.VERTICAL_POS);
  }

  public boolean innerArmIsAtBumperContactPos() {
    return innerArmIsAt(IAC.BUMPER_CONTACT_POS);
  }

  public boolean innerArmIsAtPickupPos() {
    return innerArmIsAt(IAC.NOTE_PICKUP_POS);
  }

  public boolean innerArmIsAtDistantSpeakerPos() {
    return innerArmIsAt(IAC.DISTANT_SPEAKER_GOAL_POS);
  }
      
  public boolean innerArmIsAtIndexedSpeakerPos() {
    return innerArmIsAt(IAC.INDEXED_SPEAKER_GOAL_POS);
  }

  public boolean innerArmIsHorizontalBackPos() {
    return innerArmIsAt(IAC.HORIZONTAL_BACK_POS);
  }
  
  public boolean innerArmIsAtAmpScoringPos() {
    return innerArmIsAt(IAC.AMP_GOAL_POS);
  }

  public void gotoPosition(double position) {
    m_innerArmSetpoint = position;
    m_innerArmMotor.setControl(m_innerArmMagicCtrl.withPosition(m_innerArmSetpoint));
    m_startTime = System.currentTimeMillis();
  }

  // getAbsInnerArmPos returns the InnerArmEncoder sensor position
  // which is in absolute rotations, with origin of 0 when horizontal,
  // already corrected for magnet offset (the offset must be measured,
  // then stored in NotableConstants.java).
  public double getAbsInnerArmPos() {
    return(m_innerArmCANcoder.getAbsolutePosition().getValueAsDouble());
  }

  public boolean innerArmIsAt(double position) {
    if (Math.abs(getAbsInnerArmPos() - position) < IAC.ALLOWED_INNER_ARM_POS_ERROR) {
      return true;
    } else {
      return ((System.currentTimeMillis() - m_startTime) > IAC.ALLOWED_MILLIS_PER_MOVE);
    }
  }
  
 /*****************************************
   * goto various inner arm position methods
   * Called only from Master Arm sequencer
   *****************************************/
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
      SmartDashboard.putString("Failed to apply INNER_ARM configs ", " Error code: "+status.toString());
    }
  }
  
  private void configInnerArmCANcoder() {
    var magnetSensorConfigs = new MagnetSensorConfigs().withAbsoluteSensorRange(IAC.INNER_ARM_CANCODER_RANGE)
                                                       .withSensorDirection(IAC.INNER_ARM_CANCODER_DIR)
                                                       .withMagnetOffset(IAC.INNER_ARM_CANCODER_MAGNET_OFFSET);
    var ccConfig = new CANcoderConfiguration().withMagnetSensor(magnetSensorConfigs);
    StatusCode status = m_innerArmCANcoder.getConfigurator().apply(ccConfig);
    if (! status.isOK() ) {
      SmartDashboard.putString("Failed to apply INNER_CANcoder configs ", " Error code: "+status.toString());
    }
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
    sbLayout.add("Ids: ", df1.format(IAC.INNER_ARM_FALCON_ID)+"  "+df1.format(IAC.INNER_ARM_CANCODER_ID));
    sbLayout.add("Offset ", df3.format(IAC.INNER_ARM_CANCODER_MAGNET_OFFSET));
    m_absAxlePosEntry     = sbLayout.add("Abs Pos", "0").getEntry();
    m_falconPosEntry      = sbLayout.add("Motor Pos", "0").getEntry();
    m_armSetpointEntry    = sbLayout.add("SetPt Pos", "0").getEntry();
    m_armPIDOutEntry      = sbLayout.add("PID Out", "0").getEntry();
    m_axleVelocityEntry   = sbLayout.add("Axel Vel", "0").getEntry();
    m_falconAmpsEntry     = sbLayout.add("Amps", "0").getEntry();
  }
  
  private void publishInnerArmData() {
    // SmartDashboard.putNumber("InnerCanRelPos ", m_innerArmCANcoder.getPosition().getValueAsDouble());
    SmartDashboard.putNumber("InnerRawAbsPos ", getAbsInnerArmPos()-IAC.INNER_ARM_CANCODER_MAGNET_OFFSET);
    SmartDashboard.putNumber("InnerCorrAbsPos ", getAbsInnerArmPos());
    SmartDashboard.putNumber("InnerMotorPos ", m_innerArmMotor.getPosition().getValueAsDouble());

    m_absAxlePosEntry.setString(df3.format(getAbsInnerArmPos()));
    m_falconPosEntry.setString(df3.format(m_innerArmMotor.getPosition().getValueAsDouble()));
    m_armSetpointEntry.setString(df3.format(m_innerArmSetpoint));
    m_armPIDOutEntry.setString(df3.format(m_innerArmMotor.getClosedLoopOutput().getValueAsDouble()));
    m_axleVelocityEntry.setString(df3.format(m_innerArmCANcoder.getVelocity().getValueAsDouble()));
    m_falconAmpsEntry.setString(df3.format(m_innerArmMotor.getSupplyCurrent().getValueAsDouble()));
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
