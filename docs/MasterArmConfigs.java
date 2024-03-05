 
 public class MasterArmSubsystem extends SubsystemBase {
	 
	// Code snippet - needs includes, enums, etc. before being compilable

  private TalonFX m_masterArmMotor = new TalonFX(MAC.MASTER_ARM_FALCON_ID, NotableConstants.CANIVORE_BUS_NAME);
  private CANcoder m_masterArmEncoder = new CANcoder(MAC.MASTER_ARM_ENCODER_ID, NotableConstants.CANIVORE_BUS_NAME);
  private final MotionMagicDutyCycle m_masterArmMagicCtrl = 
                                 new MotionMagicDutyCycle(0.0).withSlot(0);
								 
  /******************************************************
   * Constructor for a new MasterArmSubsystem. 
   ******************************************************/
  public MasterArmSubsystem() {
    configAbsMasterArmCANCoder();
    configMasterArmMotor();
    m_nowPlaying = Repetoire.NOTE_HANDLER_IDLE;
    m_pendingNote = m_nowPlaying;
    resetSeqNo();
    m_masterArmState = MasterArmStates.AT_INDEXED_SPEAKER_GOAL_POS;
    m_currentMasterArmSetpoint = MAC.INDEXED_SPEAKER_SHOT_POS;
    m_isDistantSpeakerShot = false;

    setupMasterArmPublishing();
  }

 /*************************************************************
   * @param setpoint
   * Routine for directing MasterArm to a desired 
   * position. The setpoint argument is in absolute 
   * rotation units, and will generally be limited 
   * to those pre-defined in NotableConstants.java.
   **************************************************************/
  public void gotoPosition(double setpoint) {
    m_masterArmMotor.setControl(m_masterArmMagicCtrl.withPosition(setpoint));
    m_currentMasterArmSetpoint = setpoint;
  }


 /*******************************************************************
   * Setup and Config routines
   ******************************************************************/
  private void configMasterArmMotor() {
    var openLoopConfig = new OpenLoopRampsConfigs().withDutyCycleOpenLoopRampPeriod(0)
                                                   .withVoltageOpenLoopRampPeriod(MAC.MASTER_ARM_OPEN_LOOP_RAMP_PERIOD)
                                                   .withTorqueOpenLoopRampPeriod(0);
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
                                                    .withOpenLoopRamps(openLoopConfig)
                                                    .withClosedLoopRamps(closedLoopConfig)
                                                    .withSlot0(pid0Config)
                                                    .withMotionMagic(motionMagicConfig);
    StatusCode status = m_masterArmMotor.getConfigurator().apply(masterArmConfig);

    if (! status.isOK() ) {
        SmartDashboard.putString("Failed to apply MASTER_ARM configs ", " Error code: "+status.toString());
    }
  }

  private void configAbsMasterArmCANCoder() {        
    var magnetSensorConfigs = new MagnetSensorConfigs().withAbsoluteSensorRange(MAC.MASTER_ARM_CANCODER_RANGE)
                                                       .withSensorDirection(MAC.MASTER_ARM_CANCODER_DIR)
                                                       .withMagnetOffset(MAC.MASTER_ARM_ENCODER_MAGNET_OFFSET);
    var ccConfig = new CANcoderConfiguration().withMagnetSensor(magnetSensorConfigs);
    StatusCode status = m_masterArmEncoder.getConfigurator().apply(ccConfig);
    if (! status.isOK() ) {
        SmartDashboard.putString("Failed to apply MASTER_ARM configs ", " Error code: "+status.toString());
    }
  }