 public class InnerArmSubsystem extends SubsystemBase {
  private TalonFX m_innerArmMotor = new TalonFX(IAC.INNER_ARM_FALCON_ID, NotableConstants.CANIVORE_BUS_NAME);
  private CANcoder m_innerArmCANcoder = new CANcoder(IAC.INNER_ARM_CANCODER_ID, NotableConstants.CANIVORE_BUS_NAME);
  private final MotionMagicVoltage m_innerArmMagicCtrl = new MotionMagicVoltage(0.0)
                                                                                .withSlot(0)
                                                                                .withEnableFOC(true);
//	...
	
/** Creates a new InnerArmSubsystem. */
  public InnerArmSubsystem() {
    configInnerArmCANcoder();
    configInnerArmMotor();
    setupInnerArmPublishing();
    m_innerArmSetpoint = IAC.INDEXED_SPEAKER_GOAL_POS;
  }

public void gotoPosition(double position) {
    m_innerArmSetpoint = position;
    m_innerArmMotor.setControl(m_innerArmMagicCtrl.withPosition(m_innerArmSetpoint));
    m_startTime = System.currentTimeMillis();
  }

/****************************************
   * Configure Hardware methods 
   ****************************************/
  private void configInnerArmMotor() {
    var openLoopConfig = new OpenLoopRampsConfigs().withDutyCycleOpenLoopRampPeriod(0)
                                                    .withVoltageOpenLoopRampPeriod(IAC.INNER_ARM_OPEN_LOOP_RAMP_PERIOD)
                                                    .withTorqueOpenLoopRampPeriod(0);
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
                                                   .withOpenLoopRamps(openLoopConfig)
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