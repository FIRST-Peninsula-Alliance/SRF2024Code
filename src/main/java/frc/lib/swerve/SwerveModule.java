package frc.lib.swerve;
import frc.robot.Constants.*;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityVoltage;

import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.OpenLoopRampsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;


import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class SwerveModule {
    public  int m_modNum;
    private SwerveModuleConstants m_moduleConstants;
    private Rotation2d m_absAngleOffset2d;
    private double desiredAngle;      // Just a local variable, persistent to relieve Java GC pressure
    private double m_lastAngle;       // This is not used for control, rather it 
                                      // stores the last setpoint requested for a
                                      // given module, for data publishing purposes.
    private final CANSparkMax m_steerMotor;
    private final RelativeEncoder m_integratedSteerEncoder;
    private final SparkPIDController m_steerController;
    private final CANcoder m_absWheelAngleCANcoder;
    private final TalonFX m_driveMotor;
    // Declare Phoenix6 control request objects for the Drive Motor:
    // Open loop control output to the drive motor is one shot DutyCycle, and must be 
    // repeated every loop to avoid safety timeout
    final DutyCycleOut m_driveOpenLoop = new DutyCycleOut(0.0).withUpdateFreqHz(0);
    // In phoenix5 the closed loop drive control was voltage compensated with arbitrary feed forward.
    // In Phoenix6 the equivalent is VelocityVoltage control, again with arbitrary feed forward calculated
    // per loop. The default update rate is 100 Hz, leave it as is and bump it every loop with updated
    // feed forward calculations.
    final VelocityVoltage m_driveClosedLoop = new VelocityVoltage(0.0);

    private SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(SDC.DRIVE_KS,
                                                                            SDC.DRIVE_KV,
                                                                            SDC.DRIVE_KA);
    private double m_velocityFeedForward;

    // Entries for publishing module data.
    // Technically, the following member Entry variables do not need to be saved,
    // as the data written via them is static and should never change after
    // initial setup
    // private GenericEntry IdsEntry;       // Drive ID, Rotate ID, CANcoder ID, in that order
                                            // Single digits only, to fit module list width
    // private GenericEntry absOffsetEntry;

    // However, the following Entry keys are for dynamic variables, and
    // will change depending on activity.
    private GenericEntry steerSetpointDegEntry;
    private GenericEntry steerEncoderDegEntry;
    private GenericEntry absCANcoderDegEntry;
    private GenericEntry steerPIDOutputEntry;
    private GenericEntry wheelCurrPosEntry;
    private GenericEntry wheelCurrSpeedEntry;
    private GenericEntry wheelAmpsEntry;
    private GenericEntry wheelTempEntry;
    private GenericEntry steerAmpsEntry;
    private GenericEntry steerTempEntry;

    public SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants) {
        m_modNum = moduleNumber;
        m_moduleConstants = moduleConstants;
        m_absAngleOffset2d = moduleConstants.ABS_ANG_OFFSET2D;
        
        /* Angle Encoder Config */
        m_absWheelAngleCANcoder = new CANcoder(m_moduleConstants.ENCODER_ID);
        // Use the following for use with CANivore
        // m_absWheelAngleCANcoder = new CANcoder(ID, canbus);
        // where canbus is a string identifying which canbus to use
        configAbsWheelAngleCANcoder();

        /* Angle Motor Config */
        m_steerMotor = new CANSparkMax(m_moduleConstants.STEER_MOTOR_ID, 
                                       MotorType.kBrushless);
        m_steerController = m_steerMotor.getPIDController();
        m_integratedSteerEncoder = m_steerMotor.getEncoder();
        configSteerMotor();

        /* Drive Motor Config */
        m_driveMotor = new TalonFX(m_moduleConstants.DRIVE_MOTOR_ID);
        configDriveMotor();

        m_lastAngle = getState().angle.getDegrees();

        setupModulePublishing();
    }

    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
        // Below is a custom optimize function, since default WPILib optimize assumes 
        // continuous controller which CTRE and Rev onboard were not.
        // The job of optimize is to minimise the amount of rotation required to
        // get to the new direction, given the actual current direction, of the
        // swerve module.
        // TODO: Check if that still applies - now that NEO encoder is configured to
        // be continuous and tracks in degrees (using a configured conversion factor)
        // perhaps the WPILib optimize would be usable as is?
        desiredState = SwerveOptimize.optimize(desiredState, getState().angle);
        desiredAngle = desiredState.angle.getDegrees();
        // But, (for normal operation, not testing of a single module) only 
        // rotate the module if wheel speed is more than 1% of robot's max speed. 
        // This prevents Jittering. Otherwise, no turn needed, as closed loop
        // steering will maintain prior direction.
        if (Math.abs(desiredState.speedMetersPerSecond) > (SDC.MAX_ROBOT_SPEED_M_PER_SEC * 0.01)) {
            setAngle(desiredAngle);
        }
        setSpeed(desiredState, isOpenLoop);
    }

    private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop) {
        if(isOpenLoop) {
            // Drive using joystick. Convert MPS to a DutyCycle (was percent) output
            m_driveOpenLoop.Output = desiredState.speedMetersPerSecond / SDC.MAX_ROBOT_SPEED_M_PER_SEC;
            m_driveMotor.setControl(m_driveOpenLoop);
        }
        else {
            // Drive using VelocityVoltage PID, using Falcon encoder units and default Slot0
            m_driveClosedLoop.Velocity = desiredState.speedMetersPerSecond * SDC.MPS_TO_FALCON_RPS_FACTOR;
            m_velocityFeedForward = feedforward.calculate(desiredState.speedMetersPerSecond);
            m_driveMotor.setControl(m_driveClosedLoop.withFeedForward(m_velocityFeedForward));
        }
    }

    public void setAngle(double desiredAngle) {
        // We can now setReference in native units (deg) since Neo encoder is
        // configured to operate in degrees. No need to convert to NEO rev units
        // When this is called by module angle test routines, there is no "optimized" turn limit
        m_steerController.setReference(desiredAngle, CANSparkBase.ControlType.kPosition);
        m_lastAngle = desiredAngle;
    }

    // getAngle2d returns the current swerve module direction as a Rotation2d value.
    private Rotation2d getAngle2d() {
        return Rotation2d.fromDegrees(m_integratedSteerEncoder.getPosition());
    }

    // NEO encoder is initialized to use native units (deg)
    public void setNeoPosDeg(double angle) {
        m_integratedSteerEncoder.setPosition(angle);
    }

    // getNeoPosDeg returns the value of the Neo's integrated encoder
    // in degrees (since configured with the conversion factor) in the 
    // range 0 to 360.
    public double getNeoPosDeg() {
        return Math.IEEEremainder(m_integratedSteerEncoder.getPosition(), 360.0) + 180.0;
    }

    public double getRawNeoPos() {
        return m_integratedSteerEncoder.getPosition();
    }

    public double getCANcoderDeg() {
        // .getAbsolutePosition returns a StatusSignal, so we need to get it's value 
        // and convert range of [0, 1] into degrees
        return m_absWheelAngleCANcoder.getAbsolutePosition().getValueAsDouble() * 360.0;
    }

    public Rotation2d getCANcoder2d(){
        return Rotation2d.fromDegrees(getCANcoderDeg());
    }

    private void waitForCANcoder() {
        // Wait for up a good CANcoder signal. This prevents a race condition during program startup
        // where trying to synchronize the Integrated motor encoder to the CANcoder before we have
        // received any position signal from the CANcoder results in failure.
        var posStatus = m_absWheelAngleCANcoder.getAbsolutePosition();

        for (int i = 0; i < 100; ++i) {
            if (posStatus.getStatus().isOK()) {
                break;
            } else {
                // Wait 10 ms and try again
                Timer.delay(0.010);    // Only called at initialization, or rarely from
                                               // teleop where no boot up delays are expected, 
                                               // so .delay() is ugly here but OK to use.
                posStatus = m_absWheelAngleCANcoder.getAbsolutePosition();
            }
            SmartDashboard.putNumber("Mod"+m_modNum+"AbsCANcoder read failed, read tries = ", i);         
        }
        posStatus.waitForUpdate(200);   // Then wait up to 200 ms for one more refresh before returning
    }

    public void resetToAbsolute(){
        waitForCANcoder();
        double CANcoderOnReset = getCANcoderDeg();
        double absModuleDegOnReset = CANcoderOnReset - m_absAngleOffset2d.getDegrees();
        //SmartDashboard.putString("Mod"+m_modNum+" CANcoder on Reset", F.df2.format(CANcoderOnReset));
        setNeoPosDeg(absModuleDegOnReset);
    }

    private void configAbsWheelAngleCANcoder(){ 
        var magnetSensorConfigs = new MagnetSensorConfigs().withAbsoluteSensorRange(SDC.CANCODER_RANGE)
                                                           .withSensorDirection(SDC.CANCODER_DIR)
                                                           .withMagnetOffset(0.0);
        var ccConfig = new CANcoderConfiguration().withMagnetSensor(magnetSensorConfigs);
        m_absWheelAngleCANcoder.getConfigurator().apply(ccConfig);

        // Remaining questions:
        // Config boot strategy, timebase? No - the first is removed from P6, always boot to absolute,
        // and the latter is configurable via setUpdateFrequency()
    }

     // call after Absolute offsets have been set
     public void slowCanCoderBusUsage() {
        // Don't use module CANCoders at all after initialization, so this is provided 
        // to slow Cancoder usage way down
        m_absWheelAngleCANcoder.getPosition().setUpdateFrequency(5);
        m_absWheelAngleCANcoder.getVelocity().setUpdateFrequency(5);
        // could call optimizeBus(), but that may affect logging.
     }

     // Wait for success before return? 
     // Or call some time before needing encoders again (e.g. to RE-set absolute offsets)?
     public void speedUpCancoderBusReports() {
        m_absWheelAngleCANcoder.getPosition().setUpdateFrequency(100);
     }

    private void reportRevError(REVLibError errorCode) {
        if (errorCode != REVLibError.kOk) {
            SmartDashboard.putString("Mod "+m_modNum+"RevLibError = ", errorCode.toString());
        }
    }

    private void configSteerMotor(){
        reportRevError(m_steerMotor.restoreFactoryDefaults());
        // Configure to send motor encoder position data frequently, but everything
        // else at a lower rate, to minimize can bus traffic.
        //reportRevError(CANSparkMaxUtil.setCANSparkMaxBusUsage(m_steerMotor, Usage.kPositionOnly));
        reportRevError(m_steerMotor.setSmartCurrentLimit(SDC.STEER_SMART_CURRENT_LIMIT));
        reportRevError(m_steerMotor.setSecondaryCurrentLimit(SDC.STEER_SECONDARY_CURRENT_LIMIT));
        // setInverted returns void
        m_steerMotor.setInverted(SDC.STEER_MOTOR_INVERT);
        reportRevError(m_steerMotor.setIdleMode(SDC.STEER_MOTOR_NEUTRAL_MODE));
        reportRevError(m_steerController.setP(SDC.STEER_KP));
        reportRevError(m_steerController.setI(SDC.STEER_KI));
        reportRevError(m_steerController.setD(SDC.STEER_KD));
        reportRevError(m_steerController.setFF(SDC.STEER_KF));
        reportRevError(m_steerController.setOutputRange(SDC.MIN_STEER_CLOSED_LOOP_OUTPUT,
                                                        SDC.MAX_STEER_CLOSED_LOOP_OUTPUT));
        reportRevError(m_steerController.setFeedbackDevice(m_integratedSteerEncoder));
        reportRevError(m_steerController.setPositionPIDWrappingEnabled(true));
        reportRevError(m_steerController.setPositionPIDWrappingMinInput(0));
        reportRevError(m_steerController.setPositionPIDWrappingMaxInput(360));
        // Make integrated encoder read in native units of degrees
        reportRevError(m_integratedSteerEncoder.setPositionConversionFactor(360.0/SDC.STEER_GEAR_RATIO));
        reportRevError(m_steerMotor.enableVoltageCompensation(SDC.STEER_MOTOR_VOLTAGE_COMPENSATION));
        //reportRevError(m_steerMotor.burnFlash());   // Do this durng development, but not
                                                    // routinely, to preserve the life of the
                                                    // flash memory. Is it even necessary, since
                                                    // all registers (except ID?) are written 
                                                    // via code on every bootup?
        //SmartDashboard.putString("Steer Motor Setup", "Complete");
        resetToAbsolute();

        // Additional item that may need to be considered for initialization
        //  ?      m_steerController.setIZone(kIz);
    }

    // setSteerKP is called for each module from SwerveDrive subsystem
    // whenever steerKP has been changed in the Shuffleboard "SwerveDrive" Tab. 
    // Allows on the fly tuning, if kP is published. Change code before competition
    // to disable this ability and thus avoid accidental changes.
    public void setSteerKP(double kP) {
        m_steerController.setP(kP);        
    }

    private void configDriveMotor(){
        var openLoopConfig = new OpenLoopRampsConfigs() .withDutyCycleOpenLoopRampPeriod(0)
                                                       .withVoltageOpenLoopRampPeriod(SDC.OPEN_LOOP_RAMP_PERIOD);
                                                       //.withTorqueOpenLoopRampPeriod(0);
        var closedLoopConfig = new ClosedLoopRampsConfigs().withDutyCycleClosedLoopRampPeriod(0)
                                                           .withVoltageClosedLoopRampPeriod(SDC.CLOSED_LOOP_RAMP_PERIOD)
                                                           .withTorqueClosedLoopRampPeriod(0);
        var feedbackConfig = new FeedbackConfigs().withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor)
                                                  .withSensorToMechanismRatio(SDC.DRIVE_GEAR_RATIO)
                                                  .withRotorToSensorRatio(1.0);
        var motorOutputConfig = new MotorOutputConfigs().withNeutralMode(SDC.DRIVE_MOTOR_NEUTRAL_MODE)
                                                        .withInverted(SDC.DRIVE_MOTOR_INVERT)
                                                        .withPeakForwardDutyCycle(SDC.OUTPUT_DRIVE_LIMIT_FACTOR)
                                                        .withPeakReverseDutyCycle(-SDC.OUTPUT_DRIVE_LIMIT_FACTOR);
                                                        //.withDutyCycleNeutralDeadband(.001);
        CurrentLimitsConfigs currentLimitConfig = new CurrentLimitsConfigs()
                                                        .withSupplyCurrentLimit(SDC.DRIVE_SUPPLY_CURRENT_LIMIT)
                                                        .withSupplyCurrentThreshold(SDC.DRIVE_SUPPLY_CURRENT_THRESHOLD)
                                                        .withSupplyTimeThreshold(SDC.DRIVE_SUPPLY_CURRENT_TIME_THRESHOLD)
                                                        .withSupplyCurrentLimitEnable(SDC.DRIVE_ENABLE_SUPPLY_CURRENT_LIMIT)
                                                        .withStatorCurrentLimit(SDC.DRIVE_STATOR_CURRENT_LIMIT)
                                                        .withStatorCurrentLimitEnable(SDC.DRIVE_ENABLE_STATOR_CURRENT_LIMIT );
        Slot0Configs pid0Configs = new Slot0Configs().withKP(SDC.DRIVE_KP)
                                                     .withKI(SDC.DRIVE_KI)
                                                     .withKD(SDC.DRIVE_KD)
                                                     .withKS(SDC.DRIVE_KS)
                                                     .withKV(SDC.DRIVE_KV)
                                                     .withKA(SDC.DRIVE_KA)
                                                     .withKG(SDC.DRIVE_KG);
                                                    // .withGravityType(   );
                                                    //    Elevator_Static if constant
                                                    //    Arm_Cosign if variable. Sensor must be 0 when mechanism
                                                    //                            is horiz, and sensor and Mechanism
                                                    //                            position must be 1:1
        var swerveDriveConfig = new TalonFXConfiguration().withFeedback(feedbackConfig)
                                                          .withMotorOutput(motorOutputConfig)
                                                          .withCurrentLimits(currentLimitConfig)
                                                          .withOpenLoopRamps(openLoopConfig)
                                                          .withClosedLoopRamps(closedLoopConfig)
                                                          .withSlot0(pid0Configs);
        StatusCode status = m_driveMotor.getConfigurator().apply(swerveDriveConfig);

        if (! status.isOK() ) {
            SmartDashboard.putString("Failed to apply Drive configs in Mod "+m_modNum, " Error code: "+status.toString());
        }
        // Remaining questions:
        // should SetSafetyEnabled(true) be set for closed loop control where updates are not needed
        // each loop? Assume true is default?
        // Set VoltageCompensation? No, eliminated in P6, replaced with xxxVoltage control modes.
        /* To make a remote cancoder over CANBus

        // Configure CANcoder to zero the magnet appropriately 
            CANcoderConfiguration cc_cfg = new CANcoderConfiguration();
            cc_cfg.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
            cc_cfg.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
            cc_cfg.MagnetSensor.MagnetOffset = 0.4;
            m_cc.getConfigurator().apply(cc_cfg);

            TalonFXConfiguration fx_cfg = new TalonFXConfiguration();
            fx_cfg.Feedback.FeedbackRemoteSensorID = m_cc.getDeviceID();
            fx_cfg.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
            fx_cfg.Feedback.SensorToMechanismRatio = 1.0;
            fx_cfg.Feedback.RotorToSensorRatio = 12.8;

            m_fx.getConfigurator().apply(fx_cfg);

            Usage is the same as any status signal:

            fx_pos.refresh();
            cc_pos.refresh();

            System.out.println("FX Position: " + fx_pos.toString());
            System.out.println("CANcoder Position: " + cc_pos.toString());
        */
    }

    public SwerveModuleState getState(){
        return new SwerveModuleState(
            m_driveMotor.getVelocity().getValueAsDouble() * SDC.FALCON_RPS_TO_MPS_FACTOR, 
            getAngle2d()
        ); 
    }

    public double getPositionM(){
        return m_driveMotor.getPosition().getValueAsDouble() * SDC.FALCON_ROT_TO_M_FACTOR; 
    }


    public SwerveModulePosition getPosition(){
        return new SwerveModulePosition(
            getPositionM(), 
            getAngle2d()
        );
    }

    public void setupModulePublishing() {
        ShuffleboardLayout sBE_Layout = m_moduleConstants.SBE_LAYOUT;
        // See comment above about not needing to retain these Entry keys
        /* IdsEntry (D R E)= */ sBE_Layout.add("Ids", 
                                               F.df1.format(m_moduleConstants.DRIVE_MOTOR_ID)+
                                               "  "+F.df1.format(m_moduleConstants.STEER_MOTOR_ID)+
                                               "  "+F.df1.format(m_moduleConstants.ENCODER_ID));
        /* absOffsetEntry = */  sBE_Layout.add("Offset", F.df1.format(m_moduleConstants.ABS_ANG_OFFSET2D.getDegrees()));
        absCANcoderDegEntry =   sBE_Layout.add("CCdeg", "0").getEntry();
        steerEncoderDegEntry =  sBE_Layout.add("MEdeg", "0").getEntry();
        steerSetpointDegEntry = sBE_Layout.add("SPdeg", "0").getEntry();
        steerPIDOutputEntry =   sBE_Layout.add("PID_O", "0").getEntry();
        wheelCurrSpeedEntry =   sBE_Layout.add("Wspd", "0").getEntry();
        wheelCurrPosEntry =     sBE_Layout.add("Wpos", "0").getEntry();
        wheelAmpsEntry =        sBE_Layout.add("DrAmps", "0").getEntry();
        wheelTempEntry =        sBE_Layout.add("DrTemp", "0").getEntry();
        steerAmpsEntry =        sBE_Layout.add("R_Amps", "0").getEntry();
        steerTempEntry =        sBE_Layout.add("R_Temp", "0").getEntry();
    }

    public void publishModuleData() {
        // CANcoder direction
        absCANcoderDegEntry.setString(F.df1.format(getCANcoderDeg()));
        // Current wheel direction
        steerEncoderDegEntry.setString(F.df1.format(getNeoPosDeg()));
        // Wheel direction (steer) setpoint
        steerSetpointDegEntry.setString(F.df1.format(Math.IEEEremainder(m_lastAngle, 360.0) + 180.0));
        //steerSetpointDegEntry.setString(F.df1.format(m_lastAngle));
        // SteerMotor PID applied ouutput    
        steerPIDOutputEntry.setString(F.df2.format(m_steerMotor.getAppliedOutput()));
        // Wheel Speed
        wheelCurrSpeedEntry.setString(F.df1.format(m_driveMotor.getVelocity().getValueAsDouble() * SDC.FALCON_RPS_TO_MPS_FACTOR));
        // Wheel position, meters
        wheelCurrPosEntry.setString(F.df2.format(getPositionM()));
        // Wheel Amps
        wheelAmpsEntry.setString(F.df1.format(m_driveMotor.getSupplyCurrent().getValueAsDouble()));
        // Wheel Temp
        wheelTempEntry.setString(F.df1.format(m_driveMotor.getDeviceTemp().getValueAsDouble()));
        // Steering Amps
        steerAmpsEntry.setString(F.df1.format(m_steerMotor.getOutputCurrent()));
        // Steering Temp
        steerTempEntry.setString(F.df1.format(m_steerMotor.getMotorTemperature()));
    }

    public void stop() {
        m_driveOpenLoop.Output = 0.0;
        m_driveMotor.setControl(m_driveOpenLoop);
        m_steerMotor.stopMotor();
    }
}