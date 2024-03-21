// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.IntSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.OpenLoopRampsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.FileRecorder;
import frc.lib.util.FileRecorder.NoteEvent;
import frc.lib.util.FileRecorder.NoteRequest;
import frc.robot.Constants;
import frc.robot.NotableConstants.SC;
import frc.robot.commands.RumbleCmd;

public class ShooterSubsystem extends SubsystemBase {
  // The shooter is started via the prepareToShoot() method, called from MasterArmSubsystem.
  // When the shooter wheels reach their target speed, and the shooter is aimed,
  // isReadyToShoot() returns true. It also returns true if a failsafe timeout occurs.
  // The motor control is voltage drive with FOC, but no PID (PIDs were found to be higher
  // current draw methods than essentially simple duty cycle operations).
  // At this point it is up to the intakeSusbsystem to feed a note into the shooter,
  // which will happen via call from MasterArmSubsystem. When the note is launched, a current
  // spike will occur (> 30 amps, typically 60) which event is used to start a 350 ms timer,
  // and at the end of the timeout the motor is stopped.
  private TalonFX m_shooterMotor = new TalonFX(SC.SHOOTER_FALCON_ID, Constants.CANIVORE_BUS_NAME);
  private CANSparkMax m_aimMotor = new CANSparkMax(SC.AIM_NEO550_ID, MotorType.kBrushless);
  private SparkPIDController m_aimController = m_aimMotor.getPIDController() ;
  private RelativeEncoder m_integratedAimEncoder = m_aimMotor.getEncoder();
  private double m_shooterVoltageOut;
  private double m_shooterTargetVel;
  private double m_aimTargetPos;
  private boolean m_isFarShot;
  private long m_startTime;       // can share between Aim and Shooter, because they
                                  // are logically synced
  private long  m_elapsedTime;
  private double m_velocityError;

  private VoltageOut m_shooterRequest = new VoltageOut(SC.SHOOTER_VOLTAGE_OUT_NEAR)
                                                       .withEnableFOC(true)
                                                       .withUpdateFreqHz(50); 
  private enum ShooterState {
    INITIALIZING_AIM_MOTOR,
    IDLE,
    PREPPING_TO_SHOOT,
    WAITING_FOR_SHOT,
    SHOT_DETECTED
  }

  private ShooterState m_shooterStatus;
  private Supplier<String> m_currentStateName;
  private IntSupplier m_currentSeqNo;
  private FileRecorder m_fileRecorder;
  private static boolean LOGGING_ACTIVE = FileRecorder.isFileRecorderAvail();

  public ShooterSubsystem(Supplier<String> currentStateName, 
                         IntSupplier currentSeqNo,
                         FileRecorder fileRecorder) {
    m_currentStateName = currentStateName;
    m_currentSeqNo = currentSeqNo;
    m_fileRecorder = fileRecorder;
  
    configShooterMotor();
    configAimingMotor();

    m_shooterVoltageOut = SC.SHOOTER_VOLTAGE_OUT_NEAR;
    m_isFarShot = false;

    m_shooterStatus = ShooterState.IDLE;
    // m_aimController.setReference(-(SC.MAX_AIM_POSITION+2), CANSparkMax.ControlType.kPosition);
    // m_startTime = System.currentTimeMillis();
  }

  /***********************************************************************
   * Methods called from MasterArm Conductor to synchronize note handlers
   * ********************************************************************/
  
   // Set shooter speed and aim in prep for a shot
  public void prepareToShoot(boolean isFarShot) {
    m_isFarShot = isFarShot;
    if (m_isFarShot) {     // setup for far shot
      m_aimTargetPos = SC.AIM_POSITION_FAR_SHOT;
      m_shooterTargetVel = SC.SHOOTER_VELOCITY_FAR;
      m_shooterVoltageOut = SC.SHOOTER_VOLTAGE_OUT_FAR;
    } else {
      m_aimTargetPos = SC.AIM_POSITION_NEAR_SHOT;
      m_shooterTargetVel = SC.SHOOTER_VELOCITY_NEAR;
      m_shooterVoltageOut = SC.SHOOTER_VOLTAGE_OUT_NEAR;     
    }
    m_shooterMotor.setControl(m_shooterRequest.withOutput(m_shooterVoltageOut));
    m_aimController.setReference(m_aimTargetPos, CANSparkMax.ControlType.kPosition);
    m_shooterStatus = ShooterState.PREPPING_TO_SHOOT;
    m_startTime = System.currentTimeMillis();
    if (LOGGING_ACTIVE) {
      m_fileRecorder.recordShooterEvent(NoteRequest.SHOOTER_PREP,
                                        m_startTime,
                                        m_shooterTargetVel,
                                        m_shooterVoltageOut,
                                        m_aimTargetPos,
                                        m_currentStateName.get(),
                                        m_currentSeqNo.getAsInt());
    }
  }

  // Report when prep is complete
  public boolean isReadyToShoot() {
    // When up to speed, or upon a timeout, the periodic method will
    // change the ShooterState
    return (m_shooterStatus == ShooterState.WAITING_FOR_SHOT);
  }

  // Get informed when intake subsystem is told to eject
  public void shotInitiated() {
    // This is just a sync method to let the shooter know exactly when
    // a shot is begun. This allows better inrush current lockout for
    // shot Amperage measurement, and a deterministic overall timeout for
    // completeion of a shot, so that the shooter motor can be stopped.
    m_startTime = System.currentTimeMillis();
    if (LOGGING_ACTIVE) {
      m_fileRecorder.recordShooterEvent(NoteRequest.SHOOTER_SCORE,
                                        m_startTime,
                                        m_shooterTargetVel,
                                        m_shooterVoltageOut,
                                        m_aimTargetPos,
                                        m_currentStateName.get(),
                                        m_currentSeqNo.getAsInt());
    }
  }

  // This is a polled method that reports when a current spike has been detected upon a shot (see periodic())
  public boolean isShotDetected() {
    // The periodic method will change the ShooterState to SHOT_DETECTED 
    // after the Amperage spike that occurs upon a shot 
    // (which may be erroneously triggered by the inrush current, so a filter 
    // for that is needed. Do not look until after an initial measurement 
    // lockout period). 
    // If no shot is detected then there is no time out, so MasterArmSubsystem 
    // has its own timeout to ensure the shooter motor is stopped.
    // But the amperage test is still useful - it allows faster response (when it works).
    return (m_shooterStatus == ShooterState.SHOT_DETECTED);
  }

  // method used to shut down the shooter after a shot
  public void cancelShooter() {
    // The shooter waits for the MasterArmSystem to tell it to
    // stop, which will generally be governed by the intakeSubsystem
    // timing out after the call to eject.
    m_shooterMotor.setControl(m_shooterRequest.withOutput(0.0));
    m_aimController.setReference(SC.AIM_POSITION_NEAR_SHOT, CANSparkMax.ControlType.kPosition);
    m_shooterStatus = ShooterState.IDLE;
    if (LOGGING_ACTIVE) {
      m_fileRecorder.recordShooterEvent(NoteRequest.CANCEL_SHOT,
                                        System.currentTimeMillis(),
                                        m_shooterTargetVel,
                                        m_shooterVoltageOut,
                                        m_aimTargetPos,
                                        m_currentStateName.get(),
                                        m_currentSeqNo.getAsInt());
    }
  }

  /*************************************
   * Motor configuration methods
   *************************************/
  public void configShooterMotor() {
  // This is intended to be a general Falcon500 Config method, so everything is configured,
    // even if not used. For intake drive, simple duty cycle output is used.
    var openLoopRampConfig = new OpenLoopRampsConfigs().withDutyCycleOpenLoopRampPeriod(SC.SHOOTER_OPEN_LOOP_RAMP_PERIOD)
                                                       .withVoltageOpenLoopRampPeriod(SC.SHOOTER_OPEN_LOOP_RAMP_PERIOD)
                                                       .withTorqueOpenLoopRampPeriod(0);
    var motorOutputConfig = new MotorOutputConfigs().withNeutralMode(SC.SHOOTER_MOTOR_NEUTRAL_MODE)
                                                    .withInverted(SC.SHOOTER_MOTOR_INVERT)
                                                    .withPeakForwardDutyCycle(SC.SHOOTER_CONTROLLER_OUTPUT_LIMIT)
                                                    .withPeakReverseDutyCycle(-SC.SHOOTER_CONTROLLER_OUTPUT_LIMIT);
                                                    //.withDutyCycleNeutralDeadband(.001);
    var currentLimitConfig = new CurrentLimitsConfigs().withSupplyCurrentLimit(SC.SHOOTER_CONT_CURRENT_LIMIT)
                                                       .withSupplyCurrentThreshold(SC.SHOOTER_PEAK_CURRENT_LIMIT)
                                                       .withSupplyTimeThreshold(SC.SHOOTER_PEAK_CURRENT_DURATION)
                                                       .withSupplyCurrentLimitEnable(SC.SHOOTER_ENABLE_CURRENT_LIMIT);
    var shooterConfig = new TalonFXConfiguration().withMotorOutput(motorOutputConfig)
                                                 .withCurrentLimits(currentLimitConfig)
                                                 .withOpenLoopRamps(openLoopRampConfig);
    StatusCode status = m_shooterMotor.getConfigurator().apply(shooterConfig);
    if (! status.isOK()) {
      System.out.println("Failed to apply Shooter motor configs. Error code: "+status.toString());
    }
  }

  private void reportRevError(REVLibError errorCode) {
    if (errorCode != REVLibError.kOk) {
        System.out.println("ShooterAimMotor RevLibError = "+errorCode.toString());
    }
  }

   public void configAimingMotor() {
      reportRevError(m_aimMotor.restoreFactoryDefaults());
      reportRevError(m_aimMotor.setSmartCurrentLimit(SC.AIM_SMART_CURRENT_LIMIT));
      // setInverted returns void
      m_aimMotor.setInverted(SC.INVERT_AIM_NEO550);
      reportRevError(m_aimMotor.setIdleMode(SC.AIM_MOTOR_NEUTRAL_MODE));
      reportRevError(m_aimController.setP(SC.AIM_KP));
      reportRevError(m_aimController.setI(SC.AIM_KI));
      reportRevError(m_aimController.setD(SC.AIM_KD));
      reportRevError(m_aimController.setFF(SC.AIM_KF));
      reportRevError(m_aimController.setOutputRange(SC.MIN_AIM_CLOSED_LOOP_OUTPUT,
                                                    SC.MAX_AIM_CLOSED_LOOP_OUTPUT));
      reportRevError(m_aimController.setFeedbackDevice(m_integratedAimEncoder));
      reportRevError(m_aimController.setPositionPIDWrappingEnabled(false));
      // reportRevError(m_aimMotor.burnFlash());     // Do this durng development, but not
                                                  // routinely, to preserve the life of the
                                                  // flash memory. Is it even necessary, since
                                                  // all registers (except ID?) are written 
                                                  // via code on every bootup?
      // Do not set software limits. TODO - set limits to allow initialize movement, then reset to normal
      // when initialize is compelte.
  }

/*
  public void setupPublishing() {
    // No need to setup publishing for this subsystem - the output is 
    // all limited to Smartdashboard.
  }
*/
  public void publishShooterData() {
    SmartDashboard.putString("ShooterState ", m_shooterStatus.toString());
    SmartDashboard.putNumber("Aim sensor position ", m_integratedAimEncoder.getPosition());
    SmartDashboard.putNumber("Shooter Voltage Out ",  m_shooterVoltageOut);
    SmartDashboard.putNumber("Shooter Velocity RPS ",  m_shooterMotor.getVelocity().getValueAsDouble());
  }

  @Override
  public void periodic() {
    publishShooterData();

    switch (m_shooterStatus) {
      case IDLE:
        // nothing to do - trigger from InstantCommand call from test button, or
        // state machine, but always via MasterArmSubsstem
        break;

      case PREPPING_TO_SHOOT:
        m_elapsedTime = System.currentTimeMillis() - m_startTime;
        if (m_elapsedTime > 700) {
          new RumbleCmd(2, .5, 200).schedule();
          if (LOGGING_ACTIVE) {
            m_fileRecorder.recordMoveEvent( "Shooter, ",
                                            NoteEvent.TIMEOUT_OCCURED,
                                            m_shooterTargetVel,
                                            m_velocityError,
                                            System.currentTimeMillis(),
                                            m_elapsedTime,
                                            m_currentStateName.get(),
                                            m_currentSeqNo.getAsInt());
          }
          m_shooterStatus = ShooterState.WAITING_FOR_SHOT;
          break;
        }

        if ((Math.abs(m_integratedAimEncoder.getPosition() - m_aimTargetPos) <= SC.ALLOWED_SHOOTER_AIM_ERROR)
            && 
            (m_shooterMotor.getVelocity().getValueAsDouble() > m_shooterTargetVel)) {
          m_shooterStatus = ShooterState.WAITING_FOR_SHOT;
          new RumbleCmd(2, .5, 200).schedule();
          if (LOGGING_ACTIVE) {
            m_fileRecorder.recordMoveEvent( "Shooter, ",
                                            NoteEvent.SETPOINT_REACHED,
                                            m_shooterMotor.getVelocity().getValueAsDouble(),
                                            m_integratedAimEncoder.getPosition(),
                                            System.currentTimeMillis(),
                                            m_elapsedTime,
                                            m_currentStateName.get(),
                                            m_currentSeqNo.getAsInt());
          }
        }
        break;

      case WAITING_FOR_SHOT:
        // Look for current spike and set ShooterState to SHOT_DETECTED if found
        m_elapsedTime = System.currentTimeMillis() - m_startTime;
        if (m_shooterMotor.getSupplyCurrent().getValueAsDouble()
            >
            SC.AMP_THRESHOLD_FOR_NOTE_LAUNCH_DETECTION) {
          if (LOGGING_ACTIVE) {
            m_fileRecorder.recordMoveEvent( "Shooter, ",
                                            NoteEvent.SHOT_DETECTED,
                                            m_shooterMotor.getVelocity().getValueAsDouble(),
                                            m_shooterMotor.getSupplyCurrent().getValueAsDouble(),
                                            System.currentTimeMillis(),
                                            m_elapsedTime,
                                            m_currentStateName.get(),
                                            m_currentSeqNo.getAsInt());
          }
          m_shooterStatus = ShooterState.SHOT_DETECTED;
        } else if (m_elapsedTime > 700) {
          if (LOGGING_ACTIVE) {
            m_fileRecorder.recordMoveEvent( "Shooter (wait for shot), ",
                                            NoteEvent.TIMEOUT_OCCURED,
                                            SC.AMP_THRESHOLD_FOR_NOTE_LAUNCH_DETECTION,
                                            m_shooterMotor.getSupplyCurrent().getValueAsDouble(),
                                            System.currentTimeMillis(),
                                            m_elapsedTime,
                                            m_currentStateName.get(),
                                            m_currentSeqNo.getAsInt());
          }
          m_shooterStatus = ShooterState.SHOT_DETECTED;     // timeout occured, so just pretend spike happened
        }
        break;

      case INITIALIZING_AIM_MOTOR:
        // Aim encoder is the integrated NEO550 rotor, so is not absolute. The plan was to use the hardware stop
        // designed into the aiming mechanism to create a current spike (via starting the aim motor on power up),
        // which event was to be used to initialize the intergrated encoder zero value, regardless of where the 
        // shooter's aim position was on start up. In practice this did not work, because the robot is not
        // enabled on power up. Short of adding an hardware solution in the form of an index sensor, the
        // new approach is to manually index the shooter aim to the indexed position before startup. To help
        // ensure it stays that way, after every shot the software automatically returns the aim to the 
        // indexed position. As a result, the INITIALIZING_AIM_MOTOR state will never be used, but is kept
        // in the code base just in case a sensor approach is resurected (in which case both current and sensor 
        // input detection will be needed). 
        if (((System.currentTimeMillis() - m_startTime) > 1000) 
            ||
            (m_aimMotor.getOutputCurrent() >= SC.AIM_DECTECTION_CURRENT_FOR_STOP)) {
          System.out.println("Initial Aim position reported when against physical stop = "+m_integratedAimEncoder.getPosition());
          // Initialize the actual AIM position, allowing for tension
          m_integratedAimEncoder.setPosition(-2.0);
          // Then take the tension off the hardware stop
          m_aimTargetPos = SC.AIM_POSITION_NEAR_SHOT;
          m_aimController.setReference(m_aimTargetPos, CANSparkMax.ControlType.kPosition);
          // And change the shooter state to IDLE
          m_shooterStatus = ShooterState.IDLE;
        }
        break;

      case SHOT_DETECTED:     // Nothing to do. MasterArmSubsystem will call cancelShooter() when appropriate
        default:
        break;
    }
  }
}

