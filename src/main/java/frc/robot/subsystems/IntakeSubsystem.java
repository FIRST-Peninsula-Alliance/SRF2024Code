// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
//import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
//import com.ctre.phoenix6.signals.InvertedValue;
//import com.ctre.phoenix6.signals.NeutralModeValue;

import java.util.function.IntSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.OpenLoopRampsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;

//import edu.wpi.first.wpilibj.DutyCycleEncoder;
//import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.FileRecorder;
import frc.lib.util.FileRecorder.NoteRequest;
import frc.robot.Constants;
import frc.robot.NotableConstants.IC;

public class IntakeSubsystem extends SubsystemBase {
    private TalonFX m_intakeMotor;
    private static boolean m_intakeIsRunning = false;        // True when in Acquire / Eject mode
    private static double m_intakeSpeedFactor = IC.HOLD_NOTE;
    private double m_intakeBaseSpeed = IC.INTAKE_BASE_SPEED;
    private static long m_startTime;

    //  private PWM m_lidar = new PWM(IC.INTAKE_PWM_SENSOR_PORT_ID);
    //  private DutyCycleEncoder m_lidar = new DutyCycleEncoder(IC.INTAKE_PWM_SENSOR_PORT_ID);
    //  private double m_intakeSensorDistance;
    //  private PWM m_lidarDistance;

    private Supplier<String> m_currentStateName;
    private IntSupplier m_currentSeqNo;
    private FileRecorder m_fileRecorder;
    private static boolean LOGGING_ACTIVE = FileRecorder.isFileRecorderAvail();

    // Declare Phoenix6 control request objects for the Intake Motor:
    // No need for VelocityVoltage PID control (with or without arbitrary 
    // feed forward) for note pickup: just use DutyCycle out at 90% for
    // pickup and release, and 25% for holding.
    // The Update rate is set to 0, so need to refresh motor drive at least 
    // every 50 ms (at 20 ms per loop, should be good)
    private final DutyCycleOut m_intakeCtrl = new DutyCycleOut(m_intakeBaseSpeed * m_intakeSpeedFactor).withUpdateFreqHz(0);
  
  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem(Supplier<String> currentStateName, 
                         IntSupplier currentSeqNo,
                         FileRecorder fileRecorder) {
    m_currentStateName = currentStateName;
    m_currentSeqNo = currentSeqNo;
    m_fileRecorder = fileRecorder;
    m_intakeMotor = new TalonFX(IC.INTAKE_FALCON_MOTOR_ID, Constants.CANIVORE_BUS_NAME);
    configIntakeMotor();
  }

  // All methods that require motor action set the m_intakeSpeedFactor
  // variable to the sesired value. periodic() will poll this variable
  // and send the appropriate control request to the motor accordingly.
  public void acquireNote() {
    m_intakeIsRunning = true;
    m_intakeSpeedFactor = IC.ACQUIRE_NOTE;
    m_startTime = System.currentTimeMillis();
    if (LOGGING_ACTIVE) {
      m_fileRecorder.recordIntakeEvent(NoteRequest.INTAKE_ACQUIRE,
                                       m_startTime,
                                       m_currentStateName.get(),
                                       m_currentSeqNo.getAsInt());
    }
  }

  public void holdNote() {
    m_intakeIsRunning = true;
    m_intakeSpeedFactor = IC.HOLD_NOTE;
    if (LOGGING_ACTIVE) {
      m_fileRecorder.recordIntakeEvent( NoteRequest.INTAKE_HOLD,
                                        System.currentTimeMillis(),
                                        m_currentStateName.get(),
                                        m_currentSeqNo.getAsInt());
    }
  }

  public void ejectNote() {
    m_intakeIsRunning = true;
    m_intakeSpeedFactor = IC.EJECT_NOTE;
    m_startTime = System.currentTimeMillis();
    if (LOGGING_ACTIVE) {
      m_fileRecorder.recordIntakeEvent( NoteRequest.INTAKE_EJECT,
                                        m_startTime,
                                        m_currentStateName.get(),
                                        m_currentSeqNo.getAsInt());
    }
  }

  public void stopIntake() {
    m_intakeIsRunning = false;
    m_intakeSpeedFactor = 0;
    if (LOGGING_ACTIVE) {
      m_fileRecorder.recordIntakeEvent( NoteRequest.INTAKE_STOP,
                                        System.currentTimeMillis(),
                                        m_currentStateName.get(),
                                        m_currentSeqNo.getAsInt());
    }
  }

  public void cancelIntake() {
    stopIntake();
  }

  public boolean isIntakeIdle() {
    return isIntakeStopped();
  }
    
  public void maintainIntakeSpeed() {
    m_intakeCtrl.Output = m_intakeBaseSpeed * m_intakeSpeedFactor;
    if (m_intakeSpeedFactor == 0.0) {
      m_intakeMotor.stopMotor();
    } else {
      m_intakeMotor.setControl(m_intakeCtrl);
    }
  }

  private void configIntakeMotor() {
    // For intake drive, simple duty cycle output is used.
    var openLoopRampConfig = new OpenLoopRampsConfigs().withDutyCycleOpenLoopRampPeriod(IC.INTAKE_OPEN_LOOP_RAMP_PERIOD)
                                                       .withVoltageOpenLoopRampPeriod(IC.INTAKE_OPEN_LOOP_RAMP_PERIOD)
                                                       .withTorqueOpenLoopRampPeriod(0);
    var motorOutputConfig = new MotorOutputConfigs().withNeutralMode(IC.INTAKE_MOTOR_NEUTRAL_MODE)
                                                    .withInverted(IC.INTAKE_MOTOR_INVERT)
                                                    .withPeakForwardDutyCycle(IC.INTAKE_CONTROLLER_OUTPUT_LIMIT)
                                                    .withPeakReverseDutyCycle(-IC.INTAKE_CONTROLLER_OUTPUT_LIMIT);
                                                    //.withDutyCycleNeutralDeadband(.001);
    var currentLimitConfig = new CurrentLimitsConfigs().withSupplyCurrentLimit(IC.INTAKE_CONT_CURRENT_LIMIT)
                                                       .withSupplyCurrentThreshold(IC.INTAKE_PEAK_CURRENT_LIMIT)
                                                       .withSupplyTimeThreshold(IC.INTAKE_PEAK_CURRENT_DURATION)
                                                       .withSupplyCurrentLimitEnable(IC.INTAKE_ENABLE_CURRENT_LIMIT);
    var intakeConfig = new TalonFXConfiguration().withMotorOutput(motorOutputConfig)
                                                 .withCurrentLimits(currentLimitConfig)
                                                 .withOpenLoopRamps(openLoopRampConfig);
    StatusCode status = m_intakeMotor.getConfigurator().apply(intakeConfig);

    if (! status.isOK()) {
      System.out.println("Failed to apply Intake configs. Error code: "+status.toString());
    }
  }

  public double getVelocityRPS() {
    return m_intakeMotor.getVelocity().getValueAsDouble(); 
  }

  public boolean isNoteAcquired() {
    // This method will only look for a note when trying to acquire one
    // i.e. not when holding, and not when ejecting, a note.
    // This method is used as a trigger to call holdNote() !
    if (m_intakeSpeedFactor == IC.ACQUIRE_NOTE) {
      // return (! m_intakeCompleteSensor.get());
      return false;
    } else {
      return isNoteHeld();
    }
  }

  public boolean isNoteHeld() {
    // This method returns true if a Note is being held, but since the 
    // Intake sensor can only see the note when the inner Arm is extended, 
    // need to infer presence of Note. If m_intakeSpeedFactor is equal to 
    // IC.HOLD_NOTE, and m_intakeIsRunning, assume we ar eholding a Note.
    return (m_intakeSpeedFactor == IC.HOLD_NOTE) && m_intakeIsRunning;
  }

  public void triggerNoteAcquired() {
    if (m_intakeSpeedFactor == IC.ACQUIRE_NOTE) {
      holdNote();
    }
  }

  public boolean isIntakeStopped() {
    return (! m_intakeIsRunning);
  }
  
  public void publishIntakeData() {
    SmartDashboard.putNumber("Intake Mode = ", m_intakeSpeedFactor);
  }

  public void checkLidarNoteDetection() {
    // only check if actively acquiring a note
    //if (m_intakeSpeedFactor == IC.ACQUIRE_NOTE) {
      //m_intakeSensorDistance = m_lidar.getPulseTimeMicroseconds() * 0.1; // Convert raw PWM 
                                                                         // value to 
                                                                         // distance in cm
      //m_intakeSensorDistance = m_lidar.get();
      //SmartDashboard.putNumber("Intake sensor distance = ", m_intakeSensorDistance);
      //if (m_intakeSensorDistance < IC.NOTE_ACQUIRED_DISTANCE_THRESHOLD) {
        // If sensor is accurate, might replace mannual LT button. To help ensure reliability,
        // add a trigger debounce time of at least .25 sec.
        //triggerNoteAcquired();
      //}
    //}
  }

  @Override
  public void periodic() {
    if (m_intakeSpeedFactor == IC.EJECT_NOTE) {
      // Note ejection in progress - is it complete?
      if (System.currentTimeMillis() - m_startTime > 1000) {
        stopIntake();
      }
    }
    checkLidarNoteDetection();
    publishIntakeData();

    if (m_intakeIsRunning) {
      maintainIntakeSpeed();
    }
  }
}
