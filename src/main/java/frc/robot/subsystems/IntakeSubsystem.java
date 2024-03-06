// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
//import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
//import com.ctre.phoenix6.signals.InvertedValue;
//import com.ctre.phoenix6.signals.NeutralModeValue;

import com.ctre.phoenix6.StatusCode;
//import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
//import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.OpenLoopRampsConfigs;
//import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;

//import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.NotableConstants.IC;

public class IntakeSubsystem extends SubsystemBase {
    private TalonFX m_intakeMotor;
    private static boolean m_intakeIsRunning = false;        // True when in Acquire / Eject mode
    private static double m_intakeSpeedFactor = IC.HOLD_NOTE;
    private double m_intakeBaseSpeed = IC.INTAKE_BASE_SPEED;
    private static long m_startTime;
    //private DigitalInput m_intakeCompleteSensor = new DigitalInput(IC.INTAKE_SENSOR_PORT_ID);

    // Declare Phoenix6 control request objects for the Intake Motor:
    // No need for VelocityVoltage PID control (with or without arbitrary feed forward)
    // for note pickup: just use DutyCycle out at 90% for pickup and release, and
    // 1% or s0 for holding.
    // The Update rate is set to 0, so need to refresh motor drive at least 
    // every 50 ms (at 20 ms per loop, should be good)
    private final DutyCycleOut m_intakeCtrl = new DutyCycleOut(m_intakeBaseSpeed * m_intakeSpeedFactor).withUpdateFreqHz(0);
  
  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() { 
    m_intakeMotor = new TalonFX(IC.INTAKE_FALCON_MOTOR_ID, Constants.CANIVORE_BUS_NAME);
    configIntakeMotor();
  }

  public void acquireNote() {
    m_intakeIsRunning = true;
    m_intakeSpeedFactor = IC.ACQUIRE_NOTE;
    m_startTime = System.currentTimeMillis();
  }

  public void holdNote() {
    m_intakeIsRunning = true;
    m_intakeSpeedFactor = IC.HOLD_NOTE;
  }

  public void ejectNote() {
    m_intakeIsRunning = true;
    m_intakeSpeedFactor = IC.EJECT_NOTE;
    m_intakeMotor.setControl(m_intakeCtrl.withOutput(m_intakeBaseSpeed * m_intakeSpeedFactor));
    m_startTime = System.currentTimeMillis();
  }

  public void stopIntake() {
    m_intakeIsRunning = false;
    m_intakeSpeedFactor = 0;
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
    // This is intended to be a general Falcon500 Config method, so everything is configured,
    // even if not used. For intake drive, simple duty cycle output is used.
    var openLoopRampConfig = new OpenLoopRampsConfigs().withDutyCycleOpenLoopRampPeriod(IC.INTAKE_OPEN_LOOP_RAMP_PERIOD)
                                                       .withVoltageOpenLoopRampPeriod(IC.INTAKE_OPEN_LOOP_RAMP_PERIOD)
                                                       .withTorqueOpenLoopRampPeriod(0);
  /*  var closedLoopRampConfig = new ClosedLoopRampsConfigs().withDutyCycleClosedLoopRampPeriod(IC.INTAKE_CLOSED_LOOP_RAMP_PERIOD)
                                                       .withVoltageClosedLoopRampPeriod(IC.INTAKE_CLOSED_LOOP_RAMP_PERIOD)
                                                       .withTorqueClosedLoopRampPeriod(0);
    var feedbackConfig = new FeedbackConfigs().withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor)
                                              .withSensorToMechanismRatio(IC.INTAKE_GEAR_RATIO)
                                              .withRotorToSensorRatio(1.0);
  */
    var motorOutputConfig = new MotorOutputConfigs().withNeutralMode(IC.INTAKE_MOTOR_NEUTRAL_MODE)
                                                    .withInverted(IC.INTAKE_MOTOR_INVERT)
                                                    .withPeakForwardDutyCycle(IC.INTAKE_CONTROLLER_OUTPUT_LIMIT)
                                                    .withPeakReverseDutyCycle(-IC.INTAKE_CONTROLLER_OUTPUT_LIMIT);
                                                    //.withDutyCycleNeutralDeadband(.001);
    var currentLimitConfig = new CurrentLimitsConfigs().withSupplyCurrentLimit(IC.INTAKE_CONT_CURRENT_LIMIT)
                                                       .withSupplyCurrentThreshold(IC.INTAKE_PEAK_CURRENT_LIMIT)
                                                       .withSupplyTimeThreshold(IC.INTAKE_PEAK_CURRENT_DURATION)
                                                       .withSupplyCurrentLimitEnable(IC.INTAKE_ENABLE_CURRENT_LIMIT);
    /*
    var pid0Configs = new Slot0Configs().withKP(IC.INTAKE_KP)
                                        .withKI(IC.INTAKE_KI)
                                        .withKD(IC.INTAKE_KD)
                                        .withKS(IC.INTAKE_KS)
                                        .withKV(IC.INTAKE_KV)
                                        .withKA(IC.INTAKE_KA)
                                        .withKG(IC.INTAKE_KG);
                                        // .withGravityType(   );   // Elevator_Static if constant, Arm_Cosign if variable. Sensor must be 0 when mechanism
    */
    var intakeConfig = new TalonFXConfiguration()     //.withFeedback(feedbackConfig)
                                                 .withMotorOutput(motorOutputConfig)
                                                 .withCurrentLimits(currentLimitConfig)
                                                 .withOpenLoopRamps(openLoopRampConfig);
                                                      //.withClosedLoopRamps(closedLoopRampConfig)
                                                      //withSlot0(pid0Configs);
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
    // The min time requirement here ensures the optical sensor will not see the note
    // before it is fully captured.
    // This method is used as a trigger to call holdNote() !
    // Temporary hack - return true after 10 seconds
    if (m_intakeSpeedFactor == IC.ACQUIRE_NOTE) {
      // return (! m_intakeCompleteSensor.get());
      // return ((System.currentTimeMillis() - m_startTime) > 10000);
      return false;
    } else {
      return isNoteHeld();
    }
  }

  public boolean isNoteHeld() {
    // This method returns true if a Note is being held, but since the Intake sensor can only see the note when
    // the Arms are extended, need to infer presence of Note. If m_intakeSpeedFactor is equal to IC.HOLD_NOTE,
    // and m_intakeIsRunning, assume we ar eholding a Note.
    // can't use: return (! m_intakeCompleteSensor.get()), so return this instead:
    return (m_intakeSpeedFactor == IC.HOLD_NOTE) && m_intakeIsRunning;
  }

  public void simulateNoteAcquired() {
    if (m_intakeSpeedFactor == 1) {
      holdNote();
    }
  }

  public boolean isIntakeStopped() {
    return (! m_intakeIsRunning);
  }
  
  public void publishIntakeData() {
    //SmartDashboard.putNumber("Intake Motor Velocity = ", m_intakeMotor.getVelocity().getValueAsDouble());
    //SmartDashboard.putNumber("iSupply Amps", m_intakeMotor.getSupplyCurrent().getValueAsDouble());
    //SmartDashboard.putNumber("Base Speed", m_intakeBaseSpeed);
    SmartDashboard.putNumber("Mode (+1, .025, -1, or 0)", m_intakeSpeedFactor);
    //SmartDashboard.putString("Intake Has Note = ", isNoteHeld() ? "Yes" : "No");
  }

  @Override
  public void periodic() {
    if (m_intakeSpeedFactor == IC.EJECT_NOTE) {
      // Note ejection in progress - is it complete?
      if (System.currentTimeMillis() - m_startTime > 1000) {
        stopIntake();
      }
    }
    publishIntakeData();
  
    if (m_intakeIsRunning) {
      maintainIntakeSpeed();
      if (isNoteAcquired()) {
        holdNote();
      }
    }
  }
}
