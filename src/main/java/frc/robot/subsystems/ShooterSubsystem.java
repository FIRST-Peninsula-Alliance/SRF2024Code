// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

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

// import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.NotableConstants.SC;

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

  private VoltageOut m_shooterRequest = new VoltageOut(SC.SHOOTER_VOLTAGE_OUT_NEAR)
                                                       .withEnableFOC(true)
                                                       .withUpdateFreqHz(50); 
  private enum ShooterState {
    IDLE,
    PREPPING_TO_SHOOT,
    WAITING_FOR_SHOT,
    SHOT_DETECTED
  }

  private ShooterState m_shooterStatus;

  public ShooterSubsystem() {
    configShooterMotor();
    configAimingMotor();
    m_shooterStatus = ShooterState.IDLE;
    m_isFarShot = false;
  }

  // Set shooter speed and aim
  public void prepareToShoot(boolean isFarShot) {
    m_isFarShot = isFarShot;
    if (isFarShot) {     // setup for far shot
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
  }

  public boolean isReadyToShoot() {
    // When up to speed, or upon a timeout, the periodic method will
    // change the ShooterState
    return (m_shooterStatus == ShooterState.WAITING_FOR_SHOT);
  }

  public void shotInitiated() {
    // This is just a sync method to let the shooter know exactly when
    // a shot is begun. This allows better inrush current lockout for
    // shot Amperage measurement, and a deterministic overall timeout for
    // completeion of a shot, so that the shooter motor can be stopped.
    m_startTime = System.currentTimeMillis();
  }

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

  public void cancelShooter() {
    // The shooter waits for the MasterArmSystem to tell it to
    // stop, which will generally be governed by the intakeSubsystem
    // timing out after the call to eject.
    m_shooterMotor.setControl(m_shooterRequest.withOutput(0.0));
    m_shooterStatus = ShooterState.IDLE;
  }

  /**********************************************
   * Methods for developing and testing Aim motor
   ***********************************************/
  // @param isDistantShot
  public void toggleShooterAimTest() {
    if (m_isFarShot) {     // setup for far shot
      m_isFarShot = false;
      m_aimTargetPos = SC.AIM_POSITION_NEAR_SHOT;
    } else {
      m_isFarShot = true;
      m_aimTargetPos = SC.AIM_POSITION_FAR_SHOT;
    }
    m_aimController.setReference(m_aimTargetPos, CANSparkMax.ControlType.kPosition);
  }

  public void toggleTestShooter() {
    if (m_shooterStatus == ShooterState.IDLE) {
      if (m_isFarShot) {
        m_shooterVoltageOut = SC.SHOOTER_VOLTAGE_OUT_FAR;
        m_shooterTargetVel = SC.SHOOTER_VELOCITY_FAR;
      } else {
        m_shooterVoltageOut = SC.SHOOTER_VOLTAGE_OUT_NEAR;
        m_shooterTargetVel = SC.SHOOTER_VELOCITY_NEAR;
      }
      m_shooterMotor.setControl(m_shooterRequest
                                .withOutput(m_shooterVoltageOut)
                                .withEnableFOC(true)
                                .withUpdateFreqHz(50));
      m_shooterStatus = ShooterState.PREPPING_TO_SHOOT;
    } else {
      cancelShooter();
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
      reportRevError(m_aimMotor.setSmartCurrentLimit(SC.AIM_CONT_CURRENT_LIMIT));
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
      //reportRevError(m_aimController.setPositionPIDWrappingMinInput(0));
      //reportRevError(m_aimController.setPositionPIDWrappingMaxInput(360));
      reportRevError(m_aimMotor.burnFlash());     // Do this durng development, but not
                                                  // routinely, to preserve the life of the
                                                  // flash memory. Is it even necessary, since
                                                  // all registers (except ID?) are written 
                                                  // via code on every bootup?
      //SmartDashboard.putString("AIM Motor Setup", "Complete");
  }
/*
  public void setupPublishing() {
    // No need for publishing setup for this subsystem - the output is 
    // all limitd to Smartdashboard.
  }
*/
  public void publishShooterData() {
    SmartDashboard.putString("ShooterState ", m_shooterStatus.toString());
    SmartDashboard.putNumber("Aim sensor position ", m_integratedAimEncoder.getPosition());
    // SmartDashboard.putNumber("Shooter Voltage Out ",  m_shooterVoltageOut);
    // SmartDashboard.putNumber("Aim MotorAmps", m_aimMotor.getOutputCurrent());

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
        if (((Math.abs(m_integratedAimEncoder.getPosition() - m_aimTargetPos) 
             <= SC.ALLOWED_SHOOTER_AIM_ERROR)
             && 
             (m_shooterMotor.getVelocity().getValueAsDouble() > m_shooterTargetVel)
            )
            ||
            (System.currentTimeMillis() - m_startTime > 700)) {
          m_shooterStatus = ShooterState.WAITING_FOR_SHOT;
        }
        break;

      case WAITING_FOR_SHOT:
        // Look for current spike and set ShooterState to SHOT_DETECTED
        // if found
        if (m_shooterMotor.getSupplyCurrent().getValueAsDouble()
            >
            SC.AMP_THRESHOLD_FOR_NOTE_LAUNCH_DETECTION) {
          m_shooterStatus = ShooterState.SHOT_DETECTED;
        } else if ((System.currentTimeMillis() - m_startTime) > 600) {
          m_shooterStatus = ShooterState.SHOT_DETECTED;     // timeout occured, so just pretend spike happened
        }
        break;

      case SHOT_DETECTED:
      default:                // Nothing to do, no timeout
        break;
      }
  }
}

