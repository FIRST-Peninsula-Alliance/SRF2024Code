// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.CANSparkMax;

// For reference, a Falcon500 has max rotational velocity of 6380 RPM +/- 10%. 

public final class NotableConstants {           // As in all these constants are used to 
                                                // assist with Note acqusition and scoring.

    public static final double MAX_FALCON500_RPS = 6380.0 / 60.0;       // ~ 106
    public static final double MAX_NEO550_RPS = 11000.0 / 60.0;         // ~ 183

    public static final int TEST_SETPOINTS_DATA_COL = 0;
    public static final int TEST_SETPOINTS_DATA_ROW = 0;
    public static final int TEST_SETPOINTS_DATA_LIST_HGT = 4;

    /************************************************************************
     * Master Arm Constants
     ************************************************************************/
    public static final class MAC {             // Master arm constants
        public static final int MASTER_ARM_FALCON_ID = 10;
        public static final NeutralModeValue MASTER_ARM_MOTOR_NEUTRAL_MODE = NeutralModeValue.Brake;
        public static final InvertedValue MASTER_ARM_MOTOR_INVERT = InvertedValue.Clockwise_Positive;

        public static final int MASTER_ARM_ENCODER_ID = 10;
        public static final double MASTER_ARM_ENCODER_MAGNET_OFFSET = -0.3679;
        public static final AbsoluteSensorRangeValue MASTER_ARM_CANCODER_RANGE = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
        public static final SensorDirectionValue MASTER_ARM_CANCODER_DIR = SensorDirectionValue.CounterClockwise_Positive;

        // Setup for fused remote sensor, 1:1 on output shaft (possible with licensed Phoenix6 Pro)
        public static final double MASTER_ARM_ENCODER_TO_AXLE_RATIO   = 1.0;
        public static final double MASTER_ARM_PLANETARY_GEARBOX_RATIO = 30;
        public static final double MASTER_ARM_CHAIN_DRIVE_GEAR_RATIO  = 30.0 / 12.0;
        public static final double MASTER_ARM_ROTOR_TO_ENCODER_RATIO  = MASTER_ARM_PLANETARY_GEARBOX_RATIO *
                                                                        MASTER_ARM_CHAIN_DRIVE_GEAR_RATIO;

        // The master arm draws a lot of current for a brief time - approaching 60 Amps stator current -
        // in order to rapidly reach the various setpoints. Try to allow that while still kicking in
        // with protective current limits by using a combination of 
        public static final double MASTER_ARM_SUPPLY_CURRENT_LIMIT         = 35.0;
        public static final double MASTER_ARM_SUPPLY_CURRENT_THRESHOLD     = 70.0;
        public static final double MASTER_ARM_SUPPLY_TIME_THRESHOLD        = 0.3;
        public static final boolean MASTER_ARM_ENABLE_CURRENT_LIMIT        = true;
        public static final double MASTER_ARM_STATOR_CURRENT_LIMIT         = 55.0;
        public static final boolean MASTER_ARM_ENABLE_STATOR_CURRENT_LIMIT = true;

        public static final double MASTER_ARM_OPEN_LOOP_RAMP_PERIOD   = 0.25;
        public static final double MASTER_ARM_CLOSED_LOOP_RAMP_PERIOD = 0.0;

        //Master Arm Motor PID Values
        public static final double MASTER_ARM_KP = 6.0;
        public static final double MASTER_ARM_KI = 0.0;
        public static final double MASTER_ARM_KD = 1.0;
        public static final double MASTER_ARM_KF = 0.0;
        public static final double MASTER_ARM_KS = 0.25; 
        public static final double MASTER_ARM_KV = 8.0;
        public static final double MASTER_ARM_KA = 0.6;
        public static final double MASTER_ARM_KG = 0.35;
        public static final double MASTER_ARM_MOTION_MAGIC_ACCEL = 8.0;
        public static final double MASTER_ARM_MOTION_MAGIC_VEL   = 4.0;
        public static final double MASTER_ARM_MOTION_MAGIC_kA    = 0.0;
        public static final double MASTER_ARM_MOTION_MAGIC_kV    = 0.0;
        public static final double MASTER_ARM_MOTION_MAGIC_JERK  = 30.0;

        public static final double MIN_MASTER_ARM_CLOSED_LOOP_OUTPUT = -1.0;
        public static final double MAX_MASTER_ARM_CLOSED_LOOP_OUTPUT =  1.0;
        public static final double MASTER_ARM_OUTPUT_LIMIT_FACTOR    =  0.8;

        // Position units are all in rotations
        public static final double DISTANT_SPEAKER_SHOT_POS     = -0.1925;
        public static final double INDEXED_SPEAKER_SHOT_POS     = -0.2076;
        public static final double LOW_SAFE_TO_ROTATE_IN_POS    = -0.2116;
        public static final double LOW_SAFE_TO_ROTATE_OUT_POS   = -0.17;
        public static final double NOTE_PICKUP_POS              = -0.154;
        public static final double MID_SAFE_TO_ROTATE_POS       = -0.125;
        public static final double HIGH_SAFE_TO_ROTATE_POS      =  0.19;
        public static final double AMP_SHOT_POS                 =  0.2466;
        public static final double MASTER_ARM_HORIZ_POS         =  0.0;

        public static final double ALLOWED_MASTER_ARM_POS_ERROR = 1.5 / 360.0;
        public static final long   ALLOWED_MILLIS_MA_SMALL_MOVE = 400;
        public static final long   ALLOWED_MILLIS_MA_LARGE_MOVE = 700;

        public static final double  MAX_MASTER_ARM_SOFT_LIMIT     =  0.26;
        public static final double  MIN_MASTER_ARM_SOFT_LIMIT     = -0.245;
        public static final boolean ENABLE_MASTER_ARM_SOFT_LIMITS = true;

        public static final int MASTER_ARM_DATA_COL      = 0;
        public static final int MASTER_ARM_DATA_ROW      = 0;
        public static final int MASTER_ARM_DATA_LIST_HGT = 8;
    }

    /*************************************************************
     * Inner Arm Constants
     *************************************************************/
    public static final class IAC {             // Inner arm constants
        public static final int INNER_ARM_FALCON_ID = 11;
        public static final NeutralModeValue INNER_ARM_MOTOR_NEUTRAL_MODE = NeutralModeValue.Brake;
        public static final InvertedValue INNER_ARM_MOTOR_INVERT = InvertedValue.Clockwise_Positive;
        
        public static final int INNER_ARM_CANCODER_ID = 11;
        public static final double INNER_ARM_CANCODER_MAGNET_OFFSET = -0.242625;
        public static final AbsoluteSensorRangeValue INNER_ARM_CANCODER_RANGE = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
        public static final SensorDirectionValue INNER_ARM_CANCODER_DIR = SensorDirectionValue.CounterClockwise_Positive;

        public static final double INNER_ARM_CANCODER_TO_AXLE_RATIO  = 20.0/18.0;        // Chain drive sprockets
        public static final double INNER_ARM_ROTOR_TO_CANCODER_RATIO = 30.0;            // Panetary gear box

        public static final double  INNER_ARM_CONT_CURRENT_LIMIT          = 16.0;
        public static final double  INNER_ARM_PEAK_CURRENT_LIMIT          = 36.0;
        public static final double  INNER_ARM_PEAK_CURRENT_DURATION       = 0.1;
        public static final boolean INNER_ARM_ENABLE_CURRENT_LIMIT        = true;
        public static final double  INNER_ARM_STATOR_CURRENT_LIMIT        = 14.0;
        public static final boolean INNER_ARM_ENABLE_STATOR_CURRENT_LIMIT = true;

        public static final double INNER_ARM_OPEN_LOOP_RAMP_PERIOD   = 0.25;
        public static final double INNER_ARM_CLOSED_LOOP_RAMP_PERIOD = 0.5;

        //INNER Arm Motor PID Values
        public static final double INNER_ARM_KP = 40.0;
        public static final double INNER_ARM_KI = 0.5;
        public static final double INNER_ARM_KD = 0.1;
        public static final double INNER_ARM_KF = 0.0;
        public static final double INNER_ARM_KS = 0.3;
        public static final double INNER_ARM_KV = 4.0;
        public static final double INNER_ARM_KA = 0.1;
        public static final double INNER_ARM_KG = 0.5;
        public static final double INNER_ARM_MOTION_MAGIC_ACCEL = 8.0;
        public static final double INNER_ARM_MOTION_MAGIC_VEL   = 5.0;
        public static final double INNER_ARM_MOTION_MAGIC_kA    = 0.0;
        public static final double INNER_ARM_MOTION_MAGIC_kV    = 0.0;
        public static final double INNER_ARM_MOTION_MAGIC_JERK  = 25.0;

        public static final double INNER_ARM_OUTPUT_LIMIT_FACTOR = 1.0;

        // VERTICAL for the inner arm means intake fingers pointing up,
        // and rotation up from the defined Zero position (fingers horizontal
        // pointing forward) is reported as a negative rotation by the
        // inner arm absolute CANcoder.
        public static final double HORIZONTAL_FORWARD_POS   =  0.0;
        public static final double VERTICAL_UP_POS          = -0.25;
        public static final double VERTICAL_DOWN_POS        =  0.25;
        public static final double HORIZONTAL_BACK_POS      = -0.445;   // -.5, but with safer margin of error

        public static final double BUMPER_CONTACT_POS       =  0.11;
        public static final double NOTE_PICKUP_POS          =  0.124;
        public static final double DISTANT_SPEAKER_GOAL_POS = -0.3572;
        public static final double INDEXED_SPEAKER_GOAL_POS = -0.3428;
        public static final double AMP_GOAL_POS             =  0.041;

        public static final double ALLOWED_INNER_ARM_POS_ERROR  = 5.0 / 360.0;

        // For testing and tuning, make the timeout per move 2 seconds and 500 ms
        // for large and small moves, respectively, and log the results using
        // FileRecorder. 
        // Note the avg times to setpoint, but use max times to setpoint when entering
        // final timeout values. 
        // Retest with different ALLOWED_INNER_ARM_POS_ERROR values to find optimal 
        // tolerance value for MotionMagicVoltage PID being used.
        public static final long ALLOWED_MILLIS_IA_SMALL_MOVE = 300;        //400;
        public static final long ALLOWED_MILLIS_IA_LARGE_MOVE = 400;        //2000;

        // to ensure inner arm always rotates in desired direction,
        // do not enable continuoue mode.
        public static final double  MAX_INNER_ARM_SOFT_LIMIT     =  0.25;
        public static final double  MIN_INNER_ARM_SOFT_LIMIT     = -0.5;
        public static final boolean ENABLE_INNER_ARM_SOFT_LIMITS = true;
 
        public static final int INNER_ARM_DATA_COL      = 2;
        public static final int INNER_ARM_DATA_ROW      = 0;
        public static final int INNER_ARM_DATA_LIST_HGT = 8;
    }

/********************************************************************
 * Intake Constants
 ********************************************************************/
    public static final class IC {              // "Intake Constants"
        public static final int INTAKE_FALCON_MOTOR_ID = 12;
        public static final boolean INVERT_INNER_ARM_FALCON = false;
        public static final int INTAKE_PWM_SENSOR_PORT_ID = 0;

        public static final double  INTAKE_CONT_CURRENT_LIMIT          = 15.0;
        public static final double  INTAKE_PEAK_CURRENT_LIMIT          = 30.0;
        public static final double  INTAKE_PEAK_CURRENT_DURATION       = 0.1;
        public static final boolean INTAKE_ENABLE_CURRENT_LIMIT        = true;
        public static final double  INTAKE_STATOR_CURRENT_LIMIT        = 12.0;
        public static final boolean INTAKE_ENABLE_STATOR_CURRENT_LIMIT = true;        

        public static final double  INTAKE_OPEN_LOOP_RAMP_PERIOD   = 0.2;
        public static final double  INTAKE_CLOSED_LOOP_RAMP_PERIOD = 0.0;

        // With a total intake gear ratio of 13.33,
        // max output shaft rotation velocity is ~ 480 RPM, or 8 RPS. 
        // With roller diameter of ~ 1.5", max belt speed is ~ 37 in/sec. With a Note diameter of 14",
        // and needing to pull in slightly more then 1/2 of that for retention, if we want acqusition in
        // under 1/2 second, it seems reasonable to shoot for a working belt speed of 18 to 24 in/sec.
        public static final double INTAKE_GEAR_RATIO                   = 10 * (24/18);
        public static final NeutralModeValue INTAKE_MOTOR_NEUTRAL_MODE = NeutralModeValue.Coast;
        public static final InvertedValue INTAKE_MOTOR_INVERT          = InvertedValue.CounterClockwise_Positive;
        public static final double INTAKE_CONTROLLER_OUTPUT_LIMIT      = 1.0;
    
        public static final double INTAKE_BASE_SPEED = 0.9;         // Runs in DutyCycleOut mode

        // The following are percent output modifiers. They also serve to identify the purpose
        // of operation of active intake belts
        public static final double HOLD_NOTE    =  0.25;
        public static final double ACQUIRE_NOTE =  1.0;
        public static final double EJECT_NOTE   = -1.0;

        public static final double NOTE_ACQUIRED_DISTANCE_THRESHOLD = 40.0;     // anything less than 30 cm
                                                                                // means note detected
    }

    /********************************************************************
     * Shooter Constants
     ********************************************************************/
    public static final class SC {             // Shooter constants
        public static final int SHOOTER_FALCON_ID = 13;     // Runs in VoltageOut mode

        public static final NeutralModeValue SHOOTER_MOTOR_NEUTRAL_MODE = NeutralModeValue.Coast;
        public static final InvertedValue SHOOTER_MOTOR_INVERT = InvertedValue.CounterClockwise_Positive;
        public static final double SHOOTER_CONTROLLER_OUTPUT_LIMIT = 1.0;
        
        public static final double  SHOOTER_CONT_CURRENT_LIMIT          = 20.0;
        public static final double  SHOOTER_PEAK_CURRENT_LIMIT          = 80.0;
        public static final double  SHOOTER_PEAK_CURRENT_DURATION       = 0.1;
        public static final boolean SHOOTER_ENABLE_CURRENT_LIMIT        = true;
        public static final double  SHOOTER_STATOR_CURRENT_LIMIT        = 18.0;
        public static final boolean SHOOTER_ENABLE_STATOR_CURRENT_LIMIT = true;

        public static final double SHOOTER_OPEN_LOOP_RAMP_PERIOD = 0.2;
        
        public static final double SHOOTER_VOLTAGE_OUT_NEAR = 10.0;       // volts, use 9.0 for double wheel shooter
        public static final double SHOOTER_VOLTAGE_OUT_FAR  = 11.5;
        public static final double SHOOTER_VELOCITY_NEAR    = 69.0;       // Estimated - needs measurment
        public static final double SHOOTER_VELOCITY_FAR     = 88.0;       // ditto
        public static final double MIN_SHOOTER_SPEED        = 55.0;
        public static final double MIN_SHOOTER_VOLTAGE      =  3.0;
        public static final double MAX_SHOOTER_VOLTAGE      = 12.5;

        public static final double AMP_THRESHOLD_FOR_NOTE_LAUNCH_DETECTION = 19.0;

        // Shooter Aim Motor parameters
        public static final int AIM_NEO550_ID = 14;
        public static final boolean INVERT_AIM_NEO550 = false;
        public static final CANSparkMax.IdleMode AIM_MOTOR_NEUTRAL_MODE = CANSparkMax.IdleMode.kBrake;

        public static final double AIM_KP = 0.1;
        public static final double AIM_KI = 0.0;
        public static final double AIM_KD = 0.0;
        public static final double AIM_KF = 0.0;
        public static final double MIN_AIM_CLOSED_LOOP_OUTPUT = -0.8;
        public static final double MAX_AIM_CLOSED_LOOP_OUTPUT =  0.8;
    
        public static final double AIM_POSITION_ZERO_POS =   0.0;
        public static final double AIM_POSITION_NEAR_SHOT =  0.0;       // Hard stop for manual zero set
        public static final double AIM_POSITION_FAR_SHOT  = 35.0;       // Limit 40.0
        public static final double AIM_GEAR_RATIO         = 100.0;

        public static final double  MIN_AIM_POSITION = 0.0;
        public static final double  MAX_AIM_POSITION = 36.0;       // Actual limit ?
        public static final boolean AIM_MOTOR_ENABLE_SOFT_LIMITS = true;

        public static final int  AIM_SMART_CURRENT_LIMIT     = 20;
        public static final double AIM_DECTECTION_CURRENT_FOR_STOP = 18.0;

        public static final double ALLOWED_SHOOTER_AIM_ERROR = 2.0/360.0;           // allow 2 deg error    
    }

    /***********************************************************
     * Climb and Elevator Constants
     ***********************************************************/
    public static final class CC {             // Climber constants
        public static final double SAFETY_THRESHOLD_TIME_BEFORE_MATCH_END = 40;     // 25 seconds
        
        public static final int CLIMB_FALCON_ID = 20;
        public static final NeutralModeValue CLIMB_MOTOR_NEUTRAL_MODE = NeutralModeValue.Brake;
        public static final InvertedValue CLIMB_MOTOR_INVERT = InvertedValue.Clockwise_Positive;
        public static final double CLIMB_OPEN_LOOP_RAMP_PERIOD = 0.0;
        public static final double CLIMB_CLOSED_LOOP_RAMP_PERIOD = 0.0;

        // Climber gear ratio
        public static double CLIMB_GEAR_RATIO = 30;

        public static final int ELEVATOR_NEO550_ID = 21;
        public static final boolean INVERT_ELEVATOR_NEO550 = false;
        // Elevator gear ratio
        public static double ELEVATOT_GEAR_RATIO = 10;

        public static final double  CLIMB_SUPPLY_CURRENT_LIMIT          = 50.0;
        public static final double  CLIMB_SUPPLY_CURRENT_THRESHOLD      = 90.0;
        public static final double  CLIMB_SUPPLY_CURRENT_TIME_THRESHOLD =  0.1;
        public static final boolean CLIMB_ENABLE_SUPPLY_CURRENT_LIMIT   = true;
        public static final double  CLIMB_STATOR_CURRENT_LIMIT          = 50.0;
        public static final boolean CLIMB_ENABLE_STATOR_CURRENT_LIMIT   = true;

        public static final double CLIMB_OUTPUT_LIMIT_FACTOR = 1.0;

        public static final double ELEVATOR_MIN_POS  = -2.0;            // only for legacy note
        public static final double ELEVATOR_MAX_POS = 122.0;            // current safety thresholds used instead

        public static final int ELEVATOR_SMART_CURRENT_LIMIT     = 16;
        public static final int ELEVATOR_SECONDARY_CURRENT_LIMIT = 20;

        public static final double CLIMBER_DUTY_CYCLE  = 1.0;
        public static final double ELEVATOR_DUTY_CYCLE = 0.32;

        public static final int    ELEVATOR_SAFETY_THRESHOLD_CURRENT_LIMIT  = 15;
        public static final long   ELEVATOR_INRUSH_LOCKOUT_TIME             = 350;      // ms
        public static final long   CLIMB_WINCH_INRUSH_LOCKOUT_TIME          = 400;            // ms
        public static final double WINCH_SAFETY_THRESHOLD_CURRENT_LIMIT     = 40.0;
    }
}

