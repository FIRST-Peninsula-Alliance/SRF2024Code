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

        public static final int MASTER_ARM_ENCODER_ID = 1;
        public static final double MASTER_ARM_ENCODER_MAGNET_OFFSET = -0.2012;        // was -.0351;     // was -0.037;
        public static final AbsoluteSensorRangeValue MASTER_ARM_CANCODER_RANGE = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
        public static final SensorDirectionValue MASTER_ARM_CANCODER_DIR = SensorDirectionValue.CounterClockwise_Positive;

        // Setup for fused remote sensor, 1:1 on output shaft (possible with licensed Phoenix6 Pro)
        public static final double MASTER_ARM_ENCODER_TO_AXLE_RATIO = 1.0;
        public static final double MASTER_ARM_PLANETARY_GEARBOX_RATIO = 30;
        public static final double MASTER_ARM_CHAIN_DRIVE_GEAR_RATIO = 30.0 / 12.0;
        public static final double MASTER_ARM_ROTOR_TO_ENCODER_RATIO = MASTER_ARM_PLANETARY_GEARBOX_RATIO *
                                                                       MASTER_ARM_CHAIN_DRIVE_GEAR_RATIO;
        public static final int MASTER_ARM_CONT_CURRENT_LIMIT = 40;
        public static final int MASTER_ARM_PEAK_CURRENT_LIMIT = 90;
        public static final double MASTER_ARM_PEAK_CURRENT_DURATION = 0.1;
        public static final boolean MASTER_ARM_ENABLE_CURRENT_LIMIT = true;

        public static final double MASTER_ARM_OPEN_LOOP_RAMP_PERIOD = 0.25;
        public static final double MASTER_ARM_CLOSED_LOOP_RAMP_PERIOD = 0.0;

        //Master Arm Motor PID Values
        public static final double MASTER_ARM_KP = 6.0;
        public static final double MASTER_ARM_KI = 0.0;
        public static final double MASTER_ARM_KD = 1.0;
        public static final double MASTER_ARM_KF = 0.0;
        // Divide SYSID values by 12 to convert from volts to percent output for CTRE
        public static final double MASTER_ARM_KS = 0.25; 
        public static final double MASTER_ARM_KV = 8.0;
        public static final double MASTER_ARM_KA = 0.6;
        public static final double MASTER_ARM_KG = 0.35;
        public static final double MASTER_ARM_MOTION_MAGIC_ACCEL = 8.0;
        public static final double MASTER_ARM_MOTION_MAGIC_VEL = 4.0;
        public static final double MASTER_ARM_MOTION_MAGIC_kA = 0;
        public static final double MASTER_ARM_MOTION_MAGIC_kV = 0;
        public static final double MASTER_ARM_MOTION_MAGIC_JERK = 30;

        public static final double MIN_MASTER_ARM_CLOSED_LOOP_OUTPUT = -1.0;
        public static final double MAX_MASTER_ARM_CLOSED_LOOP_OUTPUT = 1.0;
        public static final double MASTER_ARM_OUTPUT_LIMIT_FACTOR = 0.8;

        // Position units are rotations
        public static final double DISTANT_SPEAKER_SHOT_POS = -0.2296;
        public static final double INDEXED_SPEAKER_SHOT_POS = -.230;                      // was -0.2406;
        public static final double LOW_SAFE_TO_ROTATE_IN_POS = -.2126;
        public static final double LOW_SAFE_TO_ROTATE_OUT_POS = -0.165;             // was -.1686;
        public static final double NOTE_PICKUP_POS = -.155;             // was -.14 // was -0.1476;
        public static final double HIGH_SAFE_TO_ROTATE_DOWN_AND_OUT_POS = .19;      // was 0.0974;
        public static final double AMP_SHOT_POS  = 0.24;                            // was 0.2374;
        public static final double MASTER_ARM_HORIZ_POS = 0.0;

        public static final double ALLOWED_MASTER_ARM_POS_ERROR  = 1.5 / 360;
        public static final double ALLOWED_MILLIS_PER_MOVE = 500;

        public static final double MAX_MASTER_ARM_ROTATIONS = 0.28;
        public static final double MIN_MASTER_ARM_ROTATIONS = -0.25;

        public static final int MASTER_ARM_DATA_COL = 0;
        public static final int MASTER_ARM_DATA_ROW = 0;
        public static final int MASTER_ARM_DATA_LIST_HGT = 8;

        public static final int MASTER_ARM_MOVEMENT_TIMEOUT = 800;   // ms
    }

    /*************************************************************
     * Inner Arm Constants
     *************************************************************/
    public static final class IAC {             // Inner arm constants
        public static final int INNER_ARM_FALCON_ID = 11;
        public static final NeutralModeValue INNER_ARM_MOTOR_NEUTRAL_MODE = NeutralModeValue.Brake;
        public static final InvertedValue INNER_ARM_MOTOR_INVERT = InvertedValue.Clockwise_Positive;
        
        public static final int INNER_ARM_CANCODER_ID = 2;
        public static final double INNER_ARM_CANCODER_MAGNET_OFFSET = .5783;  // was .1713867;  // was.28955;      //was 0.2853;       // rotations
        public static final AbsoluteSensorRangeValue INNER_ARM_CANCODER_RANGE = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
        public static final SensorDirectionValue INNER_ARM_CANCODER_DIR = SensorDirectionValue.CounterClockwise_Positive;

        public static final double INNER_ARM_CANCODER_TO_AXLE_RATIO = 20 / 18;      // Chain drive sprockets
        public static final double INNER_ARM_ROTOR_TO_CANCODER_RATIO = 30;          // Panetary gear box
        public static final int INNER_ARM_CONT_CURRENT_LIMIT = 30;
        public static final int INNER_ARM_PEAK_CURRENT_LIMIT = 80;
        public static final double INNER_ARM_PEAK_CURRENT_DURATION = 0.1;
        public static final boolean INNER_ARM_ENABLE_CURRENT_LIMIT = true;

        public static final double INNER_ARM_OPEN_LOOP_RAMP_PERIOD = 0.25;
        public static final double INNER_ARM_CLOSED_LOOP_RAMP_PERIOD = 0.0;

        //INNER Arm Motor PID Values
        public static final double INNER_ARM_KP = 4.0;
        public static final double INNER_ARM_KI = 0.0;
        public static final double INNER_ARM_KD = 0.5;
        public static final double INNER_ARM_KF = 0.0;
        // Divide SYSID values by 12 to convert from volts to percent output for CTRE
        public static final double INNER_ARM_KS = .3;
        public static final double INNER_ARM_KV = 4.0;
        public static final double INNER_ARM_KA = .4;
        public static final double INNER_ARM_KG = 0.1;
        public static final double INNER_ARM_MOTION_MAGIC_ACCEL = 6.0;
        public static final double INNER_ARM_MOTION_MAGIC_VEL = 3.25;
        public static final double INNER_ARM_MOTION_MAGIC_kA = 0;
        public static final double INNER_ARM_MOTION_MAGIC_kV = 0;
        public static final double INNER_ARM_MOTION_MAGIC_JERK = 25;

        public static final double MIN_INNER_ARM_CLOSED_LOOP_OUTPUT = -1.0;
        public static final double MAX_INNER_ARM_CLOSED_LOOP_OUTPUT = 1.0;
        public static final double INNER_ARM_OUTPUT_LIMIT_FACTOR = 0.8;

        public static final double DISTANT_SPEAKER_GOAL_POS = -0.344;
        public static final double INDEXED_SPEAKER_GOAL_POS = -.35;         // was -0.336;
        // VERTICAL for the inner arm means intake fingers pointing up,
        // and rotation up from the Zero position of fingers horizontal
        // pointing forward is reported as a negative rotation by the
        // inner arm absolute CANcoder.
        public static final double VERTICAL_POS             = -0.25;
        public static final double BUMPER_CONTACT_POS       =  0.10;
        public static final double NOTE_PICKUP_POS          =  0.099999;     // was 0.145;       public static final double HORIZONTAL_FORWARD_POS   =  0.0;
        public static final double HORIZONTAL_BACK_POS      =  -.5;
        public static final double AMP_GOAL_POS             = 0.02;    // was -.9;

        public static final double ALLOWED_INNER_ARM_POS_ERROR  = 1.5 / 360;
        // public static final double ALLOWED_MILLIS_PER_MOVE = 400;
        // For testing only, make the timeout per move 2 seconds instead of 400 ms.
        // This should allow checking the impact of actual setpoint arrival timing,
        // if slower than the prior timeouts. Also add to code a printout statement
        // whenever a timeout (vs actual setpoint arrival) causes a state 
        // or sub-state transition.
        public static final double ALLOWED_MILLIS_PER_MOVE = 2000;
        // Always allow additional time for a move to vertical ("safe" position)
        public static final double ALLOWED_MILLIS_FOR_MOVE_TO_VERTICAL = 1000;


        // to ensure inner arm always rotates in desired direction,
        // do not enable continuoue mode.
        public static final double MAX_INNER_ARM_LIMIT = 0.25;
        public static final double MIN_INNER_ARM_LIMIT = -1.10;
 
        public static final int INNER_ARM_DATA_COL = 2;
        public static final int INNER_ARM_DATA_ROW = 0;
        public static final int INNER_ARM_DATA_LIST_HGT = 8;

        public static final int INNER_ARM_MOVEMENT_TIMEOUT = 400;   // ms
    }

/********************************************************************
 * Intake Constants
 ********************************************************************/
    public static final class IC {              // "Intake Constants"
        public static final int INTAKE_FALCON_MOTOR_ID = 12;
        public static final int INTAKE_SENSOR_PORT_ID = 0;
        public static final boolean INVERT_INNER_ARM_FALCON = false;

        public static final int INTAKE_CONT_CURRENT_LIMIT = 25;
        public static final int INTAKE_PEAK_CURRENT_LIMIT = 50;
        public static final double INTAKE_PEAK_CURRENT_DURATION = 0.1;
        public static final boolean INTAKE_ENABLE_CURRENT_LIMIT = true;

        public static final double INTAKE_OPEN_LOOP_RAMP_PERIOD = 0.2;
        public static final double INTAKE_CLOSED_LOOP_RAMP_PERIOD = 0.0;

        // Intake Motor PID Values - not used in this program, but here for general Config
        public static final double INTAKE_KP = 0.01;
        public static final double INTAKE_KI = 0.0;
        public static final double INTAKE_KD = 0.0;
        public static final double INTAKE_KS = 0.0;
        public static final double INTAKE_KV = 0.0;
        public static final double INTAKE_KA = 0.0;
        public static final double INTAKE_KG = 0.0;

        // With a total intake gear ratio of 13.33,
        // max output shaft rotation velocity is ~ 480 RPM, or 8 RPS. 
        // With roller diameter of ~ 1.5", max belt speed is ~ 37 in/sec. With a Note diameter of 14",
        // and needing to pull in slightly more then 1/2 of that for retention, if we want acqusition in
        // under 1/2 second, it seems reasonable to shoot for a working belt speed of 18 to 24 in/sec.
        public static final double INTAKE_GEAR_RATIO = 10 * (24/18);
        public static final NeutralModeValue INTAKE_MOTOR_NEUTRAL_MODE = NeutralModeValue.Coast;
        public static final InvertedValue INTAKE_MOTOR_INVERT = InvertedValue.CounterClockwise_Positive;
        public static final double INTAKE_CONTROLLER_OUTPUT_LIMIT = 1.0;
    
        public static final double INTAKE_BASE_SPEED = 0.9;         // Runs in DutyCycleOut mode

        // The following are percent output modifiers. They also serve to identify the purpose
        // of operation of active intake belts
        public static final double HOLD_NOTE = .25;
        public static final double ACQUIRE_NOTE = 1.0;
        public static final double EJECT_NOTE = -1.0;
    }

    /********************************************************************
     * Shooter Constants
     ********************************************************************/

    public static final class SC {             // Shooter constants
        public static final int SHOOTER_FALCON_ID = 13;     // Runs in VoltageOut mode

        public static final NeutralModeValue SHOOTER_MOTOR_NEUTRAL_MODE = NeutralModeValue.Coast;
        public static final InvertedValue SHOOTER_MOTOR_INVERT = InvertedValue.CounterClockwise_Positive;
        public static final double SHOOTER_CONTROLLER_OUTPUT_LIMIT = 1.0;
        
        public static final int SHOOTER_CONT_CURRENT_LIMIT = 40;
        public static final int SHOOTER_PEAK_CURRENT_LIMIT = 100;
        public static final double SHOOTER_PEAK_CURRENT_DURATION = 0.1;
        public static final boolean SHOOTER_ENABLE_CURRENT_LIMIT = true;

        public static final double SHOOTER_OPEN_LOOP_RAMP_PERIOD = 0.2;
        
        public static final double SHOOTER_VOLTAGE_OUT_NEAR = 10.5;       // volts, use 9.0 for double wheel shooter
        public static final double SHOOTER_VOLTAGE_OUT_FAR = 11.5;
        public static final double SHOOTER_VELOCITY_NEAR =605;          // Estimated - needs measurment
        public static final double SHOOTER_VELOCITY_FAR = 75;           // ditto
        public static final double MIN_SHOOTER_SPEED = 55;
        public static final double AMP_THRESHOLD_FOR_NOTE_LAUNCH_DETECTION = 40;

        // Shooter Aim Motor parameters

        public static final int AIM_NEO550_ID = 14;
        public static final boolean INVERT_AIM_NEO550 = false;
        public static final CANSparkMax.IdleMode AIM_MOTOR_NEUTRAL_MODE = CANSparkMax.IdleMode.kBrake;

        public static final double AIM_KP = 5.0;
        public static final double AIM_KI = 0.0;
        public static final double AIM_KD = 0.0;
        public static final double AIM_KF = 0.0;
        public static final double MIN_AIM_CLOSED_LOOP_OUTPUT = -0.8;
        public static final double MAX_AIM_CLOSED_LOOP_OUTPUT = 0.8;
    
        public static final double AIM_POSITION_NEAR_SHOT = 1.0;       // Actual is -1.619
        public static final double AIM_POSITION_FAR_SHOT = 29.0;       // Actual is 0.0, hope this eases the bang
        public static final double AIM_GEAR_RATIO = 100.0;       // Actual is 0.0, hope this eases the bang

        public static final int AIM_CONT_CURRENT_LIMIT = 20;
        public static final int AIM_PEAK_CURRENT_LIMIT = 40;
        public static final double AIM_PEAK_CURRENT_DURATION = 0.1;
        public static final boolean AIM_ENABLE_CURRENT_LIMIT = true;

        public static final double ALLOWED_SHOOTER_AIM_ERROR = 5./360.0;           // allow 5 % error    
    }
}

