package frc.robot;

import java.text.DecimalFormat;
import java.util.Map;

import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.lib.swerve.SDS_SwerveUnitParams;
import frc.lib.swerve.SwerveModuleConstants;

public final class Constants {
    /***************************************************
     * Universal Constants
     ***************************************************/
    public static final String CANIVORE_BUS_NAME = "CANivore";
    public static final String ROBO_RIO_BUS_NAME = "Rio";

    public static final class F {
        // Formatters to control number of decimal places 
        // in the various published / recorded data
        public static DecimalFormat df1 = new DecimalFormat("#.#");
        public static DecimalFormat df2 = new DecimalFormat("#.##");
        public static DecimalFormat df3 = new DecimalFormat("#.###");
        public static DecimalFormat df4 = new DecimalFormat("#.####");
        public static DecimalFormat df20 = new DecimalFormat("##");
        public static DecimalFormat df40 = new DecimalFormat("####");
        public static DecimalFormat df80 = new DecimalFormat("########");
    }

    /****************************************************
     * User Interface Constants
     ****************************************************/
    public static final class UIC {             // UIC = short for UserInterfaceConstants
        public static final double JOYSTICK_DEADBAND = 0.18;
    }

    /*****************************************************
     * Gyro Constants
     *****************************************************/
    public static final class GC {              // GC = short for GyroConstants
        public static final int PIGEON_2_CANID = 1;
        public static final boolean INVERT_GYRO = false; // In phoenix6, Pigeon2.getAngle() api
                                                        // returns heading, but it increases with
                                                        // CW rotation, the opposite of WPILib
                                                        // convention, which is CCW+ CW-
                                                        // So it must be inverted.

        // if NavX, use NavX library to init (no need for an ID), but be sure to
        // set INVERT_GYRO = true; // because NavX measures CCW as negative.
    }

    /******************************************************
     * Swerve Drive Constants
     ******************************************************/
    public static final class SDC {             // SDC = short for SwerveDriveConstants
        // Start with differentiating SDS module types MK4 and MK4I
        // Swerve Drive Specialties: MK4-L2 Module
        public static SDS_SwerveUnitParams SDSMK4(double wheelDiaInches){
            double wheelDiaM = Units.inchesToMeters(wheelDiaInches);
            double steerGearRatio = (12.8 / 1.0);
            // L2 Drive Gear Ratio for both the MK4 and MK4i module is 6.75 
            // and SRF only uses the L2 ratio.
            double driveL2GearRatio = (6.75 / 1.0);
            double steerKP = 0.005;         // Was .2 for angles in radians
            double steerKI = 0.0;
            double steerKD = 0.0;
            double steerKF = 0.0;
            InvertedValue driveMotorInvert = InvertedValue.Clockwise_Positive;
            boolean steerMotorInvert = false;
            SensorDirectionValue canCoderDir = SensorDirectionValue.CounterClockwise_Positive;
            return new SDS_SwerveUnitParams(wheelDiaM, 
                                            steerGearRatio, 
                                            driveL2GearRatio, 
                                            steerKP, 
                                            steerKI, 
                                            steerKD, 
                                            steerKF, 
                                            driveMotorInvert, 
                                            steerMotorInvert, 
                                            canCoderDir);
        }

        // Swerve Drive Specialties: MK4I-L2 Module
        public static SDS_SwerveUnitParams SDSMK4i(double wheelDiaInches){
            double wheelDiaM = Units.inchesToMeters(wheelDiaInches);
            double steerGearRatio = ((150.0 / 7.0) / 1.0);
            // L2 Drive Gear Ratio for both the MK4 and MK4i module is 6.75 
            // and SRF only uses the L2 ratio.
            double driveL2GearRatio = (6.75 / 1.0);
            double steerKP = 0.008;             // was .3 for angles in radians
            double steerKI = 0.0;
            double steerKD = 0.0;
            double steerKF = 0.0;
            InvertedValue driveMotorInvert = InvertedValue.CounterClockwise_Positive;       // TODO - Double check this!!!
                                                                                            // Tsunami drives backwards from Black Knight
            boolean steerMotorInvert = true;
            SensorDirectionValue canCoderDir = SensorDirectionValue.CounterClockwise_Positive;
            
            return new SDS_SwerveUnitParams(wheelDiaM, 
                                            steerGearRatio, 
                                            driveL2GearRatio, 
                                            steerKP, 
                                            steerKI, 
                                            steerKD, 
                                            steerKF, 
                                            driveMotorInvert, 
                                            steerMotorInvert, 
                                            canCoderDir);
        }

        public static final double  BILLET_WHEEL_DIA_INCHES = 4.0;
        public static final double  COLSON_WHEEL_DIA_INCHES = 4.0;

        // Set CHOOSEN_MODULE to either the SDSMK4 or SDSMK4I static method
         public static final SDS_SwerveUnitParams CHOOSEN_MODULE =  
                                                    // Uncomment only one module type,
                                                    // and pass the installed wheel
                                                    // diameter as an argument
                                                    // SDSMK4(BILLET_WHEEL_DIA_INCHES);
                                                    SDSMK4i(COLSON_WHEEL_DIA_INCHES);
        // Now use CHOOSEN_MODULE to initialize generically named (and usable)
        // constants but which are specific to a given module type. 
        // Note some constants are actually the same between the only two SDS module
        // types which SRF currently uses, but this approach, originally designed 
        // to support the declaration of any COTS swerve module, was easier to leave 
        // mostly intact, especially since with this revision it should be much easier 
        // to understand for newbie programmers while still teaching a useful 
        // abstraction technique.
        public static final double WHEEL_DIAMETER_M = CHOOSEN_MODULE.WHEEL_DIAMETER_M;
        public static final double WHEEL_CIRCUMFERENCE_M = CHOOSEN_MODULE.WHEEL_CIRCUMFERENCE_M;
        public static final double DRIVE_GEAR_RATIO = CHOOSEN_MODULE.DRIVE_GEAR_RATIO;
        public static final double STEER_GEAR_RATIO = CHOOSEN_MODULE.STEER_GEAR_RATIO;
        public static final double STEER_KP = CHOOSEN_MODULE.STEER_KP;
        public static final double STEER_KI = CHOOSEN_MODULE.STEER_KI;
        public static final double STEER_KD = CHOOSEN_MODULE.STEER_KD;
        public static final double STEER_KF = CHOOSEN_MODULE.STEER_KF;
        public static final boolean STEER_MOTOR_INVERT = CHOOSEN_MODULE.STEER_MOTOR_INVERT;
        public static final InvertedValue DRIVE_MOTOR_INVERT = CHOOSEN_MODULE.DRIVE_MOTOR_INVERT;
        public static final SensorDirectionValue CANCODER_DIR = CHOOSEN_MODULE.CANCODER_DIR;

        public static final boolean STEER_KP_TUNING_ENABLED = false;
        
        public static final AbsoluteSensorRangeValue CANCODER_RANGE = AbsoluteSensorRangeValue.Unsigned_0To1;
 
        // Unit conversion factors
        public static final double NEO_REV_TO_DEG_FACTOR = (360.0 / STEER_GEAR_RATIO);
        public static final double CANCODER_TO_DEG_FACTOR = (360.0 / (STEER_GEAR_RATIO * 4096.0));
        public static final double FALCON_TO_DEG_FACTOR = (360.0 / (DRIVE_GEAR_RATIO * 2048.0));
        public static final double FALCON_VEL_TO_RPM_FACTOR = ((600.0 / 2048.0) / DRIVE_GEAR_RATIO);
        public static final double RPM_TO_FALCON_VEL_FACTOR = ((2048.0 / 600.0) * DRIVE_GEAR_RATIO);
        public static final double FALCON_VEL_TO_MPS_FACTOR = FALCON_VEL_TO_RPM_FACTOR * WHEEL_CIRCUMFERENCE_M / 60;
        public static final double MPS_TO_RPM_FACTOR = (60 / WHEEL_CIRCUMFERENCE_M);
        public static final double MPS_TO_FALCON_VEL_FACTOR = MPS_TO_RPM_FACTOR * RPM_TO_FALCON_VEL_FACTOR;
        public static final double FALCON_TO_M_FACTOR = (WHEEL_CIRCUMFERENCE_M / (DRIVE_GEAR_RATIO * 2048.0));
        public static final double M_TO_FALCON_FACTOR = 1.0 / (WHEEL_CIRCUMFERENCE_M / (DRIVE_GEAR_RATIO * 2048.0));

        // Phoenix6 conversion factors to accomodate Falcon units now being in rotations.
        public static final double FALCON_ROT_TO_M_FACTOR = WHEEL_CIRCUMFERENCE_M / DRIVE_GEAR_RATIO;
        public static final double FALCON_RPS_TO_MPS_FACTOR = WHEEL_CIRCUMFERENCE_M / DRIVE_GEAR_RATIO;
        public static final double MPS_TO_FALCON_RPS_FACTOR = DRIVE_GEAR_RATIO / WHEEL_CIRCUMFERENCE_M;

        // Now set the Drivetrain Constants which are independent of a
        // specific swerve module type. The 2024 chassis measures 28" x 30",
        // and with Mk4i modules, the wheel to wheel disances are Width = 22.75",
        // and length = 24.75" (for Mk4i modules, wheels end up inset 2-5/8" from
        // frame sides).
        public static final double TRACK_WIDTH = Units.inchesToMeters(22.75); // was 19.5 for 2023
        public static final double WHEEL_BASE = Units.inchesToMeters(24.75);    // was 23.5 for 2023

        // Set Swerve Kinematics.  No need to ever change this unless you are not 
        // doing a traditional rectangular/square 4 module swerve. The order is
        // always FL, FR, BL, and BR, referenced to the center of the robot as 
        // origin, as you can see by following the signs of the sequential 
        // Translation2d(x, y) coordinates below:
        public static final SwerveDriveKinematics SWERVE_KINEMATICS = new SwerveDriveKinematics(
                                new Translation2d(WHEEL_BASE / 2.0, TRACK_WIDTH / 2.0),
                                new Translation2d(WHEEL_BASE / 2.0, -TRACK_WIDTH / 2.0),
                                new Translation2d(-WHEEL_BASE / 2.0, TRACK_WIDTH / 2.0),
                                new Translation2d(-WHEEL_BASE / 2.0, -TRACK_WIDTH / 2.0));
        
        // Offsets for changing the center of rotation, if needed. See SwerveSubsystem.
        // This is especially useful for blocking while playing defense, and sometimes
        // even for evasion while on offense, but requires and ties up a significant 
        // number of GameController button resources (4 to 5 buttons!) so it may
        // not be practical for a given season.
        public static final Translation2d   ROTATE_ABOUT_CEN = new Translation2d(0, 0);
        public static final Translation2d   ROTATE_ABOUT_FL = new Translation2d(WHEEL_BASE / 2.0, TRACK_WIDTH / 2.0);
        public static final Translation2d   ROTATE_ABOUT_FR = new Translation2d(WHEEL_BASE / 2.0, -TRACK_WIDTH / 2.0);
        public static final Translation2d   ROTATE_ABOUT_BL = new Translation2d(-WHEEL_BASE / 2.0, TRACK_WIDTH / 2.0);
        public static final Translation2d   ROTATE_ABOUT_BR = new Translation2d(-WHEEL_BASE / 2.0, -TRACK_WIDTH / 2.0);
   
        // Wheel angles for "park" - makes it difficult to move, slide, or be pushed
        public static final double PARK_ANGLE_LEFT_DEG = -45;
        public static final double PARK_ANGLE_RIGHT_DEG = 45;
        
        // Current Limiting motor protection - same for both module types
        public static final int STEER_CONT_CURRENT_LIMIT = 25;
        public static final int STEER_PEAK_CURRENT_LIMIT = 50;
        public static final double STEER_PEAK_CURRENT_DURATION = 0.1;
        public static final boolean STEER_ENABLE_CURRENT_LIMIT = true;

        public static final double DRIVE_CONT_CURRENT_LIMIT = 35.0;
        public static final double DRIVE_PEAK_CURRENT_LIMIT = 65.0;
        public static final double DRIVE_PEAK_CURRENT_DURATION = 0.1;
        public static final boolean DRIVE_ENABLE_CURRENT_LIMIT = true;

        // Voltage compensation not used by CTRE anymore, but is for REV NEOs
        public static final double STEER_MOTOR_VOLTAGE_COMPENSATION = 12.0;

        // These values are used by the drive motor to ramp in open loop.
        // Team 364 found a small open loop ramp (0.25) helps with tread wear, 
        // avoiding tipping, etc. In closed loop control, it would probably be 
        // better to employ profiled PID controllers.
        public static final double OPEN_LOOP_RAMP_PERIOD = 1.25;    // Part of attempt to take smotor stuttering on quick starts
        public static final double CLOSED_LOOP_RAMP_PERIOD = 0.0;

        // Drive Motor PID Values
        public static final double DRIVE_KP = 0.1; //TODO: This must be tuned to specific robot
        public static final double DRIVE_KI = 0.0;
        public static final double DRIVE_KD = 0.0;
        public static final double DRIVE_KF = 0.0;

        // Drive Motor Characterization Values
        // Divide SYSID values by 12 to convert from volts to percent output for CTRE
        public static final double DRIVE_KS = (0.32 / 12); //TODO: This must be tuned to specific robot
        public static final double DRIVE_KV = (1.51 / 12);
        public static final double DRIVE_KA = (0.27 / 12);
        public static final double DRIVE_KG = 0.27;
        
        // Swerve Profiling Values for Robot
        // Best if getten by characterizing the robot, but these values worked
        // tolerably well in 2023 as is. Used during Auto mode moves, which are
        // tracked by Odometry.
        public static final double MAX_ROBOT_SPEED_M_PER_SEC = 4.5; // 4.96 theoretically 
        public static final double MAX_ROBOT_ANG_VEL_RAD_PER_SEC = 11.0; // 11.96 theoretically 

        // Swerve output fixed limit values for teleop control (reduce if
        // speeds are too fast for the experience level of the drive team).
        // (In Auto, tuning should set speeds to reasonable values, no need
        // to reduce them - in fact, just the opposite, want fastest possible
        // movements in Auto mode, consistent with safety).
        public static final double OUTPUT_DRIVE_LIMIT_FACTOR = 0.7;     // Part of attempt to fix motor
        public static final double OUTPUT_ROTATE_LIMIT_FACTOR = 0.7;    // on quick starts

        // When monitored while set at -1 to 1, seemed like the Steering PID output 
        // did not generate percent outputs greater than about .4
        public static final double MIN_STEER_CLOSED_LOOP_OUTPUT = -0.6;
        public static final double MAX_STEER_CLOSED_LOOP_OUTPUT = 0.6;

        /* Default Motor Neutral Modes */
        public static final CANSparkMax.IdleMode STEER_MOTOR_NEUTRAL_MODE = CANSparkMax.IdleMode.kCoast;
        public static final NeutralModeValue DRIVE_MOTOR_NEUTRAL_MODE = NeutralModeValue.Brake;

        // Finally, declare constants to define and allow addressing the (typically 4)
        // individual module components, including the Shuffleboard coordinates
        // embedded in the associated ShuffleboardLayout objects created for each
        // module under the shared "SwerveDrive" Shuffleboard Tab
        public static final int FIRST_SWERVE_MOD_LIST_COL = 5;
        public static final int FIRST_SWERVE_MOD_LIST_ROW = 0;
        public static final int SWERVE_MOD_LIST_HGT = 4;
        public static final ShuffleboardTab sbt = Shuffleboard.getTab("SwerveDrive");
        // Front Left Module - Module 0
        public static final class FL_Mod0 {
            public static final int driveMotorID = 1;
            public static final int steerMotorID = 2;
            public static final int canCoderID = 1;
            // TODO: measure and enter Mod0 absolute wheel angle offset in degrees here
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(61.5);
            public static final ShuffleboardLayout sBE_Layout0 = 
                                    sbt.getLayout("FL_Mod0", BuiltInLayouts.kList)
                                       .withPosition(FIRST_SWERVE_MOD_LIST_COL + 0,
                                                     FIRST_SWERVE_MOD_LIST_ROW)
                                       .withSize(1, SWERVE_MOD_LIST_HGT)
                                       .withProperties(Map.of("Label position", "LEFT"));
            public static final SwerveModuleConstants MODULE_CONSTANTS = 
                                        new SwerveModuleConstants(driveMotorID, 
                                                                  steerMotorID, 
                                                                  canCoderID, 
                                                                  angleOffset,
                                                                  sBE_Layout0);
        }

        // Front Right Module - Module 1
        public static final class FR_Mod1 {
            public static final int driveMotorID = 3;
            public static final int steerMotorID = 4;
            public static final int canCoderID = 2;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(337.41);                             
            public static final ShuffleboardLayout sBE_Layout1 = 
                                     sbt.getLayout("FR_Mod1", BuiltInLayouts.kList)
                                        .withPosition(FIRST_SWERVE_MOD_LIST_COL + 1,
                                                    FIRST_SWERVE_MOD_LIST_ROW)
                                        .withSize(1, SWERVE_MOD_LIST_HGT)
                                        .withProperties(Map.of("Label position", "LEFT"));
            public static final SwerveModuleConstants MODULE_CONSTANTS = 
                                        new SwerveModuleConstants(driveMotorID, 
                                                                  steerMotorID, 
                                                                  canCoderID, 
                                                                  angleOffset,
                                                                  sBE_Layout1);
        }
        
        /* Back Left Module - Module 2 */
        public static final class BL_Mod2 {
            public static final int driveMotorID = 5;
            public static final int steerMotorID = 6;
            public static final int canCoderID = 3;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(261.7);
            public static final ShuffleboardLayout sBE_Layout2 = 
                                    sbt.getLayout("BL_Mod2", BuiltInLayouts.kList)
                                       .withPosition(FIRST_SWERVE_MOD_LIST_COL + 2,
                                                     FIRST_SWERVE_MOD_LIST_ROW)
                                       .withSize(1, SWERVE_MOD_LIST_HGT)
                                       .withProperties(Map.of("Label position", "LEFT"));
            public static final SwerveModuleConstants MODULE_CONSTANTS = 
                                        new SwerveModuleConstants(driveMotorID, 
                                                                  steerMotorID, 
                                                                  canCoderID, 
                                                                  angleOffset,
                                                                  sBE_Layout2);
        }

        /* Back Right Module - Module 3 */
        public static final class BR_Mod3 {
            public static final int driveMotorID = 7;
            public static final int steerMotorID = 8;
            public static final int canCoderID = 4;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(34.58);
            public static final ShuffleboardLayout sBE_Layout3 = 
                                    sbt.getLayout("BR_Mod3", BuiltInLayouts.kList)
                                       .withPosition(FIRST_SWERVE_MOD_LIST_COL + 3,
                                                     FIRST_SWERVE_MOD_LIST_ROW)
                                       .withSize(1, SWERVE_MOD_LIST_HGT)
                                       .withProperties(Map.of("Label position", "LEFT"));
            public static final SwerveModuleConstants MODULE_CONSTANTS = 
                                        new SwerveModuleConstants(driveMotorID, 
                                                                  steerMotorID, 
                                                                  canCoderID, 
                                                                  angleOffset,
                                                                  sBE_Layout3);
        }
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

        public static final int CLIMB_CONT_CURRENT_LIMIT = 50;
        public static final int CLIMB_PEAK_CURRENT_LIMIT = 90;
        public static final double CLIMB_PEAK_CURRENT_DURATION = 0.1;
        public static final boolean CLIMB_ENABLE_CURRENT_LIMIT = true;

        public static final double CLIMB_OUTPUT_LIMIT_FACTOR = 1.0;

        public static final double ELEVATOR_MIN_POS = -2.0;
        public static final double ELEVATOR_MAX_POS = 122.0;

        public static final double CLIMBER_DUTY_CYCLE = 1.0;
        public static final double ELEVATOR_DUTY_CYCLE = 0.32;
        public static final double ELEVATOR_SAFETY_THRESHOLD_CURRENT_LIMIT = 20;
        public static final long ELEVATOR_INRUSH_LOCKOUT_TIME = 350;      // ms
        public static final long CLIMB_WINCH_INRUSH_LOCKOUT_TIME = 400;            // ms
        public static final double WINCH_SAFETY_THRESHOLD_CURRENT_LIMIT = 20;
    }

    /***********************************************************
     * Autonomous Constants
     ***********************************************************/
    public static final class AutoC {       // AutoC = short for AutoConstants
        //TODO: The below constants are used in the example auto, and must be tuned to specific robot
        public static final double AUTO_MAX_SPEED_M_PER_SEC = 2.5;        // was 4.5
        public static final double AUTO_MAX_ACCEL_M_PER_SEC2 = 1.5;      // was 3
        public static final double AUTO_MAX_ANG_VEL_RAD_PER_SEC = 2.5*Math.PI;      // was 4
        public static final double AUTO_MAX_ANG_ACCEL_RAD_PER_SEC2 = 1*Math.PI;    // was 2

        public static final double KP_X_CONTROLLER = 1;        // TODO: was 2
        public static final double KP_Y_CONTROLLER = 1;          // was 1
        public static final double KP_THETA_CONTROLLER = 1.6;        //was 1.6
    
        /* Constraint for the motion profiled robot angle controller */
        public static final TrapezoidProfile.Constraints K_THETA_CONTROLLER_CONSTRAINTS =
                            new TrapezoidProfile.Constraints(AUTO_MAX_ANG_VEL_RAD_PER_SEC, 
                                                             AUTO_MAX_ANG_ACCEL_RAD_PER_SEC2);

        public static final double AUTO_DISTANCE_CORR_FACTOR = 1.055;     // Factor to fine tune actual dist vs. Robot calculated distance
        public static final Pose2d START_POSE2D_FOR_SCORING = new Pose2d(1.0, 0.0, new Rotation2d(0.0));
        public static final Pose2d END_POSE2D_FOR_SCORE_THEN_EXIT = new Pose2d(6.5, 0.0, new Rotation2d(0.0));
        public static final double AUTO_SPEED_FACTOR_FOR_SCORING = 0.5;
        public static final double AUTO_SPEED_FACTOR_GENERIC = 0.4;		// TODO make 0.6
        public static final double AUTO_ACCEL_FACTOR_GENERIC = 0.4;		// and 0.5

        public static final double AUTO_SPEED_FACTOR_FOR_PLATFORM = .4;
        public static final double AUTO_SPEED_FACTOR_FOR_BACKWARDS_PLATFORM = 1.15;
        public static final double AUTO_SPEED_FACTOR_FOR_BALANCE = 0.4;
        public static final double MAX_OUTPUT_FOR_BALANCE = 0.7;

        public static final double BALANCING_DRIVE_KP = 0.005; // P (Proportional) constant of a PID loop
        public static final double BALANCING_GOAL_DEGREES = 0;
        public static final double BALANCING_ANGLE_THRESHOLD_DEGREES = 1.25;
    }
}
