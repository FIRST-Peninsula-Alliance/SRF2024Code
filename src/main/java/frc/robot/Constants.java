package frc.robot;

import java.text.DecimalFormat;
import java.util.Map;

import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.CANSparkMax;

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

        public static final double BILLET_WHEEL_DIA_INCHES = 4.0;
        public static final double COLSON_WHEEL_DIA_INCHES = 3.96;
        public static final double PARADE_WHEEL_DIA_INCHES = 3.75;

        // The 2024 chassis (Tsunami) measures 28" x 30", and uses Mk4i-L2 modules. 
        // The wheel to wheel disances for Tsunami are: Width = 22.75", and length = 24.75" 
        // (wheels are inset just 2-5/8" from frame sides).
        // The 2023 chassis (BlackKnight) measures 26" x 30" and uses Mk4-L2 modules. 
        // The wheel to wheel distances for BlackKnight are: Width = 19.5", and length = 23.5"
        // (wheels are inset 3.25" from frame sides).
        public static final double BLACK_KNIGHT_TRACK_WIDTH = Units.inchesToMeters(19.5); 
        public static final double BLACK_KNIGHT_WHEEL_BASE = Units.inchesToMeters(23.5); 
        public static final double HALF_BKTW = BLACK_KNIGHT_TRACK_WIDTH / 2.0;
        public static final double HALF_BKWB = BLACK_KNIGHT_WHEEL_BASE / 2.0;

        public static final double TSUNAMI_TRACK_WIDTH = Units.inchesToMeters(22.75); 
        public static final double TSUNAMI_WHEEL_BASE = Units.inchesToMeters(24.75); 
        public static final double HALF_TTW = TSUNAMI_TRACK_WIDTH / 2.0;
        public static final double HALF_TWB = TSUNAMI_WHEEL_BASE / 2.0;

        public static final Translation2d BK_FL = new Translation2d(HALF_BKWB, HALF_BKTW);                                    
        public static final Translation2d BK_FR = new Translation2d(HALF_BKWB, -HALF_BKTW);
        public static final Translation2d BK_BL = new Translation2d(-HALF_BKWB, HALF_BKTW);
        public static final Translation2d BK_BR = new Translation2d(-HALF_BKWB, -HALF_BKTW);

        public static final Translation2d T_FL = new Translation2d(HALF_TWB, HALF_TTW);                                    
        public static final Translation2d T_FR = new Translation2d(HALF_TWB, -HALF_TTW);
        public static final Translation2d T_BL = new Translation2d(-HALF_TWB, HALF_TTW);
        public static final Translation2d T_BR = new Translation2d(-HALF_TWB, -HALF_TTW);

        // Offsets for changing the center of rotation, if needed, are the same as the
        // Translation2d coordinates of each corner, plus the center of the robot.
        // See SwerveSubsystem. Corner rotations are especially useful for blocking 
        // while playing defense, and sometimes even for evasion while on offense, but 
        // requires and ties up a significant number of GameController button resources
        // (min 4 buttons, or 5 if not using temporary "active only when held" buttons) 
        // so doing this may not be practical for a given season.
        public static final Translation2d   REL_POS2D_CEN = new Translation2d(0, 0);

        // Wheel angles for Back KNight to "park" - makes it difficult to move, slide, or be pushed
        public static final double BK_PARK_ANGLE_LEFT_DEG = 50.2;
        public static final double BK_PARK_ANGLE_RIGHT_DEG = 129.8;

        // Wheel angles for Tsunami "park"
        public static final double T_PARK_ANGLE_LEFT_DEG = 45.0;
        public static final double T_PARK_ANGLE_RIGHT_DEG = -45.0;
        
        // Differentiate SDS module types MK4 and MK4i
        // which are used with the Black Knight and Tsunami chassis, respectively
        // 
        // Swerve Drive Specialties: MK4-L2 Module on Black Knight
        public static SDS_SwerveUnitParams SDSMK4_BLACK_KNIGHT(double wheelDiaInches){
            double wheelDiaM = Units.inchesToMeters(wheelDiaInches);
            Translation2d relPos2D_FL = BK_FL;
            Translation2d relPos2D_FR = BK_FR;
            Translation2d relPos2D_BL = BK_BL;
            Translation2d relPos2D_BR = BK_BR;
            // TODO: measure and enter BlackKnight absolute wheel angle offsets, in degrees, here
            Rotation2d absOffsetFL = Rotation2d.fromDegrees(250.8);
            Rotation2d absOffsetFR = Rotation2d.fromDegrees(252.4);
            Rotation2d absOffsetBL = Rotation2d.fromDegrees(305.3);
            Rotation2d absOffsetBR = Rotation2d.fromDegrees(103.0);
            double parkAngleLeftDeg = BK_PARK_ANGLE_LEFT_DEG;
            double parkAngleRightDeg = BK_PARK_ANGLE_RIGHT_DEG;
            double steerGearRatio = (12.8 / 1.0);
            // L2 Drive Gear Ratio for both the MK4 and MK4i module is 6.75 
            // and SRF uses only the L2 ratio.
            double driveL2GearRatio = (6.75 / 1.0);
            double steerKP = 0.005;         // Was .2 for angles in radians
            double steerKI = 0.0;
            double steerKD = 0.0;
            double steerKF = 0.0;
            InvertedValue driveMotorInvert = InvertedValue.Clockwise_Positive;
            boolean steerMotorInvert = false;
            SensorDirectionValue canCoderDir = SensorDirectionValue.CounterClockwise_Positive;
            return new SDS_SwerveUnitParams(wheelDiaM,
                                            relPos2D_FL,
                                            relPos2D_FR,
                                            relPos2D_BL,
                                            relPos2D_BR,
                                            absOffsetFL, 
                                            absOffsetFR, 
                                            absOffsetBL, 
                                            absOffsetBR, 
                                            parkAngleLeftDeg,
                                            parkAngleRightDeg,
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
        public static SDS_SwerveUnitParams SDSMK4i_TSUNAMI(double wheelDiaInches){
            double wheelDiaM = Units.inchesToMeters(wheelDiaInches);
            Translation2d relPos2D_FL = T_FL;
            Translation2d relPos2D_FR = T_FR;
            Translation2d relPos2D_BL = T_BL;
            Translation2d relPos2D_BR = T_BR;
            // TODO: measure and enter Tsunami absolute wheel angle offsets, in degrees, here
            Rotation2d absOffsetFL = Rotation2d.fromDegrees(58.5);
            Rotation2d absOffsetFR = Rotation2d.fromDegrees(335.41);
            Rotation2d absOffsetBL = Rotation2d.fromDegrees(261.7);
            Rotation2d absOffsetBR = Rotation2d.fromDegrees(35.58);
            double parkAngleLeftDeg = T_PARK_ANGLE_LEFT_DEG;
            double parkAngleRightDeg = T_PARK_ANGLE_RIGHT_DEG;
            double steerGearRatio = ((150.0 / 7.0) / 1.0);
            // L2 Drive Gear Ratio for both the MK4 and MK4i module is 6.75 
            // and SRF only uses the L2 ratio.
            double driveL2GearRatio = (6.75 / 1.0);
            double steerKP = 0.008;             // was .3 for angles in radians
            double steerKI = 0.0;
            double steerKD = 0.0;
            double steerKF = 0.0;
            InvertedValue driveMotorInvert = InvertedValue.CounterClockwise_Positive;   // Tsunami is inverted from Black Knight
            boolean steerMotorInvert = true;
            SensorDirectionValue canCoderDir = SensorDirectionValue.CounterClockwise_Positive;
            
            return new SDS_SwerveUnitParams(wheelDiaM,
                                            relPos2D_FL,
                                            relPos2D_FR,
                                            relPos2D_BL,
                                            relPos2D_BR,
                                            absOffsetFL,
                                            absOffsetFR,
                                            absOffsetBL,
                                            absOffsetBR,
                                            parkAngleLeftDeg,
                                            parkAngleRightDeg,
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
        
        // Set CHOOSEN_MODULE to either the SDSMK4 or SDSMK4I static method
         public static final SDS_SwerveUnitParams CHOOSEN_MODULE =  
                                                    // Uncomment only one module type,
                                                    // and pass the installed wheel
                                                    // diameter as an argument
                                                    // SDSMK4_BLACK_KNIGHT(PARADE_WHEEL_DIA_INCHES);
                                                    SDSMK4i_TSUNAMI(COLSON_WHEEL_DIA_INCHES);

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
        public static final Translation2d REL_POS2D_FL = CHOOSEN_MODULE.REL_POS2D_FL;
        public static final Translation2d REL_POS2D_FR = CHOOSEN_MODULE.REL_POS2D_FR;
        public static final Translation2d REL_POS2D_BL = CHOOSEN_MODULE.REL_POS2D_BL;
        public static final Translation2d REL_POS2D_BR = CHOOSEN_MODULE.REL_POS2D_BR;
        public static final Rotation2d ABS_OFFSET_FL = CHOOSEN_MODULE.ABS_OFFSET_FL;
        public static final Rotation2d ABS_OFFSET_FR = CHOOSEN_MODULE.ABS_OFFSET_FR;
        public static final Rotation2d ABS_OFFSET_BL = CHOOSEN_MODULE.ABS_OFFSET_BL;
        public static final Rotation2d ABS_OFFSET_BR = CHOOSEN_MODULE.ABS_OFFSET_BR;
        public static final double PARK_ANGLE_LEFT_DEG = CHOOSEN_MODULE.PARK_ANGLE_LEFT_DEG;
        public static final double PARK_ANGLE_RIGHT_DEG = CHOOSEN_MODULE.PARK_ANGLE_LEFT_DEG;
        public static final double DRIVE_GEAR_RATIO = CHOOSEN_MODULE.DRIVE_GEAR_RATIO;
        public static final double STEER_GEAR_RATIO = CHOOSEN_MODULE.STEER_GEAR_RATIO;
        public static final double STEER_KP = CHOOSEN_MODULE.STEER_KP;
        public static final double STEER_KI = CHOOSEN_MODULE.STEER_KI;
        public static final double STEER_KD = CHOOSEN_MODULE.STEER_KD;
        public static final double STEER_KF = CHOOSEN_MODULE.STEER_KF;
        public static final boolean STEER_MOTOR_INVERT = CHOOSEN_MODULE.STEER_MOTOR_INVERT;
        public static final InvertedValue DRIVE_MOTOR_INVERT = CHOOSEN_MODULE.DRIVE_MOTOR_INVERT;
        public static final SensorDirectionValue CANCODER_DIR = CHOOSEN_MODULE.CANCODER_DIR;
        // Set Swerve Kinematics.  The order is always FL, FR, BL, and BR, 
        // referenced to an origin at the center of the robot
        // as evidenced by following the signs of the sequential 
        // Translation2d(x, y) coordinates below:
        public static final SwerveDriveKinematics SWERVE_KINEMATICS = 
                                new SwerveDriveKinematics(REL_POS2D_FL, REL_POS2D_FR, REL_POS2D_BL, REL_POS2D_BR);

        // Swerve Drive Constants which are independent of given modules and chassis:
        public static final AbsoluteSensorRangeValue CANCODER_RANGE = AbsoluteSensorRangeValue.Unsigned_0To1;

        // Unit conversion factors. With Phoenix6, the gear ratios are handled by the
        // motor controllers, so that motor.getPosition() values (assuming sensor source is
        // the embedded motor encoder) track the actual mechanism rotations. Thus the 
        // conversion factor for drive system rotations to meters of travel, and for drive
        // rotational velocity (RPS) to linear velopcity (MPS), is just 
        // the driven wheel circumference.
        public static final double TALONFX_ROT_TO_M_FACTOR = WHEEL_CIRCUMFERENCE_M;
        public static final double M_TO_TALONFX_ROT_FACTOR = 1.0 / WHEEL_CIRCUMFERENCE_M;
        public static final double TALONFX_RPS_TO_MPS_FACTOR = WHEEL_CIRCUMFERENCE_M;
        public static final double MPS_TO_TALONFX_RPS_FACTOR = 1.0 / WHEEL_CIRCUMFERENCE_M;

        // Current Limiting motor protection - same for both module types
        // But different for NEO/SmartMAX:
        public static final int  STEER_SMART_CURRENT_LIMIT     = 25;
        public static final boolean STEER_ENABLE_CURRENT_LIMIT = true;
        // and Falcon/FX:
        public static final double  DRIVE_SUPPLY_CURRENT_LIMIT          = 40.0;
        public static final double  DRIVE_SUPPLY_CURRENT_THRESHOLD      = 0.0;
        public static final double  DRIVE_SUPPLY_CURRENT_TIME_THRESHOLD = 0.1;
        public static final boolean DRIVE_ENABLE_SUPPLY_CURRENT_LIMIT   = true;
        public static final double  DRIVE_STATOR_CURRENT_LIMIT          = 45.0;
        public static final boolean DRIVE_ENABLE_STATOR_CURRENT_LIMIT   = true;
        
        // Voltage compensation
        public static final double STEER_MOTOR_VOLTAGE_COMPENSATION = 12.0;

        // These values are used by the drive motor to ramp in open loop.
        // Team 364 found a small open loop ramp (0.25) helps with tread wear, 
        // avoiding tipping, etc. In closed loop control, it would probably be 
        // better to employ profiled PID controllers.
        public static final double OPEN_LOOP_RAMP_PERIOD = 1.00;
        public static final double CLOSED_LOOP_RAMP_PERIOD = 0.0;

        // Drive Motor PID Values
        public static final double DRIVE_KP = 0.1;
        public static final double DRIVE_KI = 0.0;
        public static final double DRIVE_KD = 0.0;
        public static final double DRIVE_KF = 0.0;

        // Drive Motor Characterization Values
        // Divide SYSID values by 12 to convert from volts to percent output for CTRE
        public static final double DRIVE_KS = (0.32 / 12);
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
        public static final double OUTPUT_DRIVE_LIMIT_FACTOR = 1.00;
        public static final double OUTPUT_ROTATE_LIMIT_FACTOR = 1.00;

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
            public static final int canCoderID   = 1;
            public static final Rotation2d angleOffset = ABS_OFFSET_FL;
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
            public static final int canCoderID   = 2;
            public static final Rotation2d angleOffset = ABS_OFFSET_FR;                             
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
            public static final int canCoderID   = 3;
            public static final Rotation2d angleOffset = ABS_OFFSET_BL;
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
            public static final int canCoderID   = 4;
            public static final Rotation2d angleOffset = ABS_OFFSET_BR;
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
     * Autonomous Constants
     ***********************************************************/
    public static final class AutoC {       // AutoC = short for AutoConstants
        public static final double AUTO_MAX_SPEED_M_PER_SEC = 4.5;       // was 4.5
        public static final double AUTO_MAX_ACCEL_M_PER_SEC2 = 2.0;      // was 3.0
        public static final double AUTO_MAX_ANG_VEL_RAD_PER_SEC = 4.0*Math.PI;
        public static final double AUTO_MAX_ANG_ACCEL_RAD_PER_SEC2 = 1.5*Math.PI;    // was 2.0

        public static final double KP_X_CONTROLLER = 10.0;
        public static final double KI_X_CONTROLLER = 0.5;
        public static final double KP_Y_CONTROLLER = 10.0;
        public static final double KI_Y_CONTROLLER = 0.5;
        public static final double KP_THETA_CONTROLLER = 6.0;
        public static final double KI_THETA_CONTROLLER = 0.5;
    
        /* Constraint for the motion profiled robot angle controller */
        public static final TrapezoidProfile.Constraints K_THETA_CONTROLLER_CONSTRAINTS =
                            new TrapezoidProfile.Constraints(AUTO_MAX_ANG_VEL_RAD_PER_SEC, 
                                                             AUTO_MAX_ANG_ACCEL_RAD_PER_SEC2);

        public static final double AUTO_SPEED_FACTOR_GENERIC = 1.0;
        public static final double AUTO_ACCEL_FACTOR_GENERIC = 1.0;

        // Declare handles for choosable starting positions
        // Normally would use an enum for this, but Java/compiler was not cooperating -
        // need better understanding of enum syntax requirements, apparently.
        // For now, just use string objects. 
        public static final String STARTING_LEFT = "Left";
        public static final String STARTING_CENTER = "Center";
        public static final String STARTING_RIGHT  = "Right";
    }
}
