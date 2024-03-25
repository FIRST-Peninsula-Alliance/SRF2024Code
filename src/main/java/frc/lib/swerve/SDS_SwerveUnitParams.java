package frc.lib.swerve;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

// Helper Class for the Constants.SDC SubClass
// Takes values and required settings that are constant for a given SDS
// swerve module type, but which may differ from other SDS swerve module types, 
// and stores them as a set of constants (with generic symbolics) so that the 
// symbolics used for writing drivetrain code can be universal, regardless 
// of SDS module type being used.
public class SDS_SwerveUnitParams {
    public final double WHEEL_DIAMETER_M;
    public final double WHEEL_CIRCUMFERENCE_M;
    public final Translation2d REL_POS2D_FL;
    public final Translation2d REL_POS2D_FR;
    public final Translation2d REL_POS2D_BL;
    public final Translation2d REL_POS2D_BR;
    public final Rotation2d ABS_OFFSET_FL;
    public final Rotation2d ABS_OFFSET_FR;
    public final Rotation2d ABS_OFFSET_BL;
    public final Rotation2d ABS_OFFSET_BR;
    public final double PARK_ANGLE_LEFT_DEG;
    public final double PARK_ANGLE_RIGHT_DEG;
    public final double STEER_GEAR_RATIO;
    public final double DRIVE_GEAR_RATIO;
    public final double STEER_KP;
    public final double STEER_KI;
    public final double STEER_KD;
    public final double STEER_KF;
    public final InvertedValue DRIVE_MOTOR_INVERT;
    public final boolean STEER_MOTOR_INVERT;
    public final SensorDirectionValue CANCODER_DIR;

    public SDS_SwerveUnitParams(double wheelDiameterM,
                                Translation2d relPos2D_FL,
                                Translation2d relPos2D_FR,
                                Translation2d relPos2D_BL,
                                Translation2d relPos2D_BR,
                                Rotation2d absOffsetFL,
                                Rotation2d absOffsetFR,
                                Rotation2d absOffsetBL,
                                Rotation2d absOffsetBR,
                                double parkAngleLeftDeg, 
                                double parkAngleRightDeg, 
                                double steerGearRatio, 
                                double driveGearRatio, 
                                double steerKP, 
                                double steerKI, 
                                double steerKD,
                                double steerKF, 
                                InvertedValue driveMotorInvert, 
                                boolean steerMotorInvert,
                                SensorDirectionValue canCoderDir) {
        WHEEL_DIAMETER_M = wheelDiameterM;
        WHEEL_CIRCUMFERENCE_M = wheelDiameterM * Math.PI;
        REL_POS2D_FL = relPos2D_FL;
        REL_POS2D_FR = relPos2D_FR;
        REL_POS2D_BL = relPos2D_BL;
        REL_POS2D_BR = relPos2D_BR;
        ABS_OFFSET_FL = absOffsetFL;
        ABS_OFFSET_FR = absOffsetFR;
        ABS_OFFSET_BL = absOffsetBL;
        ABS_OFFSET_BR = absOffsetBR;
        PARK_ANGLE_LEFT_DEG = parkAngleLeftDeg;
        PARK_ANGLE_RIGHT_DEG = parkAngleRightDeg;
        STEER_GEAR_RATIO = steerGearRatio;
        DRIVE_GEAR_RATIO = driveGearRatio;
        STEER_KP = steerKP;
        STEER_KI = steerKI;
        STEER_KD = steerKD;
        STEER_KF = steerKF;
        DRIVE_MOTOR_INVERT = driveMotorInvert;
        STEER_MOTOR_INVERT = steerMotorInvert;
        CANCODER_DIR = canCoderDir;
    }
}  