package frc.lib.swerve;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

// Helper Class for the Constants.SDC SubClass
// Takes values and required settings that are constant for a given SDS
// swerve module type, but which may differ from other SDS swerve module types, 
// and stores them as a set of constants (with generic symbolics) so that the 
// symbolics used for writing drivetrain code can be universal, regardless 
// of SDS module type being used.
public class SDS_SwerveUnitParams {
    public final double WHEEL_DIAMETER_M;
    public final double WHEEL_CIRCUMFERENCE_M;
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