package frc.lib.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;

public class SwerveModuleConstants {
    public final int DRIVE_MOTOR_ID;
    public final int STEER_MOTOR_ID;
    public final int ENCODER_ID;
    public final Rotation2d ABS_ANG_OFFSET2D;
    public final ShuffleboardLayout SBE_LAYOUT;

    /**
     * Swerve Module Constants to be used when creating swerve modules.
     * Setup variables in, per module, stored as module running constants.
     * @param driveMotorId
     * @param steerMotorId
     * @param encoderId
     * @param absEncoderOffset2d
     * @param sBE_Layout           (ShuffleBoard Layout handle - to hold SB entry keys)
     */
    public SwerveModuleConstants(int driveMotorID, 
                                 int steerMotorID, 
                                 int canCoderID, 
                                 Rotation2d absEncoderOffset2d,
                                 ShuffleboardLayout sBE_Layout) {
        DRIVE_MOTOR_ID = driveMotorID;
        STEER_MOTOR_ID = steerMotorID;
        ENCODER_ID = canCoderID;
        ABS_ANG_OFFSET2D = absEncoderOffset2d;
        SBE_LAYOUT = sBE_Layout;
    }
}
