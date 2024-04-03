package frc.robot.commands;

import frc.robot.Constants.*;
import frc.robot.subsystems.*;

import java.util.function.DoubleSupplier;
import edu.wpi.first.math.MathUtil;
//  import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;

public class DefaultDriveCmd extends Command {    
    private SwerveSubsystem m_swerveDrive;    
    private DoubleSupplier m_translationSup;
    private DoubleSupplier m_strafeSup;
    private DoubleSupplier m_rotationSup;
    // private SlewRateLimiter m_translateSRLimiter;
    // private SlewRateLimiter m_strafeSRLimiter;
    // private SlewRateLimiter m_rotateSRLimiter;
    private double m_translateVal;
    private double m_strafeVal;
    private double m_rotateVal;

    public DefaultDriveCmd( SwerveSubsystem swerveDriveSubsys, 
                            DoubleSupplier translationSup, 
                            DoubleSupplier strafeSup,
                            DoubleSupplier rotationSup) {
        m_translationSup = translationSup;
        m_strafeSup = strafeSup;
        m_rotationSup = rotationSup;
        m_swerveDrive = swerveDriveSubsys;
        addRequirements(swerveDriveSubsys);

        // m_translateSRLimiter = new SlewRateLimiter(0.5);
        //  m_strafeSRLimiter = new SlewRateLimiter(0.5);
        // m_rotateSRLimiter = new SlewRateLimiter(0.5);
    }

    @Override
    public void execute() {
        // Get Values, apply Deadband
        m_translateVal = MathUtil.applyDeadband(m_translationSup.getAsDouble(), UIC.JOYSTICK_DEADBAND);
        m_strafeVal = MathUtil.applyDeadband(m_strafeSup.getAsDouble(), UIC.JOYSTICK_DEADBAND);
        m_rotateVal = MathUtil.applyDeadband(m_rotationSup.getAsDouble(), UIC.JOYSTICK_DEADBAND);

        // Apply slewRateLimiters
        // m_translateVal = m_translateSRLimiter.calculate(m_translateVal);
        // m_strafeVal = m_strafeSRLimiter.calculate(m_strafeVal);
        // m_rotateVal = m_rotateSRLimiter.calculate(m_rotateVal);

        // Drive
        m_swerveDrive.drive(new Translation2d(m_translateVal, m_strafeVal)
                                .times(SDC.MAX_ROBOT_SPEED_M_PER_SEC), 
                                m_rotateVal * SDC.MAX_ROBOT_ANG_VEL_RAD_PER_SEC, 
                                true);
    }
}