package frc.robot.commands;

import frc.robot.Constants.*;
import frc.robot.subsystems.*;

import java.util.function.DoubleSupplier;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.Command;

public class DefaultDriveCmd extends Command {    
    private SwerveSubsystem m_swerveDrive;    
    private DoubleSupplier m_translationSup;
    private DoubleSupplier m_strafeSup;
    private DoubleSupplier m_rotationSup;

    public DefaultDriveCmd( SwerveSubsystem swerveDriveSubsys, 
                            DoubleSupplier translationSup, 
                            DoubleSupplier strafeSup,       // ignore
                            DoubleSupplier rotationSup) {
        m_swerveDrive = swerveDriveSubsys;
        addRequirements(swerveDriveSubsys);

        m_translationSup = translationSup;
        m_strafeSup = strafeSup;
        m_rotationSup = rotationSup;
    }

    @Override
    public void execute() {
        if (RobotState.isTest() || (! RobotState.isEnabled())) {
            return;
        } else {
            /* Get Values, Deadband*/
            double translationVal = MathUtil.applyDeadband(m_translationSup.getAsDouble(), UIC.JOYSTICK_DEADBAND);
            double strafeVal = MathUtil.applyDeadband(m_strafeSup.getAsDouble(), UIC.JOYSTICK_DEADBAND);
            double rotationVal = MathUtil.applyDeadband(m_rotationSup.getAsDouble(), UIC.JOYSTICK_DEADBAND);

            // Drive
            m_swerveDrive.drive(new Translation2d(translationVal, strafeVal)
                                    .times(SDC.MAX_ROBOT_SPEED_M_PER_SEC), 
                                    rotationVal * SDC.MAX_ROBOT_ANG_VEL_RAD_PER_SEC, 
                                    true);
        } 
    }
}