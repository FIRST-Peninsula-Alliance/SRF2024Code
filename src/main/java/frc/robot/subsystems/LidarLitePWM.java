package frc.robot.subsystems;

import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.DigitalSource;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class LidarLitePWM {
	private Counter	 		m_counter;
	private int		 		m_printedWarningCount = 5;
	private	double	 		m_cm;
	private MedianFilter	m_filter;
	private double          m_rawCM;

	/*********************************************************************************
	 * Constructor:
	 * Create a LidarLitePWM object to read the sensor attached to the specified
	 * digital input port on the roboRIO
	 * @param source the DigitalSource where the LIDAR-Lite is attached
	 * (ex: new DigitalInput(0))
	 *********************************************************************************/
	public LidarLitePWM (DigitalSource source) {
		m_counter = new Counter(source);
		m_counter.setMaxPeriod(1.0);
		// Configure for measuring rising to falling pulses
		m_counter.setSemiPeriodMode(true);
		m_counter.reset();
		m_filter = new MedianFilter(5);
	}

	/**
	 * Take a measurement, filter it, and return the filtered distance in cm
	 * 
	 * @return Distance in cm
	 */
	public double getDistance() {
		// If we haven't seen the first rising to falling pulse, then we have no measurement.
		// This happens when there is no LIDAR-Lite plugged in, btw. Only give the warning
		// 5 times to avoid output terminal clutter.
		if (m_counter.get() < 1) {
			if (m_printedWarningCount-- > 0) {
				System.out.println("LidarLitePWM: waiting for distance measurement");
			}
			return 0;
		}
		// getPeriod returns time in seconds, of type double. The hardware resolution 
		// is microseconds.  The LIDAR-Lite unit sends a high signal for 10 microseconds 
		// per cm of distance, so to get cm multiply by (1000000 / 10), or 100000.
		m_rawCM = m_counter.getPeriod() * 100000.0;
		m_cm = m_filter.calculate(m_rawCM);
		//SmartDashboard.putNumber("Lidar Raw Distance ", m_rawCM);
		//SmartDashboard.putNumber("Lidar Filtered Distance ", m_cm);
		return m_cm;
	}
}