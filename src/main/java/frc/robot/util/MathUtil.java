package frc.robot.util;

public class MathUtil {
    /**
	 * Normalize value within a given range
	 * @param min Minimum value in input range
	 * @param max Maximum value in input range
	 * @param a Minimum value in output range
	 * @param b Maximum value in output range
	 * @param x Value to be normalized
	 * @return
	 */
    public static double normalize(double min, double max, double a, double b, double x) {
		return (((b-a) * (x-min)) / (max-min)) + a;
	}
}