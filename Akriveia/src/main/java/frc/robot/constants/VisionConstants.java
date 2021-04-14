package frc.robot.constants;

public class VisionConstants {
    public static final double VISION_CONSTANT = 269;
    public static final double FEET2METER = 1 / 3.28084;
    public static final double DELTA_X_REAL = (39.25 / 12) * FEET2METER;
    public static final double DEG2RAD = Math.PI / 180;
    public static final double FLYWHEEL_RADIUS = (2 / 12) * FEET2METER;
    public static final double RPM2MPS = (VisionConstants.FLYWHEEL_RADIUS * ((2 * Math.PI) / 60));
}
