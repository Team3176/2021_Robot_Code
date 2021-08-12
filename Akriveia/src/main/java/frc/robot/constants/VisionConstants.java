package frc.robot.constants;

public class VisionConstants {
    public static final double VISION_CONSTANT = 269;
    public static final double FEET2METER = 1 / 3.28084;
    public static final double DELTA_X_REAL = (39.25 / 12) * FEET2METER;
    public static final double DEG2RAD = Math.PI / 180;
    public static final double FLYWHEEL_RADIUS = (2 / 12) * FEET2METER;
    public static final double RPM2MPS = (VisionConstants.FLYWHEEL_RADIUS * ((2 * Math.PI) / 60));
    public static final int VISION_LED_OFF = 1;
    public static final int VISION_LED_ON = 2;
    public static final int VISION_LED_PIPELINE = 0;
    public static final double MIN_TURN_OUTPUT = 0.2;
    public static final double VISION_STEER_KP = 0.3;
    public static final int PIPELINE_FOR_TARGET_RECOG = 1;
    public static final int PIPELINE_FOR_DRIVER_CAM = 0;
}
