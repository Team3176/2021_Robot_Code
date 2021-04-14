package frc.robot.constants;

public class AngledShooterConstants {
    public static final int MOTOR_CAN_ID = 60;
	public static final int SlotIdx = 0;
    public static final int PIDLoopIdx = 0;
    public static final int TimeoutMs = 30;
    public static boolean SENSOR_PHASE = true;
    public static boolean MOTOR_INVERT = false;

    public static final double MOVE_PCT = .1;
    public static final double HOLD_PCT = .05;
    
    public static final double[] PIDF = {.31, 0.002, 2.12, 0.35};

    public static final double GEAR_RATIO = 4/1;
	
	public static final double MAX_DEGREES_IN_TIC_UNITS = 341.3 * GEAR_RATIO;
	public static final double TICS_EQUAL_TO_5DEGREES = (MAX_DEGREES_IN_TIC_UNITS / 6 );

    public static final double MIN_TICS = -3822;
    public static final double MAX_TICS = -2461 ;
    public static final double RANGE = (MAX_TICS-MIN_TICS);

    public static final double[] POS = {MIN_TICS, MIN_TICS + (RANGE*.25), MIN_TICS+ (RANGE*.5), MIN_TICS + (RANGE*.75), MAX_TICS};
}