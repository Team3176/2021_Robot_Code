package frc.robot.constants;

public class DrumConstants {
    public static final int MOTOR_CAN_ID = 50;
    public static final int IDLER_IRSENSOR_DIO_ID = 1;
    public static final int BACKWALL_IRSENSOR_DIO_ID = 4;

    public static final double DURATION_LIMIT_BACK_IR_SENSOR_SECONDS = 0.5;
    public static final double DURATION_LIMIT_IDLER_IR_SENSOR_SECONDS = 0.5;

    /* Drum PIDF Constants for SparkMAX Neo 550 */

    public static final double kIZone = 0; 
    public static final double kRampRate = 2500;
    public static final double[] PIDF = {0.00013, 0.0000008, 0.003, 0.000097};    

    public static final double kMaxOutput = 1.0;
    public static final double kMinOutput = -1.0;

    /* RPM Array */

    public static final int[] SPEEDS = {0, 2500, 3500/**/,3750/**/, 4250/**/, 4500, 4600/**/, 5000/**/, 7000, 8500};
    public static final double SHAKE_PCT = 0.35;
    public static final double MILLI = 1000000;
    public static final double SEC = 1000000000;
}