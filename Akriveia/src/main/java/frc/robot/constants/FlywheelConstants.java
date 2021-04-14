package frc.robot.constants;

public class FlywheelConstants {
    public static final int MOTOR_CAN_ID = 54;
    public static final double[] PIDF = {/*kP*/ 0.14,/*kI*/ 0.0,/*kD*/ 1.5,/*kF*/ 0.0};
    public static final int kIzone = 0;
    public static final double kPeakOutput = 1.0;
    public static final int kTimeoutMs = 30;
    public static final int kPIDLoopIdx = 0;
    public static final int[] SPEEDS = {0, 3000, 3850, 4000, 4150, 4300, 4400, 5500};
}