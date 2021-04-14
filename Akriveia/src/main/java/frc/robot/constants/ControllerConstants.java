package frc.robot.constants;

public class ControllerConstants {
    // ID values for joysticks and buttons
    public static final int ROTATION_STICK_ID = 0;      // Right side
    public static final int TRANSLATION_STICK_ID = 1;   // Left side
    public static final int XBOX_CONTROLLER_ID = 2;

    // Multipliers and other constants for Controller logic
    public static final double SLOW_DRIVE_MULT = 0.5;

    // Constants
    public static int FORWARD__AXIS_INVERSION = (MasterConstants.is2021Bot) ?  1 : -1;
    public static int STRAFE_AXIS_INVERSION = (MasterConstants.is2021Bot) ?  -1 : 1;
    public static int SPIN_AXIS_INVERSION = (MasterConstants.is2021Bot) ?  -1 : 1;
}
