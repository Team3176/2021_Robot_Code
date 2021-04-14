package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Axis;
import edu.wpi.first.wpilibj.XboxController.Button;
import frc.robot.constants.ControllerConstants;
import frc.robot.util.XboxLoneButton;
import frc.robot.util.XboxAxisAsButton;
public class Controller {
    private static Controller instance = new Controller();
    public static Controller getInstance() {return instance;}

    /* Controllers */

    private final Joystick transStick; 
    private final Joystick rotStick; 
    private final XboxController op; 

    /* Drivetrain */
    
    private final JoystickButton turboButton;
    private final JoystickButton orbitButton;
    private final JoystickButton dosadoButton;
    private final JoystickButton visionButton;
    private final JoystickButton defenseButton;
    private final JoystickButton fieldCentricButton;
    private final JoystickButton robotCentricButton;
    private final JoystickButton backRobotCentricButton; 
    private final JoystickButton resetGyroButton;
    private final JoystickButton lockSpinButton;
    
    /* Intake */

    private final Trigger intakeSpinButton;
    private final Trigger intakeReverseButton;
    private final Trigger intakeHarvestButton;
    private final Trigger intakeHarvestResetButton;

    /* Drum */

    private final Trigger drumDownButton;
    private final Trigger drumUpButton;
    private final Trigger drumCCWButton;
    private final Trigger drumInputResetButton;
    private final Trigger drumAgitateButton;
    private final Trigger drumAgitatePreShootButton;

    /* Transfer */

    private final Trigger transferStraightButton;
    private final Trigger transferPivotButton;

    /* Command Buttons */

    private final Trigger shootButton;
    private final Trigger resetShootButton;

    /* DPad POV Buttons */

    private final POVButton dpadUp;
    private final POVButton dpadDown;
    private final POVButton dpadLeft;
    private final POVButton dpadRight;

    private final Trigger angledShooterOffButton;

    public Controller() {

        // Define control sticks: Translation stick, Rotation stick, and XboxController(aka "op")
        transStick = new Joystick(ControllerConstants.TRANSLATION_STICK_ID);
        rotStick = new Joystick(ControllerConstants.ROTATION_STICK_ID);
        op = new XboxController(ControllerConstants.XBOX_CONTROLLER_ID);;

        // All buttons numbers subject to change
        turboButton = new JoystickButton(transStick, 1);
        orbitButton = new JoystickButton(rotStick, 1);
        dosadoButton = new JoystickButton(rotStick, 3);
        defenseButton = new JoystickButton(transStick, 2);
        visionButton = new JoystickButton(transStick, 3); //Should be part of the xbox controller later
        fieldCentricButton = new JoystickButton(transStick, 4);
        robotCentricButton = new JoystickButton(transStick, 5);
        backRobotCentricButton = new JoystickButton(transStick, 6);
        resetGyroButton = new JoystickButton(rotStick, 8);
        lockSpinButton = new JoystickButton(rotStick, 9);
           
        /* Intake */

        intakeSpinButton = new XboxLoneButton(op, Button.kA.value, Button.kBumperLeft.value);
        intakeReverseButton = new JoystickButton(op, Button.kA.value).and(new JoystickButton(op, Button.kBumperLeft.value));
        // intakeReverseButton = new JoystickButton(op, Button.kBumperLeft.value).and(new JoystickButton(op, Button.kA.value));
        intakeHarvestButton = new XboxLoneButton(op, Button.kY.value, Button.kBumperLeft.value);
        intakeHarvestResetButton = new JoystickButton(op, Button.kY.value).and(new JoystickButton(op, Button.kBumperLeft.value));

        /* Drum */

        drumDownButton = new JoystickButton(op, Button.kB.value).and(new JoystickButton(op, Button.kBumperLeft.value));
        drumUpButton = new XboxLoneButton(op, Button.kB.value, Button.kBumperLeft.value);
        drumInputResetButton = new XboxLoneButton(op, Button.kBack.value, Button.kBumperLeft.value);
        drumCCWButton = new JoystickButton(op, Button.kBack.value).and(new JoystickButton(op, Button.kBumperLeft.value));
        drumAgitateButton = new XboxLoneButton(op, Button.kStart.value, Button.kBumperLeft.value);
        drumAgitatePreShootButton = new JoystickButton(op, Button.kStart.value).and(new JoystickButton(op, Button.kBumperLeft.value));

        /* Transfer */

        transferPivotButton = new JoystickButton(op, Button.kX.value).and(new JoystickButton(op, Button.kBumperLeft.value));
        transferStraightButton = new XboxLoneButton(op, Button.kX.value, Button.kBumperLeft.value);

        /* Command Buttons */

        shootButton = new JoystickButton(op, Button.kBumperRight.value);
        //resetShootButton = new JoystickButton(op, Button.kBumperRight.value).and(new JoystickButton(op, Button.kBumperLeft.value));
        resetShootButton = new XboxAxisAsButton(op, Axis.kRightTrigger.value, 0.5);

        /* DPad POV Buttons */

        dpadUp = new POVButton(op, 0);
        dpadDown = new POVButton(op, 180);
        dpadRight = new POVButton(op, 90);
        dpadLeft = new POVButton(op, 270);

        angledShooterOffButton = new XboxLoneButton(op, Button.kStart.value, Button.kBumperLeft.value);
    }

    /* Swerve Axis Data */

    public double getForward() { 
        if (Math.abs(-transStick.getY()) < 0.06) {
            return 0.0;
        } else {
            return ControllerConstants.FORWARD__AXIS_INVERSION * Math.pow(transStick.getY(), 1); }
    }
    
    public double getStrafe() { 
        if (Math.abs(transStick.getX()) < 0.06) {
            return 0.0;
        } else { 
            return ControllerConstants.STRAFE_AXIS_INVERSION * Math.pow(transStick.getX(), 1); }
    }

    public double getSpin() { 
        if (Math.abs(rotStick.getX()) < 0.06) {
            return 0.0;
        } else {
            return 0.2 * ControllerConstants.SPIN_AXIS_INVERSION * (Math.pow(rotStick.getX(), 1) / 7.0); }
    } 

    public double getOrbitSpeed() { 
        if (Math.abs(rotStick.getY()) < 0.06) {
            return 0.0;
        } else {
            return Math.pow(rotStick.getY(), 1) / 7.0; }
    } 
    
    
    public int getPOVTransStick() {
        return transStick.getPOV();
    }

    /* Drivetrain Buttons */

    public JoystickButton getTurboButton() {return turboButton;}
    public JoystickButton getOrbitButton() {return orbitButton;}
    public JoystickButton getDosadoButton() {return dosadoButton;}
    public JoystickButton getVisionButton() {return visionButton;}
    public JoystickButton getDefenseButton() {return defenseButton;}
    public JoystickButton getResetGyroButton() {return resetGyroButton;}
    public JoystickButton getLockSpinButton() {return lockSpinButton;}
    public boolean isGetLockSpinButtonPressed() {return lockSpinButton.get();}
    public boolean isFieldCentricButtonPressed() {return fieldCentricButton.get();}
    public boolean isRobotCentricButtonPressed() {return robotCentricButton.get();}
    public boolean isBackRobotCentricButtonPressed() {return backRobotCentricButton.get();}

    /* Intake Buttons */

    public Trigger getIntakeSpinButton() {return intakeSpinButton;}
    public Trigger getIntakeReverseButton() {return intakeReverseButton;}
    public Trigger getIntakeHarvestButton() {return intakeHarvestButton;}
    public Trigger getIntakeHarvestResetButton() {return intakeHarvestResetButton;}

    /* Drum */

    public Trigger getDrumUpButton() {return drumUpButton;}
    public Trigger getDrumDownButton() {return drumDownButton;}
    public Trigger getDrumCCWButton() {return drumCCWButton;}
    public Trigger getDrumInputResetButton() {return drumInputResetButton;}
    // public Trigger getDrumAgitateButton() {return drumAgitateButton;}
    public Trigger getDrumAgitatePreShootButton() {return drumAgitatePreShootButton;}

    /* Transfer */

    public Trigger getTransferStraightButton() {return transferStraightButton;}
    public Trigger getTransferPivotButton() {return transferPivotButton;}

    /* Command Buttons */

    public Trigger getShootButton() {return shootButton;}
    public Trigger getResetShootButton() {return resetShootButton;}

    /* DPAD Methods */

    public POVButton getPOVUp() {return dpadUp;}
    public POVButton getPOVDown() {return dpadDown;}
    public POVButton getPOVLeft() {return dpadLeft;}
    public POVButton getPOVRight() {return dpadRight;}

    public Trigger getAngledShooterOffButton() {return angledShooterOffButton;}

/**
 * Unshifted:                   | Shifted:
 * A - Intake Roll              | Reverse Intake Roll
 * B - Drum Speed               | Drum Slow 
 * X - BallTransferPivotAndRoll | BallTransferPivot 
 * Y - Harvest                  | HarvestRest
 * Start - AngledShooterOff     | Agitate Drum Pre Shoot
 * Back - DrumInputReset        | CCWDrum
 * D-Pad Up - Shooter Up        |
 * D-Pad Down - Shooter Down    |
 * D-Pad Left - Flywheel Slow   |
 * D-Pad Right - Flywheel Speed |
 * Left Joy Button -            |
 * Right Joy Button -           |
 * Left Joy Y -                 |
 * Left Joy X -                 |
 * Right Joy Y -                |
 * Left Bumper -  Hold to Shift | Release to Unshift 
 * Right Bumper - Shoot         | ShootReset
 * Left Trigger -               | 
 * Right Trigger -        ShootReset 
 * 
 * make a Harvest cmd that executes 1) CCW Drum, 2) Intake Roll
 * make a HarvestReset cmd that executes 1) Intake Reverse, 2) Drum Agitate
 */
}