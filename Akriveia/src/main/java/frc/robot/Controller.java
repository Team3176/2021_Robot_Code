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
    
    private final JoystickButton transStick_Button1;
    private final JoystickButton transStick_Button2;
    private final JoystickButton transStick_Button3;
    private final JoystickButton transStick_Button4;
    private final JoystickButton transStick_Button5;
    private final JoystickButton transStick_Button6;
    private final JoystickButton transStick_Button7;
    private final JoystickButton transStick_Button8;
    private final JoystickButton transStick_Button9;
    private final JoystickButton transStick_Button10;
    private final JoystickButton transStick_Button11;
    private final JoystickButton transStick_Button12;
    private final JoystickButton transStick_Button13;
    private final JoystickButton transStick_Button14;
    private final JoystickButton rotStick_Button1;
    private final JoystickButton rotStick_Button2;
    private final JoystickButton rotStick_Button3;
    private final JoystickButton rotStick_Button4;
    private final JoystickButton rotStick_Button5;
    private final JoystickButton rotStick_Button6;
    private final JoystickButton rotStick_Button7;
    private final JoystickButton rotStick_Button8;
    private final JoystickButton rotStick_Button9;
    private final JoystickButton rotStick_Button10;
    private final JoystickButton rotStick_Button11;
    private final JoystickButton rotStick_Button12;
    private final JoystickButton rotStick_Button13;
    private final JoystickButton rotStick_Button14;
    
    private final Trigger op_ButtonY;
    private final Trigger op_ButtonYPlusBumperLeft;

    /* Intake */

    private final Trigger op_ButtonA;
    private final Trigger op_ButtonAPlusBumperLeft;

    /* Drum */

    private final Trigger op_ButtonBPlusBumperLeft;
    private final Trigger op_ButtonB;
    private final Trigger op_ButtonBackPlusBumperLeft;
    private final Trigger op_ButtonBack;
    private final Trigger op_ButtonStart;
    private final Trigger op_ButtonStartPlusBumperLeft;

    /* Transfer */

    // private final Trigger op_ButtonX;
    private final JoystickButton op_ButtonX;
    // private final Trigger op_ButtonXPlusBumperLeft;

    /* Command Buttons */

    // private final Trigger op_BumperRight;
    private final JoystickButton op_BumperRight;
    // private final Trigger op_BumperRightPlusBumperLeft;
    private final Trigger op_TriggerRight;
    private final Trigger op_TriggerLeft;

    /* DPad POV Buttons */

    private final POVButton op_DpadUp;
    private final POVButton op_DpadDown;
    private final POVButton op_DpadLeft;
    private final POVButton op_DpadRight;

    private final Trigger angledShooterOffButton;

    /* ANGLED SHOOTED TEST BUTTONS */

    private final Trigger ushootertestbutton;
    private final Trigger dshootertestbutton;
    private final Trigger hshootertestbutton;
    private final Trigger pshootertestbutton;

    private final Trigger k1;
    private final Trigger k2;

    public Controller() {

        // Define control sticks: Translation stick, Rotation stick, and XboxController(aka "op")
        transStick = new Joystick(ControllerConstants.TRANSLATION_STICK_ID);
        rotStick = new Joystick(ControllerConstants.ROTATION_STICK_ID);
        op = new XboxController(ControllerConstants.XBOX_CONTROLLER_ID);;

        // All buttons numbers subject to change
        transStick_Button1 = new JoystickButton(transStick, 1);
        transStick_Button2 = new JoystickButton(transStick, 2);
        transStick_Button3 = new JoystickButton(transStick, 3); 
        transStick_Button4 = new JoystickButton(transStick, 4);
        transStick_Button5 = new JoystickButton(transStick, 5);
        transStick_Button6 = new JoystickButton(transStick, 6);
        transStick_Button7 = new JoystickButton(transStick, 7);
        transStick_Button8 = new JoystickButton(transStick, 8);
        transStick_Button9 = new JoystickButton(transStick, 9);
        transStick_Button10 = new JoystickButton(transStick, 10);
        transStick_Button11 = new JoystickButton(transStick, 11);
        transStick_Button12 = new JoystickButton(transStick, 12);
        transStick_Button13 = new JoystickButton(transStick, 13);
        transStick_Button14 = new JoystickButton(transStick, 14);
        rotStick_Button1 = new JoystickButton(rotStick, 1);
        rotStick_Button2 = new JoystickButton(rotStick, 2);
        rotStick_Button3 = new JoystickButton(rotStick, 3);
        rotStick_Button4 = new JoystickButton(rotStick, 4);
        rotStick_Button5 = new JoystickButton(rotStick, 5);
        rotStick_Button6 = new JoystickButton(rotStick, 6);
        rotStick_Button7 = new JoystickButton(rotStick, 7);
        rotStick_Button8 = new JoystickButton(rotStick, 8);
        rotStick_Button9 = new JoystickButton(rotStick, 9);
        rotStick_Button10 = new JoystickButton(rotStick, 10);
        rotStick_Button11 = new JoystickButton(rotStick, 11);
        rotStick_Button12 = new JoystickButton(rotStick, 12);
        rotStick_Button13 = new JoystickButton(rotStick, 13);
        rotStick_Button14 = new JoystickButton(rotStick, 14);
          

        op_ButtonY = new XboxLoneButton(op, Button.kY.value, Button.kBumperLeft.value);
        op_ButtonYPlusBumperLeft = new JoystickButton(op, Button.kY.value).and(new JoystickButton(op, Button.kBumperLeft.value));
        /* Intake */

        op_ButtonA = new XboxLoneButton(op, Button.kA.value, Button.kBumperLeft.value);
        op_ButtonAPlusBumperLeft = new JoystickButton(op, Button.kA.value).and(new JoystickButton(op, Button.kBumperLeft.value));
        // intakeReverseButton = new JoystickButton(op, Button.kBumperLeft.value).and(new JoystickButton(op, Button.kA.value));

        /* Drum */

        op_ButtonBPlusBumperLeft = new JoystickButton(op, Button.kB.value).and(new JoystickButton(op, Button.kBumperLeft.value));
        op_ButtonB = new XboxLoneButton(op, Button.kB.value, Button.kBumperLeft.value);
        op_ButtonBack = new XboxLoneButton(op, Button.kBack.value, Button.kBumperLeft.value);
        op_ButtonBackPlusBumperLeft = new JoystickButton(op, Button.kBack.value).and(new JoystickButton(op, Button.kBumperLeft.value));
        op_ButtonStart = new XboxLoneButton(op, Button.kStart.value, Button.kBumperLeft.value);
        op_ButtonStartPlusBumperLeft = new JoystickButton(op, Button.kStart.value).and(new JoystickButton(op, Button.kBumperLeft.value));

        /* Transfer */

        // op_ButtonXPlusBumperLeft = new JoystickButton(op, Button.kX.value).and(new JoystickButton(op, Button.kBumperLeft.value));
        // op_ButtonX = new XboxLoneButton(op, Button.kX.value, Button.kBumperLeft.value);
        op_ButtonX = new JoystickButton(op, Button.kX.value);

        /* Command Buttons */

        // op_BumperRight = new XboxLoneButton(op, Button.kBumperRight.value, Button.kBumperLeft.value);
        op_BumperRight = new JoystickButton(op, Button.kBumperRight.value);
        // op_BumperRightPlusBumperLeft = new JoystickButton(op, Button.kBumperRight.value).and(new JoystickButton(op, Button.kBumperLeft.value));
        op_TriggerRight = new XboxAxisAsButton(op, Axis.kRightTrigger.value, 0.5);
        op_TriggerLeft = new XboxAxisAsButton(op, Axis.kLeftTrigger.value, 0.5);

        /* DPad POV Buttons */

        op_DpadUp = new POVButton(op, 0);
        op_DpadDown = new POVButton(op, 180);
        op_DpadRight = new POVButton(op, 90);
        op_DpadLeft = new POVButton(op, 270);

        angledShooterOffButton = new XboxLoneButton(op, Button.kStart.value, Button.kBumperLeft.value);

        ushootertestbutton = new JoystickButton(transStick, 8);
        dshootertestbutton = new JoystickButton(transStick, 9);
        hshootertestbutton = new JoystickButton(transStick, 10);
        pshootertestbutton = new JoystickButton(transStick, 14);

        k1 = new XboxAxisAsButton(op, Axis.kLeftTrigger.value, 0.5);
        k2 = new XboxLoneButton(op, Button.kStickLeft.value, Button.kBumperLeft.value);
    }
    public Trigger getLTrigger() {return k1;}
    public Trigger getLStick() {return k2;}

    public Trigger getUShoot() {return ushootertestbutton;}
    public Trigger getDShoot() {return dshootertestbutton;}
    public Trigger getHShoot() {return hshootertestbutton;}
    public Trigger getPShoot() {return pshootertestbutton;}

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
            return Math.pow(rotStick.getY(), 1); }
    } 
    
    
    public int getPOVTransStick() {
        return transStick.getPOV();
    }

    /* Drivetrain Buttons */

    public JoystickButton getTransStick_Button1() {return transStick_Button1;}
    public JoystickButton getTransStick_Button2() {return transStick_Button2;}
    public JoystickButton getTransStick_Button3() {return transStick_Button3;}
    public JoystickButton getTransStick_Button4() {return transStick_Button4;}
    public JoystickButton getTransStick_Button5() {return transStick_Button5;}
    public JoystickButton getTransStick_Button6() {return transStick_Button6;}
    public JoystickButton getTransStick_Button7() {return transStick_Button7;}
    public JoystickButton getTransStick_Button8() {return transStick_Button8;}
    public JoystickButton getTransStick_Button9() {return transStick_Button9;}
    public JoystickButton getTransStick_Button10() {return transStick_Button10;}
    public JoystickButton getTransStick_Button11() {return transStick_Button11;}
    public JoystickButton getTransStick_Button12() {return transStick_Button12;}
    public JoystickButton getTransStick_Button13() {return transStick_Button13;}
    public JoystickButton getTransStick_Button14() {return transStick_Button14;}
    public JoystickButton getRotStick_Button1() {return rotStick_Button1;}
    public JoystickButton getRotStick_Button2() {return rotStick_Button2;}
    public JoystickButton getRotStick_Button3() {return rotStick_Button3;}
    public JoystickButton getRotStick_Button4() {return rotStick_Button4;}
    public JoystickButton getRotStick_Button5() {return rotStick_Button5;}
    public JoystickButton getRotStick_Button6() {return rotStick_Button6;}
    public JoystickButton getRotStick_Button7() {return rotStick_Button7;}
    public JoystickButton getRotStick_Button8() {return rotStick_Button8;}
    public JoystickButton getRotStick_Button9() {return rotStick_Button9;}
    public JoystickButton getRotStick_Button10() {return rotStick_Button10;}
    public JoystickButton getRotStick_Button11() {return rotStick_Button11;}
    public JoystickButton getRotStick_Button12() {return rotStick_Button12;}
    public JoystickButton getRotStick_Button13() {return rotStick_Button13;}
    public JoystickButton getRotStick_Button14() {return rotStick_Button14;}

    public boolean isFieldCentricButtonPressed() {return rotStick_Button9.get();}
    public boolean isRobotCentricButtonPressed() {return transStick_Button4.get();}
    //public boolean isRobotCentricButtonPressed() {return transStick_Button5.get();}
    public boolean isRotStick_Button4_Pressed() {return rotStick_Button4.get();}
    
    public Trigger getOp_ButtonY() {return op_ButtonY;}
    public Trigger getOp_ButtonYPlusBumperLeft() {return op_ButtonYPlusBumperLeft;}


    /* Intake Buttons */

    public Trigger getOp_ButtonA() {return op_ButtonA;}
    public Trigger getOp_ButtonAPlusBumperLeft() {return op_ButtonAPlusBumperLeft;}

    /* Drum */

    public Trigger getOp_ButtonB() {return op_ButtonB;}
    public Trigger getOp_ButtonBPlusBumperLeft() {return op_ButtonBPlusBumperLeft;}
    public Trigger getOp_ButtonBackPlusBumperLeft() {return op_ButtonBackPlusBumperLeft;}
    public Trigger getOp_ButtonBack() {return op_ButtonBack;}
    // public Trigger getDrumAgitateButton() {return drumAgitateButton;}
    public Trigger getOp_ButtonStartPlusBumperLeft() {return op_ButtonStartPlusBumperLeft;}

    /* Transfer */

    public JoystickButton getOp_ButtonX() {return op_ButtonX;}
    // public Trigger getOp_ButtonX() {return op_ButtonX;}
    // public Trigger getOp_ButtonXPlusBumperLeft() {return op_ButtonXPlusBumperLeft;}

    /* Command Buttons */

    // public Trigger getOp_BumperRight() {return op_BumperRight;}
    // public Trigger getOp_BumperRightPlusBumperLeft() {return op_BumperRightPlusBumperLeft;}
    public JoystickButton getOp_BumperRight() {return op_BumperRight;}

    public Trigger getOp_TriggerRight() {return op_TriggerRight;}
    public Trigger getOp_TriggerLeft() {return op_TriggerLeft;}

    /* DPAD Methods */

    public POVButton getOp_DpadUp() {System.out.println("DPAD UP UP AND AWAY"); return op_DpadUp;}
    public POVButton getOp_DpadDown() {return op_DpadDown;}
    public POVButton getOp_DpadLeft() {return op_DpadLeft;}
    public POVButton getOp_DpadRight() {return op_DpadRight;}

    public Trigger getOp_ButtonStart() {return op_ButtonStart;}

/**
 * Unshifted:                   | Shifted:
 * A - Intake Roll              | Reverse Intake Roll
 * B - Drum Speed               | Drum Slow 
 * X - BallTransferPivotAndRoll | BallTransferPivot 
 * Y -                          | 
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