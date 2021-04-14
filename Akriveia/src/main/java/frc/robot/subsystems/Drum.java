package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.SlewRateLimiter;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.constants.DrumConstants;

import java.util.UUID;

/**
 * <b> The Drum subsystem </b>
 * <p>
 * This mechanism holds up to 5 power cells, after the Intake and before the ball transfer / Flywheel. It spins at variable
 * SPEEDS to unload power cells, and can shake to dislodge power cells stuck on top of one another inside the Drum.
 */

public class Drum extends SubsystemBase {

  /** Creates a new Drum. */

  private CANSparkMax drumMotor = new CANSparkMax(DrumConstants.MOTOR_CAN_ID, MotorType.kBrushless);
  private CANPIDController drumPIDController;
  private CANEncoder drumEncoder;
  private SlewRateLimiter rateLimiter;
  private static Drum instance = new Drum();
  public boolean drumPctOutputMode = false;
  private boolean isRateLimitOff = true;
  private int lastSetting = 0;
  private int fQuarter, half, tQuarter; //First Quarter, Half, Third Quarter
  private int range; //The range of the lastSetting and the level
  private int direction = 1;
  private double shakeStartTime = -1;
  private double shakeIterations = 0;

  private PowerManagement m_PowerManagement;
  private Timer time;

  String procTag;


  /**
   * Initializes the Drum subsystem once at code deploy.
   * <p>
   * Sets the Drum's PIDF constants, located in the Constants class, and creates rateLimiter and encoder objects.
   */

  public Drum() {
    m_PowerManagement = PowerManagement.getInstance();
    drumMotor.restoreFactoryDefaults();
    drumPIDController = drumMotor.getPIDController();
    drumEncoder = drumMotor.getEncoder();

    drumMotor.setClosedLoopRampRate(.3);

    drumPIDController.setReference(0.0, ControlType.kVelocity);

    rateLimiter = new SlewRateLimiter(DrumConstants.kRampRate, 0);

    time = new Timer();
    
    /* Set PID constants */

    drumPIDController.setP(DrumConstants.PIDF[0]);
    drumPIDController.setI(DrumConstants.PIDF[1]);
    drumPIDController.setD(DrumConstants.PIDF[2]);
    drumPIDController.setFF(DrumConstants.PIDF[3]);
    drumPIDController.setIZone(DrumConstants.kIZone);
    drumPIDController.setOutputRange(DrumConstants.kMinOutput, DrumConstants.kMaxOutput);    
  }

  /**
   * Called one during each run of a nonzero spin speed method. This is to turn the rateLimiter back on at the Drum motor's current
   * velocity if it was turned off during the drumPowerOff and shakeDrum methods. It prevents sudden, violent jumps in speed.
   * @see Drum.drumPowerOff
   * @see Drum.shakeDrum
   */

  public void reengageRampLimit() {
    // System.out.println("************ Drum.reengageRampLimit():  Entered the method.....");
     if (isRateLimitOff) {
      rateLimiter.reset(drumEncoder.getVelocity());
      // System.out.println("********* Drum.reengageRampLimit():  getVelocity()= "+drumEncoder.getVelocity());
      //isRateLimitOff = false;
     }
  }

  /**
   * <b> Spin </b>
   * <p>
   * Sets the Drum motor to whatever the index level is in the drumSPEEDS array.
   * @param level the index in the SPEEDS array that it is set to
   * @param direction the direction of the velocity, 1 = Fast, 0 = Slow, 2 = Same, Others = No Speed
   * @see commands.teleop.DrumVelocitySlow
   * @see commands.teleop.DrumVelocitySpeed
   */

  public void pidVelCtrl_step4LevelsToDesiredSpeed(int level, int direction, String procTag) {
    pidVelCtrl_step4LevelsToDesiredSpeed(level, direction);
    this.procTag = procTag; 
  }

   /**
    * @param level Corresponds to the index of an array containing the RPM speeds
    * @param direction  Determines if we are stepping up or down to the desired "speed level"
    */
  public boolean pidVelCtrl_step4LevelsToDesiredSpeed(int level, int direction) {
    time.start();
    time.reset();
    reengageRampLimit();
    // if(level == 0) {isRateLimitOff = true;}
    // System.out.println("*********************** Drum.setSpeed:  level="+level+" lastSetting="+lastSetting);
    // System.out.println("Magic");
    // System.out.println(level);
    range = Math.abs(DrumConstants.SPEEDS[lastSetting] - DrumConstants.SPEEDS[level]);
    if(direction == 1) {//Up
      fQuarter = DrumConstants.SPEEDS[lastSetting] + (range) / 4;
      half = DrumConstants.SPEEDS[lastSetting] + range / 2;
      tQuarter = DrumConstants.SPEEDS[lastSetting] + 3 * (range) / 4;
    } else if(direction == 0) {//Down
      fQuarter = DrumConstants.SPEEDS[level] + (3 * (range) / 4);
      half = DrumConstants.SPEEDS[level] + range / 2;
      tQuarter = DrumConstants.SPEEDS[level] + (range) / 4;
    } else {
      fQuarter = 3 * DrumConstants.SPEEDS[lastSetting] / 4;
      half = DrumConstants.SPEEDS[lastSetting] / 2;
      tQuarter = DrumConstants.SPEEDS[lastSetting] / 4;
      level = 0;
    }
    // if(time.get() % 2 == 0) System.out.println("The last setting is /*Should be currently there*/: " + DrumConstants.SPEEDS[lastSetting] + ", fQuarter: " + fQuarter + ", half" + half + ", tQuarter" + tQuarter + ", level " + DrumConstants.SPEEDS[level]);
    drumPIDController.setReference(DrumConstants.SPEEDS[lastSetting], ControlType.kVelocity);
    if(direction != 2) {
      Timer.delay(1);
      pidVelCtrl_set(fQuarter);
      Timer.delay(1);
      pidVelCtrl_set(half);
      Timer.delay(1);
      pidVelCtrl_set(tQuarter);
      Timer.delay(1);
      pidVelCtrl_set(DrumConstants.SPEEDS[level]);
      lastSetting = level;
    }
    time.stop(); 
    return true;
  }

  /**
   * @param pct The percentage to set the Drum to
   */
  public void pctCtrl_set(double pct) {
    drumMotor.set(pct);
  }

  /**
   * 
   * @param rpm desired RPM to be used as setpoint for velocity of Drum
   */
  public void pidVelCtrl_set(int rpm) {
    drumPIDController.setReference(rpm, ControlType.kVelocity);
  }


  public boolean pidVelCtrl_setRpmLevel(int lvl, String procTag) {
    boolean rtnVal = pidVelCtrl_setRpmLevel(lvl);
    this.procTag = procTag;
    return rtnVal;
  }


  /**
   * 
   * @param lvl Corresponds to index of DrumConstants.SPEEDS array.  Typical Range = [0,1,2,3,4].
   * @return True if lvl within accepted range.
   */
  public boolean pidVelCtrl_setRpmLevel(int lvl) {
    if ((lvl < 0) || (lvl > DrumConstants.SPEEDS.length)) {
      // System.out.println("Drum.pidVelCtrl_setRPMLevel("+lvl+"): index out of range.");
      // System.out.println("Drum.pidVelCtrl_setRPMLevel("+lvl+"):  Parameter input value of "+lvl+" exceeds length of DrumConstants.SPEEDS array which is "+DrumConstants.SPEEDS.length+" elements long.");
      return false;
    } else {
      pidVelCtrl_set(DrumConstants.SPEEDS[lvl]);
      return true;
    }
  }

  /**
   * <b> Shaking the Drum: </b>
   * <p>
   * Shakes the Drum rapidly, 5 times in each direction plus one extra time in the positive direction to slow it down initially.
   * Utilizes System.nanoTime to create a delay in between each change in direction.
   * <p>
   * The returned boolean tells the AgitateDrum command whether or not to keep running. Avoids using a loop as <b> loops
   * confuse the CommandScheduler</b>.
   * @see commands.AgitateDrum
   * @return Boolean -- whether or not the Drum has changed direction the specified number of times
   */

  public boolean shakeDrum() {
    isRateLimitOff = true;
    if (shakeIterations < 10) {
      if (shakeStartTime == -1) {
        drumMotor.set(DrumConstants.SHAKE_PCT * direction);
        shakeStartTime = System.nanoTime() / DrumConstants.MILLI;
        direction *= -1;
      }
      if ((System.nanoTime() / DrumConstants.MILLI) - shakeStartTime >= 150) {
        drumMotor.set(DrumConstants.SHAKE_PCT * direction);
        direction *= -1;
        shakeIterations += 1;
        shakeStartTime = System.nanoTime() / DrumConstants.MILLI;
      }
    } else {
      shakeIterations = 0;
      shakeStartTime = -1;
      direction = 1;
      return true;
    }
    return false;
  }

  /**
   * <b> Checks the time </b>
   * <p>
   * If: spins at the slowest speed in reverse
   * <p>
   * If not: sets the motor to its last setting
   */

  public void ShortSpinInReverse() {
    if (System.nanoTime() / DrumConstants.SEC >= 2) {
      drumPIDController.setReference(rateLimiter.calculate(DrumConstants.SPEEDS[1] * -1), ControlType.kVelocity);
    }
    pidVelCtrl_step4LevelsToDesiredSpeed(lastSetting, 2);
  }

  public boolean PreShootSpinAgitate() {
    isRateLimitOff = true;
    if (shakeIterations < 4) {
      if (shakeStartTime == -1) {
        drumMotor.set(DrumConstants.SHAKE_PCT * direction);
        shakeStartTime = System.nanoTime() / DrumConstants.MILLI;
        direction *= -1;
      }
      if ((System.nanoTime() / DrumConstants.MILLI) - shakeStartTime >= 150) {
        drumMotor.set(DrumConstants.SHAKE_PCT * direction);
        direction *= -1;
        shakeIterations += 1;
        shakeStartTime = System.nanoTime() / DrumConstants.MILLI;
      }
    } else {
      return true;
    }
    return false;
  }

  public void resetShakeVariables() {
    shakeIterations = 0;
    shakeStartTime = -1;
    direction = 1;
  }

  /**
   * Gets the Current/Amp of the Drum and if it is past a certain voltage it stops the intake
   */
  public void checkForCurrentSpike() {
    double amps = m_PowerManagement.getDrumAvgAmp();
    if (amps >= 20) {
      drumMotor.set(0);
      drumMotor.disable();
    }
  }

  /**
   * Sets the motor to -0.3 percent which is slow counter clockwise
   */

  public void CounterClockwise() {
    drumMotor.set(-0.3);
  }

  /**
   * @param PCT the percentage to make negative and go counter clockwise
   */

  public void CounterClockwise(double PCT) {
    drumMotor.set(-PCT);
  }

  /**
   * @return the last speed of the Drum
   */

  public int getLastSetting() {
    return lastSetting;
  }

  public String getProcTag() {
    return procTag;
  }

  /**
   * <b> Returns Drum Instance </b>
   * <p>
   * Returns the instance created above for Drum.
   * @return instance
   */

  public static Drum getInstance() {
    return instance;
  }

  @Override
  public void periodic() {
    //checkForCurrentSpike();
  }
}