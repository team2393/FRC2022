package frc.robot.cargo;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class BallHandling extends SubsystemBase
{
    // TODO Determine good voltages for running intake and feeder
    private static final double INTAKE_VOLTAGE = 3.0;
    private static final double CONVEYOR_VOLTAGE = 4.0;
    private static final double FEEDER_VOLTAGE = 6.0;

    private Solenoid intake_arm = new Solenoid(PneumaticsModuleType.REVPH, RobotMap.INTAKE_ARM);
    private WPI_TalonFX intake = new WPI_TalonFX(RobotMap.INTAKE);
    private WPI_TalonFX conveyor = new WPI_TalonFX(RobotMap.CONVEYOR);
    private WPI_TalonFX feeder = new WPI_TalonFX(RobotMap.FEEDER);
    private Spinner spinner = new Spinner();
    private DigitalInput conveyor_sensor = new DigitalInput(RobotMap.CONVEYOR_SENSOR);
    private DigitalInput feeder_sensor = new DigitalInput(RobotMap.FEEDER_SENSOR);

    // Could get replaced by watching spinner current
    private DigitalInput ejection_sensor = new DigitalInput(RobotMap.EJECTION_SENSOR);

    enum LoadStates
    {
        OFF,
        LOADING,
        NOT_LOADING
    }

    private LoadStates load_state = LoadStates.OFF;

    enum ShooterStates
    {
        OFF,
        IDLE,
        SPINUP,
        SHOOTING
    }

    private ShooterStates shooter_state = ShooterStates.OFF;

    private boolean keep_spinner_running = false;

    private Timer spinner_runtime = new Timer();

    private boolean shot_requested = false;

    /** Overall Off/enabled */
    public void enable(boolean do_enable)
    {
        if (do_enable)
        {
            load_state = LoadStates.NOT_LOADING;
            shooter_state = ShooterStates.IDLE;
        }
        else
        {
            load_state = LoadStates.OFF;
            shooter_state = ShooterStates.OFF;
            keep_spinner_running = false;
        }
    }

    /** Toggle the 'keep spinner running' flag */
    public void toggleSpinner()
    {
        if (keep_spinner_running)
            keep_spinner_running = false;
        else
            keep_spinner_running = true;
    }

    /** Switch between loading and not */
    public void load(boolean do_load)
    {
        if (do_load)
            load_state = LoadStates.LOADING;
        else
            load_state = LoadStates.NOT_LOADING;
    }

    public void toggleLoading()
    {
        load(load_state == LoadStates.NOT_LOADING);
    }

    /** Try to shoot ASAP */
    public void shoot()
    {
        shot_requested = true;
    }

    @Override
    public void periodic()
    {
        // Common OFF state
        if (load_state == LoadStates.OFF  ||
            shooter_state == ShooterStates.OFF)
        {
            intake_arm.set(false);
            intake.setVoltage(0);
            conveyor.setVoltage(0);
            feeder.setVoltage(0);
            spinner.stop();
        }

        // Loading/unloading
        if (load_state == LoadStates.LOADING)
        {
            intake_arm.set(true);

            boolean have_all_balls = conveyor_sensor.get() && feeder_sensor.get();
            if (have_all_balls)
            {
                intake.setVoltage(0);
                conveyor.setVoltage(0);
            }
            else
            {
                intake.setVoltage(INTAKE_VOLTAGE);
                conveyor.setVoltage(CONVEYOR_VOLTAGE);
            }
        }
        else if (load_state == LoadStates.NOT_LOADING)
        {
            intake_arm.set(false);
            intake.setVoltage(0);
            
            boolean have_single_ball_to_move_on =
               conveyor_sensor.get() && !feeder_sensor.get();
            if (have_single_ball_to_move_on)
                conveyor.setVoltage(CONVEYOR_VOLTAGE);
            else
                conveyor.setVoltage(0);
        }

        // Shooter states
        if (shooter_state == ShooterStates.IDLE)
        {
            feeder.setVoltage(0.0);

            // Always run, or leave running for 2 sec after last shot
            if (keep_spinner_running  ||
                ! spinner_runtime.hasElapsed(2.0))
                spinner.run();
            else
                spinner.stop();
            
            if (shot_requested)
            {
                shooter_state = ShooterStates.SPINUP;
                shot_requested = false;
            }
        }
        if (shooter_state == ShooterStates.SPINUP)
        {
            feeder.setVoltage(0.0);
            spinner.run();

            // Is spinner fast enough?
            // TODO Find good threshold. Is it 95%??
            if (spinner.getSpeed() >= 0.95*SmartDashboard.getNumber("Spinner Setpoint", 0.0))
                shooter_state = ShooterStates.SHOOTING;
        }
        if (shooter_state == ShooterStates.SHOOTING)
        {
            feeder.setVoltage(FEEDER_VOLTAGE);
            spinner.run();

            // Did we eject a ball?
            if (ejection_sensor.get())
            {
                shooter_state = ShooterStates.IDLE;
                spinner_runtime.reset();
                spinner_runtime.start();
            }
        }
    }
}
