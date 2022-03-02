// Copyright (c) Team 2393, FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.cargo;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class BallHandling extends SubsystemBase
{
    // TODO Determine good voltages
    public static final double INTAKE_VOLTAGE = 6.0;
    public static final double CONVEYOR_VOLTAGE = 3.0;
    public static final double FEEDER_VOLTAGE = 3.0;
    
    private final Solenoid intake_arm = new Solenoid(RobotMap.PCM_TYPE, RobotMap.INTAKE_ARM);
    private final Solenoid shooter_angle = new Solenoid(RobotMap.PCM_TYPE, RobotMap.SHOOTER_ANGLE);

    private final WPI_TalonSRX intake = new WPI_TalonSRX(RobotMap.LEFT_INTAKE);
    private final WPI_TalonSRX secondary_intake = new WPI_TalonSRX(RobotMap.RIGHT_INTAKE);

    /** Talon to which pigeon is connected */
    public final WPI_TalonSRX pigeon_carrier = intake;

    private final WPI_TalonFX conveyor = new WPI_TalonFX(RobotMap.CONVEYOR);
    private final WPI_TalonFX feeder = new WPI_TalonFX(RobotMap.FEEDER);
    public final Spinner spinner = new Spinner();

    // private CargoSensor conveyor_sensor = new CargoSensor();
    private final DigitalInput conveyor_sensor = new DigitalInput(RobotMap.CONVEYOR_SENSOR);
    private final DigitalInput feeder_sensor = new DigitalInput(RobotMap.FEEDER_SENSOR);
    
    // Most recent state of sensors
    // (so we read only once and might be able to add a delay)
    private boolean ball_in_conveyor;
    private boolean ball_in_feeder;
    private boolean ball_ejected;

    /** Overall state of the ball handling mechanism */
    enum LoadStates
    {
        /** All off */
        OFF,
        /** Remove balls, 'unclog', overrides ShooterStates */
        REVERSE,
        /** Intake open, loading balls */
        LOADING,
        /** Intake closed */
        NOT_LOADING
    }
    private LoadStates load_state = LoadStates.OFF;
    
    /** While in LOADING or NOT_LOADING, we can also be shooting */
    enum ShooterStates
    {
        /** Awake but not shooting */
        IDLE,
        /** Shot requested, wait for spinner to be ready */
        SPINUP,
        /** Ball should come out now */
        SHOOTING
    }
    private ShooterStates shooter_state = ShooterStates.IDLE;
    
    /** Should spinner run all the time? Or start it for each SPINUP? */
    private boolean keep_spinner_running = false;
    
    /** Time when spinner should stop after last shot (unless we keep_spinner_running) */
    private double spinner_endtime = 0;

    /** Time to keep spinner running after last shot (unless we keep_spinner_running) */
    private static final double SPINNER_CONTINUE = 2.0;

    /** How long we've been trying to spin up?perform one shot */
    private final Timer spinup_timer = new Timer();

    /** How long we've been trying to perform one shot */
    private final Timer shot_timer = new Timer();
    
    /** Has a shot been requested? */
    private boolean shot_requested = false;

    /** Timeout used for shot_requested */
    private static final double SPINUP_TIMEOUT = 3.0;

    /** Timeout used for shot_requested */
    private static final double SHOT_TIMEOUT = 2.0;
    
    public BallHandling()
    {
        // Apply common settings
        initializeMotor(conveyor);
        conveyor.setInverted(true);
        initializeMotor(feeder);
        
        // Intake spinners can coast
        initializeIntakeMotor(intake);
        initializeIntakeMotor(secondary_intake);
        secondary_intake.setInverted(true);
        secondary_intake.follow(intake);

        // Allow control of shooter angle from smartboard
        SmartDashboard.setDefaultBoolean("High", false);
    }

    public static void initializeMotor(final WPI_TalonFX motor)
    {
        motor.configFactoryDefault();
        motor.clearStickyFaults();
        // Conveyor and feeder must break and accelerate crisply
        // for accurate ball movement
        motor.setNeutralMode(NeutralMode.Brake);
    }

    public static void initializeIntakeMotor(final WPI_TalonSRX motor)
    {
        motor.configFactoryDefault();
        motor.clearStickyFaults();
        // Intake can be smooth to preserve mechanics
        motor.setNeutralMode(NeutralMode.Coast);
        motor.configOpenloopRamp(0.5);
    }

    /** Overall Off/enabled */
    public void enable(final boolean do_enable)
    {
        // Reset
        shooter_state = ShooterStates.IDLE;
        keep_spinner_running = false;
        if (do_enable)
            load_state = LoadStates.NOT_LOADING;
        else
            load_state = LoadStates.OFF;
    }

    /** Toggle the 'keep spinner running' flag */
    public void toggleSpinner()
    {
        keep_spinner_running = !keep_spinner_running;
    }

    /** @param do_load Load or not? */
    public void load(final boolean do_load)
    {
        if (do_load)
            load_state = LoadStates.LOADING;
        else
            load_state = LoadStates.NOT_LOADING;
    }

    /** Toggle between loading and not */
    public void toggleLoading()
    {
        // From NOT_LOADING, go to LOADING.
        // From LOADING or REVERSE go to NOT_LOADING
        load(load_state == LoadStates.NOT_LOADING);
    }

    /** Reverse intake, 'unclog' stuck balls */
    public void reverse(final boolean do_reverse)
    {
        if (do_reverse)
            load_state = LoadStates.REVERSE;
        else if (load_state == LoadStates.REVERSE)
            load_state = LoadStates.LOADING;
    }

    /** Try to shoot ASAP */
    public void shoot()
    {
        shot_requested = true;
    }

    // private final CycleDelayFilter conveyor_sensor_delay = new CycleDelayFilter(2);

    @Override
    public void periodic()
    {
        // Read sensors to get current data
        // cargo_info = conveyor_sensor.read(); cargo_info != CargoInfo.NOTHING
        // Delay?  ball_in_conveyor = conveyor_sensor_delay.compute(conveyor_sensor.get());
        ball_in_conveyor = conveyor_sensor.get();
        ball_in_feeder = feeder_sensor.get();
        ball_ejected = spinner.isBallEjected();

        // Update indicators
        SmartDashboard.putBoolean("Ball in Conveyor", ball_in_conveyor);
        SmartDashboard.putBoolean("Ball in Feeder", ball_in_feeder);
        SmartDashboard.putBoolean("Ball Ejected", ball_ejected);
        SmartDashboard.putString("Load State", load_state.name());
        SmartDashboard.putString("Shoot State", shooter_state.name());

        // Control shooter angle from dashboard
        shooter_angle.set(SmartDashboard.getBoolean("High", false));
        
        // Update and handle states
        if (load_state == LoadStates.OFF)
            state_off();
        else if (load_state == LoadStates.REVERSE)
            state_reverse();
        else
        {
            update_shooter_state();
            handle_combined_states();
        }
    }

    private void state_off()
    {
        intake_arm.set(false);

        intake.setVoltage(0);
        conveyor.setVoltage(0);
        feeder.setVoltage(0);
        spinner.stop();
    }

    private void state_reverse()
    {
        // Open intake to allow balls to get out.
        // After REVERSE mode, we go to LOADING, which
        // leaves the intake open
        intake_arm.set(false);

        // Run motors backwards, but slowly
        intake.setVoltage(-INTAKE_VOLTAGE/2);
        conveyor.setVoltage(-CONVEYOR_VOLTAGE/2);
        feeder.setVoltage(-FEEDER_VOLTAGE/2);
        spinner.stop();
    }

    private void update_shooter_state()
    {
        if (shooter_state == ShooterStates.IDLE  &&  shot_requested)
        {   // From idle, start a shot on request
            shooter_state = ShooterStates.SPINUP;
            System.out.println("Spinup...");
            spinup_timer.stop();
            spinup_timer.reset();
            spinup_timer.start();
            shot_requested = false;
        }
        if (shooter_state == ShooterStates.SPINUP)
        {   // Is spinner fast enough?
            if (spinner.getSpeed() >= 0.95*SmartDashboard.getNumber("SpinnerSetpoint", 0.0))
            {
                shooter_state = ShooterStates.SHOOTING;
                System.out.println("Shooting...");
                shot_timer.stop();
                shot_timer.reset();
                shot_timer.start();
            }
            else if (spinup_timer.hasElapsed(SPINUP_TIMEOUT))
            {
                shooter_state = ShooterStates.SHOOTING;
                System.out.println("Not reaching spinner setpoint, shooting anyway");
                shot_timer.stop();
                shot_timer.reset();
                shot_timer.start();
            }
        }
        if (shooter_state == ShooterStates.SHOOTING)
        {   // Did we eject a ball?
            if (ball_ejected)
            {
                shooter_state = ShooterStates.IDLE;
                System.out.println("Shot!");
                spinner_endtime = keep_spinner_running ? 0 : Timer.getFPGATimestamp() + SPINNER_CONTINUE;
            }
            else
             if (shot_timer.hasElapsed(SHOT_TIMEOUT))
            {
                shooter_state = ShooterStates.IDLE;
                System.out.println("No ball shot? Giving up");
                spinner_endtime = keep_spinner_running ? 0 : Timer.getFPGATimestamp() + SPINNER_CONTINUE;
            }
        }
    }

    private void handle_combined_states()
    {
        final boolean have_all_balls = ball_in_conveyor  &&  ball_in_feeder;
        final boolean have_single_ball_to_move = ball_in_conveyor  &&  !ball_in_feeder;

        // Automatically close intake when we have 2 balls
        if (have_all_balls)
            load_state = LoadStates.NOT_LOADING;

        // Intake: Open while loading
        intake_arm.set(load_state == LoadStates.LOADING);
        // Run while loading until we have two balls
        if (load_state == LoadStates.LOADING  &&  !have_all_balls)
            intake.setVoltage(INTAKE_VOLTAGE);
        else
            intake.setVoltage(0);

        // Conveyor: Stop while shooting to prevent feeding a 2nd ball
        // Otherwise try to load two balls, or move single ball forward 
        if (shooter_state == ShooterStates.IDLE &&
            ( (load_state == LoadStates.LOADING && !have_all_balls) ||
              (load_state == LoadStates.NOT_LOADING && have_single_ball_to_move)
            ))
            conveyor.setVoltage(CONVEYOR_VOLTAGE);
        else
            conveyor.setVoltage(0);

        // Feeder: Shooting?
        // otherwise: Loading? Run until there's a ball in the feeder
        //        Not Loading? Run if one ball on conveyor that needs to move forward
        if (shooter_state == ShooterStates.SHOOTING ||
            (shooter_state == ShooterStates.IDLE  &&  load_state == LoadStates.LOADING     &&  !ball_in_feeder) ||
            (shooter_state == ShooterStates.IDLE  &&  load_state == LoadStates.NOT_LOADING &&  have_single_ball_to_move))
            feeder.setVoltage(FEEDER_VOLTAGE);
        else
            feeder.setVoltage(0.0);
        
        // Spinner: Always, shooting, leave running for a few secs after last shot?
        if (keep_spinner_running ||
            shooter_state == ShooterStates.SPINUP ||
            shooter_state == ShooterStates.SHOOTING  ||
            Timer.getFPGATimestamp() < spinner_endtime)
            spinner.run();
        else
            spinner.stop();
    }
}
