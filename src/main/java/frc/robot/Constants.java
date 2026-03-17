package frc.robot;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.robot.generated.TunerConstants;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.CANBus;



public class Constants {

    public static class Driving {
        public static final LinearVelocity kMaxSpeed = TunerConstants.kSpeedAt12Volts;
        public static final AngularVelocity kMaxRotationalRate = RotationsPerSecond.of(1);
        public static final AngularVelocity kPIDRotationDeadband = kMaxRotationalRate.times(0.005);
    }

    public static class KrakenX60 {
        public static final AngularVelocity kFreeSpeed = RPM.of(6000);
    }

    public static final AngularVelocity kPivotRPM = RPM.of(6000);

    public static final double SLOW = 0.25;

    public static final double SLOW_SHOOT = 0.5;

    public static final double SLOW_HANG = 0.25;

    //public static final double INTAKE = 0.5;


    //Shoot Calc Stuff

    public static final double SHOOTER_OFFSET_X = 0.23;
    public static final double SHOOTER_OFFSET_Y = 0;

    public static final double ROBOT_WEIGHT = 50.10;

    public static final double BALL_MASS = 0.215;
    public static final double BALL_DIAMETER = 0.1501;
    public static final double DRAG_COEFF = 0.47;
    public static final double MAGNUS_COEFF = 0.2;
    public static final double AIR_DENSITY = 1.225;

    public static final double EXIT_HEIGHT = 0.43;
    public static final double WHEEL_DIAMETER = 0.1016;

    public static final double TARGET_HEIGHT = 1.83;

    public static final double SLIP_FACTOR = 0.6;
    public static final double LAUNCH_ANGLE = 45.0;
    public static final double SIM_TIMESTEP = 0.001;
    
    public static final double MIN_RPM = 1500;
    public static final double MAX_RPM = 6000;

    public static final int SEARCH_ITERS = 25;
    public static final double MAX_SIM_TIME = 5.0;


    

    // PWM IDS


    public static final int HOOD_L = 0;
    public static final int HOOD_R = 1;

    public static final int LATCH = 2;
    
    
    // DIO IDS

    // CAN IDS
    // TALON 
    public static final int FRONT_RIGHT_DRIVE = 25;
    public static final int REAR_RIGHT_DRIVE = 55;
    public static final int FRONT_LEFT_DRIVE = 10;
    public static final int REAR_LEFT_DRIVE = 40;

    public static final int FRONT_LEFT_STEER = 22;
    public static final int FRONT_RIGHT_STEER = 20;
    public static final int REAR_LEFT_STEER = 35;
    public static final int REAR_RIGHT_STEER = 50;
    
    public static final int REAR_LEFT_ENCODER = 45;
    public static final int FRONT_LEFT_ENCODER = 15;
    public static final int FRONT_RIGHT_ENCODER = 30;
    public static final int REAR_RIGHT_ENCODER = 60;

    public static final int PIGEONCAWWWWWWWWW = 13;


    // CAN Buses
    public static final CANBus kRoboRioCANBus = new CANBus("rio");
    public static final CANBus kCANivoreCANBus = new CANBus("main");

   
    public static final int kIntakePivot = 29; //SPARK

    public static final int kIntakeRollers = 5;
    public static final int kFloor = 12;
    public static final int kFeeder = 7;
    public static final int kShooterLeft = 11;
    public static final int kShooterRight = 17;


    public static final int kHanger = 28; //SPARK


    
    public static final int kAgitatorL = 43; //SPARK
    public static final int kAgitatorR = 47; //SPARK


}
