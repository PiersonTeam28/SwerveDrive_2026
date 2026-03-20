// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.HootAutoReplay;

//import frc.util.ProjectileSimulator;
//import frc.util.ProjectileSimulator.GeneratedLUT;
//import frc.util.ProjectileSimulator.LUTEntry;

import frc.robot.Constants;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
    private Command m_autonomousCommand;

    private final RobotContainer m_robotContainer;
    //private final ProjectileSimulator pSim;
    //public GeneratedLUT lut;
    //private ProjectileSimulator.SimParameters params;

    /* log and replay timestamp and joystick data */
    private final HootAutoReplay m_timeAndJoystickReplay = new HootAutoReplay()
        .withTimestampReplay()
        .withJoystickReplay();

    public Robot() {
        m_robotContainer = new RobotContainer();
        //params = new SimParameters(Constants.BALL_MASS, Constants.BALL_DIAMETER, Constants.DRAG_COEFF, Constants.MAGNUS_COEFF, Constants.AIR_DENSITY, Constants.EXIT_HEIGHT, Constants.WHEEL_DIAMETER, Constants.TARGET_HEIGHT, Constants.SLIP_FACTOR, Constants.LAUNCH_ANGLE, Constants.SIM_TIMESTEP, Constants.MIN_RPM, Constants.MAX_RPM, Constants.SEARCH_ITERS, Constants.MAX_SIM_TIME);
        //pSim = new ProjectileSimulator(new ProjectileSimulator.SimParameters(Constants.BALL_MASS, Constants.BALL_DIAMETER, Constants.DRAG_COEFF, Constants.MAGNUS_COEFF, Constants.AIR_DENSITY, Constants.EXIT_HEIGHT, Constants.WHEEL_DIAMETER, Constants.TARGET_HEIGHT, Constants.SLIP_FACTOR, Constants.LAUNCH_ANGLE, Constants.SIM_TIMESTEP, Constants.MIN_RPM, Constants.MAX_RPM, Constants.SEARCH_ITERS, Constants.MAX_SIM_TIME));

        //lut = pSim.generateLUT();

    //     for (LUTEntry entry : lut.entries()) {
    //     if (entry.reachable()) {
    //         System.out.printf("%.2fm -> %.0f RPM, %.3fs TOF%n",
    //             entry.distanceM(), entry.rpm(), entry.tof());
    //     }
    // }
    }

    

    @Override
    public void robotPeriodic() {
        m_timeAndJoystickReplay.update();
        CommandScheduler.getInstance().run(); 
    }

    @Override
    public void disabledInit() {}

    @Override
    public void disabledPeriodic() {}

    @Override
    public void disabledExit() {}

    @Override
    public void autonomousInit() {
        m_autonomousCommand = m_robotContainer.getAutonomousCommand();

        if (m_autonomousCommand != null) {
            CommandScheduler.getInstance().schedule(m_autonomousCommand);
        }
    }

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void autonomousExit() {}

    @Override
    public void teleopInit() {
        if (m_autonomousCommand != null) {
            CommandScheduler.getInstance().cancel(m_autonomousCommand);
        }
    }

    @Override
    public void teleopPeriodic() {}

    @Override
    public void teleopExit() {}

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {}

    @Override
    public void testExit() {}

    @Override
    public void simulationPeriodic() {}
}
