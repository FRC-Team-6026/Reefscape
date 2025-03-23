// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;

import org.littletonrobotics.urcl.URCL;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  public Robot() {
    m_robotContainer = new RobotContainer();

    CameraServer.startAutomaticCapture();

    // SysId - Enabling logging. This method is specific to our Rev motors.
    // (Thank you to team 6328 for this logger!)
    DataLogManager.start();
    HashMap<Integer,String> aliases = new HashMap<Integer,String>();
    // Map of CanID's to convenient names
    aliases.put(1, "FL Wheel");
    aliases.put(3, "FR wheel");
    aliases.put(5, "RL wheel");
    aliases.put(7, "RR wheel");
    aliases.put(Constants.Setup.elevatorSpark1, "Elevator Motor 1");
    URCL.start(aliases);

    // Limelight port.
    for (int port = 5800; port <= 5809; port++) {
      PortForwarder.add(port, "limelight.local", port);
    }
  }

  @Override
  public void robotPeriodic() {
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
    m_robotContainer.autoInit();
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    m_robotContainer.teleopInit();
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() { m_robotContainer.teleopExit(); }

  @Override
  public void testInit() {
    // SysId - set up SysID tests. Preferably, set up tests on one or more buttons so they can be interrupted easily.
    CommandScheduler.getInstance().cancelAll();
    m_robotContainer.testInit();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {
    CommandScheduler.getInstance().cancelAll();
  }
}
