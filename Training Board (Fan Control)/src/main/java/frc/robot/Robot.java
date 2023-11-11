// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.shuffleboard.*;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import javax.swing.plaf.TreeUI;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends TimedRobot {
  private final PWMSparkMax m_leftDrive = new PWMSparkMax(0);
  private final PWMSparkMax m_rightDrive = new PWMSparkMax(1);
  private final DifferentialDrive m_robotDrive = new DifferentialDrive(m_leftDrive, m_rightDrive);
  private final XboxController m_controller = new XboxController(0);
  private final Timer m_timer = new Timer();
  private final DigitalOutput digitaloutput0 = new DigitalOutput(0);
  private final DigitalOutput digitaloutput1 = new DigitalOutput(1);
  private final DigitalOutput digitaloutput2 = new DigitalOutput(2);
  private final DigitalOutput digitaloutput3 = new DigitalOutput(3);
  private final VictorSPX fan = new VictorSPX(1);
  private final SendableChooser<Boolean> lightswitch = new SendableChooser<>();
  private final SendableChooser<Boolean> invertfan = new SendableChooser<>();
  boolean input = false;
  boolean faninvert = false;
  double speed = 0;
  int count = 0;
  //ashduihauduwa
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    m_rightDrive.setInverted(true);
    SmartDashboard.putNumber("speed", speed);
    SmartDashboard.putNumber("speed1", speed);
  }

  /** This function is run once each time the robot enters autonomous mode. */
  @Override
  public void autonomousInit() {
    m_timer.restart();
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    // Drive for 2 seconds
    /*if (m_timer.get() < 2.0) {
      // Drive forwards half speed, make sure to turn input squaring off
      m_robotDrive.arcadeDrive(0.5, 0.0, false);
    } else {
      m_robotDrive.stopMotor(); // stop robot
    }*/
  }

  /** This function is called once each time the robot enters teleoperated mode. */
  @Override
  public void teleopInit() {}

  /** This function is called periodically during teleoperated mode. */
  @Override
  public void teleopPeriodic() {
    //m_robotDrive.arcadeDrive(-m_controller.getLeftY(), -m_controller.getRightX());

    lightswitch.setDefaultOption("off", false);
    lightswitch.addOption("on", true);

    invertfan.setDefaultOption("default", false);
    invertfan.addOption("invert", true);

    SmartDashboard.putData("lights", lightswitch);
    SmartDashboard.putData("fan direction", invertfan);
    SmartDashboard.putBoolean("input", input);

    input = lightswitch.getSelected();
    faninvert = invertfan.getSelected();
    speed = SmartDashboard.getNumber("speed", 0);
    SmartDashboard.putNumber("speed1", speed);

    fan.setInverted(faninvert);
    fan.set(VictorSPXControlMode.PercentOutput, speed);

    if (!input) {
      digitaloutput0.set(false);
      digitaloutput1.set(false);
      digitaloutput2.set(false);
      digitaloutput3.set(false);
      SmartDashboard.putBoolean("DO0", digitaloutput0.get());
      SmartDashboard.putBoolean("DO1", digitaloutput1.get());
      SmartDashboard.putBoolean("DO2", digitaloutput2.get());
      SmartDashboard.putBoolean("DO3", digitaloutput3.get());
      count = 0;
    }

    if (input) {
      input = lightswitch.getSelected();
      SmartDashboard.putBoolean("input", input);
      count += 1;
      if(count % 10 <= 5){
        digitaloutput0.set(true);
      }
      else{
        digitaloutput0.set(false);
      }
      if(count % 20 <= 10){
        digitaloutput1.set(true);
      }
      else{
        digitaloutput1.set(false);
      }
      if(count % 40 <= 20){
        digitaloutput2.set(true);
      }
      else{
        digitaloutput2.set(false);
      }
      if(count % 80 <= 40){
        digitaloutput3.set(true);
      }
      else{
        digitaloutput3.set(false);
      }
      SmartDashboard.putBoolean("DO0", digitaloutput0.get());
      SmartDashboard.putBoolean("DO1", digitaloutput1.get());
      SmartDashboard.putBoolean("DO2", digitaloutput2.get());
      SmartDashboard.putBoolean("DO3", digitaloutput3.get());
    }
  }

  /** This function is called once each time the robot enters test mode. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
}
