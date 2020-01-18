/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Robot.h"

#include <iostream>

#include <frc/smartdashboard/SmartDashboard.h>

void Robot::RobotInit() {
  m_chooser.SetDefaultOption(kAutoNameDefault, kAutoNameDefault);
  m_chooser.AddOption(kAutoNameCustom, kAutoNameCustom);
  frc::SmartDashboard::PutData("Auto Modes", &m_chooser);

    frc::Color detectedColor = m_colorSensor.GetColor();m_colorMatcher.AddColorMatch(kBlueTarget);

    m_colorMatcher.AddColorMatch(kGreenTarget);

    m_colorMatcher.AddColorMatch(kRedTarget);

    m_colorMatcher.AddColorMatch(kYellowTarget);
}

/**
 * This function is called every robot packet, no matter the mode. Use
 * this for items like diagnostics that you want ran during disabled,
 * autonomous, teleoperated and test.
 *
 * <p> This runs after the mode specific periodic functions, but before
 * LiveWindow and SmartDashboard integrated updating.
 */
void Robot::RobotPeriodic() {}

/**
 * This autonomous (along with the chooser code above) shows how to select
 * between different autonomous modes using the dashboard. The sendable chooser
 * code works with the Java SmartDashboard. If you prefer the LabVIEW Dashboard,
 * remove all of the chooser code and uncomment the GetString line to get the
 * auto name from the text box below the Gyro.
 *
 * You can add additional auto modes by adding additional comparisons to the
 * if-else structure below with additional strings. If using the SendableChooser
 * make sure to add them to the chooser code above as well.
 */
void Robot::AutonomousInit() {
  m_autoSelected = m_chooser.GetSelected();
  // m_autoSelected = SmartDashboard::GetString("Auto Selector",
  //     kAutoNameDefault);
  std::cout << "Auto selected: " << m_autoSelected << std::endl;

  if (m_autoSelected == kAutoNameCustom) {
    // Custom Auto goes here
  } else {
    // Default Auto goes here
  }
}

void Robot::AutonomousPeriodic() {
  if (m_autoSelected == kAutoNameCustom) {
    // Custom Auto goes here
  } else {
    // Default Auto goes here
  }
}

void Robot::TeleopInit() {}

void Robot::TeleopPeriodic() {

  ColorPizza();
  Drive();
  std::shared_ptr<NetworkTable> table = nt::NetworkTableInstance::GetDefault().GetTable("limelight");
  double targetArea = table->GetNumber("ta",0.0);

  frc::SmartDashboard::PutNumber("Target Area", targetArea);

}

 void Robot::ColorPizza() {
    frc::Color detectedColor = m_colorSensor.GetColor();



    /**

      * Run the color match algorithm on our detected color

      */

    std::string colorString;

    double confidence = 0.0;

    frc::Color matchedColor = m_colorMatcher.MatchClosestColor(detectedColor, confidence);



    if (matchedColor == kBlueTarget) {

      colorString = "Blue";

    } else if (matchedColor == kRedTarget) {

      colorString = "Red";

    } else if (matchedColor == kGreenTarget) {

      colorString = "Green";

    } else if (matchedColor == kYellowTarget) {

      colorString = "Yellow";

    } else {

      colorString = "Unknown";

    }

    frc::SmartDashboard::PutString("Detected color", colorString);



  }

  void Robot::Drive() {
    LBd = JLeft.GetY();
    RBd = JRight.GetY();

    //Left deadZone
    if (LBd > deadZone){
      Ld = ((LBd - deadZone) * (1 / (1 - deadZone)));
    }
    else if (LBd < - deadZone){
      Ld = ((LBd + deadZone) * (-1 / (-1 + deadZone)));
    }
    else{
      Ld = 0;
    }

    //Right deadZone
    if (RBd > deadZone){
      Rd = ((RBd - deadZone) * (1 / (1 - deadZone)));
    }
    else if (RBd < - deadZone){
      Rd = ((RBd + deadZone) * (-1 / (-1 + deadZone)));
    }
    else{
      Rd = 0;
    }


    m_left.Set(Ld);
    m_right.Set(Rd);
  }


void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
