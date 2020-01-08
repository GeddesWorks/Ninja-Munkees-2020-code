/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include "frc/WPILib.h"
#include <string>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc/smartdashboard/SmartDashboard.h>
//#include <frc/Talon.h>
#include <frc/spark.h>
#include <cameraserver/CameraServer.h>
//#include "rev/CANSparkMax.h"
#include <frc/DigitalInput.h>
#include <wpi/raw_ostream.h>
#include <frc/encoder.h>
//#include "ctre/Phoenix.h"
#include <frc/TimedRobot.h>

#include <frc/DoubleSolenoid.h>
#include <frc/Joystick.h>
#include <frc/Solenoid.h>

#include <frc/util/color.h>
#include "rev/ColorSensorV3.h"
#include "rev/ColorMatch.h"


class Robot : public frc::TimedRobot {
 public:
  void RobotInit() override;
  void RobotPeriodic() override;
  void AutonomousInit() override;
  void AutonomousPeriodic() override;
  void TeleopInit() override;
  void TeleopPeriodic() override;
  void TestPeriodic() override;
// Robot scripts setup

  void Drive();
  void Shooter();
  void Intake();
  void Climber();
  void ColorPizza();


  // Input

  frc::Joystick mainJoystick{0};

  frc::Joystick buttonBoard{1};
  
  static constexpr auto i2cPort = frc::I2C::Port::kOnboard;
  rev::ColorSensorV3 m_colorSensor{i2cPort};
  rev::ColorMatch m_colorMatcher;
  /**

   * Note: Any example colors should be calibrated as the user needs, these

   * are here as a basic example.

   */

  static constexpr frc::Color kBlueTarget = frc::Color(0.143, 0.427, 0.429);

  static constexpr frc::Color kGreenTarget = frc::Color(0.197, 0.561, 0.240);

  static constexpr frc::Color kRedTarget = frc::Color(0.561, 0.232, 0.114);

  static constexpr frc::Color kYellowTarget = frc::Color(0.361, 0.524, 0.113);










 private:
  frc::SendableChooser<std::string> m_chooser;
  const std::string kAutoNameDefault = "Default";
  const std::string kAutoNameCustom = "My Auto";
  std::string m_autoSelected;
};