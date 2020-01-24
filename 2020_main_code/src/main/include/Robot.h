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

#include <frc/spark.h>
#include <cameraserver/CameraServer.h>

#include <frc/DigitalInput.h>
#include <wpi/raw_ostream.h>
#include <frc/encoder.h>
#include <frc/TimedRobot.h>
#include <frc/DoubleSolenoid.h>
#include <frc/Joystick.h>
#include <frc/Solenoid.h>

#include <frc/util/color.h>
#include "rev/ColorSensorV3.h"
#include "rev/ColorMatch.h"

#include <frc/SpeedControllerGroup.h>

#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableInstance.h"

#include <frc/Talon.h>
#include "rev/CANSparkMax.h"
#include "ctre/Phoenix.h"

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

  frc::Joystick JLeft{0};
  frc::Joystick JRight{1};

  frc::Joystick buttonBoard{2};
  
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


// drive train setup

  float deadZone = .25;
  rev::CANSparkMax frontLeftMotor1{0, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax frontLeftMotor2{1, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax frontRightMotor1{2, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax frontRightMotor2{3, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax rearLeftMotor1{4, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax rearLeftMotor2{5, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax rearRightMotor1{6, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax rearRightMotor2{7, rev::CANSparkMax::MotorType::kBrushless};

  rev::CANEncoder frontLeftEncoder = frontLeftMotor1.GetEncoder();
  rev::CANEncoder frontRightEncoder = frontRightMotor1.GetEncoder();
  rev::CANEncoder rearLeftEncoder = rearLeftMotor1.GetEncoder();
  rev::CANEncoder rearRightEncoder = rearRightMotor1.GetEncoder();

  frc::SpeedControllerGroup m_left{frontLeftMotor1, frontLeftMotor2, rearLeftMotor1, rearLeftMotor2};
  frc::SpeedControllerGroup m_right{frontRightMotor1, frontRightMotor2, rearRightMotor1, rearRightMotor2};


// Aiming------------------
  std::shared_ptr<NetworkTable> table = nt::NetworkTableInstance::GetDefault().GetTable("limelight");
 
  float Kp = -0.1f;
  float min_command = 0.05f;

 // Talon encoder-----------
  TalonSRX * wristMotor = new TalonSRX(1);


 private:
  frc::SendableChooser<std::string> m_chooser;
  const std::string kAutoNameDefault = "Default";
  const std::string kAutoNameCustom = "My Auto";
  std::string m_autoSelected;
  
  float LBd;
  float RBd;
  float Ld;
  float Rd;

 

  
};
