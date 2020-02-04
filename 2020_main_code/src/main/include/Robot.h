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
  frc::DigitalInput upSwitch{1};
  frc::DigitalInput downSwitch{2};
  
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
 
  //rev::CANSparkMax frontLeftMotor1{1, rev::CANSparkMax::MotorType::kBrushless};
  //rev::CANSparkMax frontLeftMotor2{2, rev::CANSparkMax::MotorType::kBrushless};
  //rev::CANSparkMax frontRightMotor1{3, rev::CANSparkMax::MotorType::kBrushless};
  //rev::CANSparkMax frontRightMotor2{4, rev::CANSparkMax::MotorType::kBrushless};
  /*rev::CANSparkMax rearLeftMotor1{5, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax rearLeftMotor2{6, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax rearRightMotor1{7, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax rearRightMotor2{8, rev::CANSparkMax::MotorType::kBrushless};


  rev::CANEncoder frontLeftEncoder = frontLeftMotor1.GetEncoder();
  rev::CANEncoder frontRightEncoder = frontRightMotor1.GetEncoder();
  rev::CANEncoder rearLeftEncoder = rearLeftMotor1.GetEncoder();
  rev::CANEncoder rearRightEncoder = rearRightMotor1.GetEncoder();

  frc::SpeedControllerGroup m_left{frontLeftMotor1, frontLeftMotor2, rearLeftMotor1, rearLeftMotor2};
  frc::SpeedControllerGroup m_right{frontRightMotor1, frontRightMotor2, rearRightMotor1, rearRightMotor2};
*/

// Aiming------------------
  std::shared_ptr<NetworkTable> table = nt::NetworkTableInstance::GetDefault().GetTable("limelight");
 
  float Kp = -0.1f;
  float min_command = 0.05f;

 // Talon encoder shooter-----------
  TalonFX * shoot1 = new TalonFX(9);
  TalonFX * shoot2 = new TalonFX(12);
  double speed = .10;
  double Ospeed;

  const float WRIST_kP = 0.001;
  const float WRIST_kI = 0;
  const float WRIST_kD = 0;
  const float WRIST_kF = 0;

  const int kPIDLoopIdx = 0;

  const int kTimeoutMs = 30;

  const float  wrist_low = 0;
  const float  wrist_pickup = 0;
  const float  wrist_mid = 4096 / 4;
  const float  wrist_high = 0.15 * 4096;

  const int kSlotIdx = 0;

// Intake-------------------
  VictorSPX * intakeMove = new VictorSPX(10);
  TalonSRX * intakeRun = new TalonSRX(13);
  
// Climber------------------
  rev::CANSparkMax climber{14, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANPIDController climbPID = climber.GetPIDController();

  double pos;
  double posUp = 0;
  double posDown = 0;

  double kPe = 0.2, 
    kI = 0, 
    kD = 1, 
    kIz = 0, 
    kFF = 0, 
    kMaxOutput = 1, 
    kMinOutput = -1;

// Index-------------------
  TalonSRX * index = new TalonSRX(11);




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
