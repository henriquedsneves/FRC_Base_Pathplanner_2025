package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel;






import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {

  SparkMax leftMotor = new SparkMax(Constants.ShooterMotors.rightMotor, SparkLowLevel.MotorType.kBrushless);
  SparkMax rightMotor = new SparkMax(Constants.ShooterMotors.leftMotor, SparkLowLevel.MotorType.kBrushless);
  
  

  RelativeEncoder leftEncoder = leftMotor.getEncoder();
  RelativeEncoder rightEncoder = rightMotor.getEncoder();

  DifferentialDrive differentialDrive = new DifferentialDrive(leftMotor, rightMotor);

  

  public Shooter() {
 

    leftEncoder.setPosition(0);
    rightEncoder.setPosition(0);

    leftEncoder = leftMotor.getEncoder();
    rightEncoder = rightMotor.getEncoder();

    leftMotor.setInverted(false);
    rightMotor.setInverted(true);


  }


  @Override
  public void periodic() {
    System.out.println(leftEncoder.getPosition());
    System.out.println(rightEncoder.getPosition());
  }

  public void arcadeMode(double drive, double turn){
    differentialDrive.arcadeDrive(drive, turn);
  }

  public void tankmode(double left, double right){
    differentialDrive.tankDrive(left, right);
  }
  public void stop(){
    differentialDrive.stopMotor(); 
  }
  public double ValorEncoder(){
    return leftEncoder.getPosition();
  }
  public void SetEncoder(){

    leftEncoder.setPosition(0);
  }
  public Rotation2d getHeading(){
    return Rotation2d.fromDegrees(ValorEncoder());
  }
}
