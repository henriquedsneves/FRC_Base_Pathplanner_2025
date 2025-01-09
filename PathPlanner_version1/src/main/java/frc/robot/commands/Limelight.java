package frc.robot.commands;

import frc.robot.Constants;

import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.LimelightSubsystem;

import frc.robot.subsystems.SwerveSubsystem;

import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class Limelight extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  SwerveSubsystem swerve;
  LimelightSubsystem LimeLight;
  double drive;
  double turn;
  double velocidade;
  boolean definirModos;
  boolean Limelight10 = false;

  
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  

public Limelight(LimelightSubsystem limeLight, SwerveSubsystem swerve) {
  this.LimeLight = limeLight;
  this.swerve = swerve;
  addRequirements(LimeLight, swerve); // Declara dependências
}

  // Called when the command is initially scheduled.
  @Override
  public void initialize()
   {

    drive = 0;
    turn = 0;

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
   
     if(LimeLight.hasTarget()  ==  true  ){
      System.out.println("Deu Certo!!!!!!!!!!!!!!!!!!!!!!");
    }else{
     System.out.println("Deu erradoooooooooooooooo");
    }
    
  }

  
   
  

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
  /*public void stopLime(){{
    System.out.println();
    if(LimeLight.getTx() > -3 ){
      System.out.println("Deu Certo!!!!!!!!!!!!!!!!!!!!!!");
    }else{
     System.out.println("Deu erradoooooooooooooooo");
    }
    
  }}*/
  
} 







