package frc.robot.commands;

import frc.robot.Constants;

import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.LimelightSubsystem;

import frc.robot.subsystems.SwerveSubsystem;
import swervelib.SwerveDrive;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;

/** An example command that uses an example subsystem. */
public class Limelight extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  SwerveSubsystem swerve;
  LimelightSubsystem LimeLight;
  double drive;
  double turn;
  double velocidade;
  boolean definirModos;
  boolean Limelight10 = true;
  boolean ligado;

  
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
    ligado = false;

    drive = 0;
    turn = 0;

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SwerveDrive swerveDrive = swerve.swerveDrive;
   
      if(LimeLight.hasTarget()  ==  true  ){
      System.out.println("Deu Certo!!!!!!!!!!!!!!!!!!!!!!");
      swerveDrive.drive( new Translation2d(0.0, 0.0), 0.0,true, false );
      ligado = true;
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
    return ligado;
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







