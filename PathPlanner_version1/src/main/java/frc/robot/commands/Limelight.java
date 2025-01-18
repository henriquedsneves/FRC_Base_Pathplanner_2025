package frc.robot.commands;

import frc.robot.Constants;

import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.LimelightSubsystem;

import frc.robot.subsystems.Elevador;
import swervelib.SwerveDrive;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
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
 
  boolean definirModos;
  boolean Limelight10 = true;
  boolean ligado;
  SwerveSubsystem swerveSubsystem;
  LimelightSubsystem limeLightSubsystem;
  boolean finalizar;
  double velocidadeAlinhamento = 0.12;
  SwerveDrive swerveDrive;
  SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(0.01, 0, 0);
  PIDController rotationPID = new PIDController(0.01, 0.00, 0.00);
  double velocidade = 0.2;
  double output;
  
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  

public Limelight(LimelightSubsystem limeLight, SwerveSubsystem swerve) {
  this.limeLightSubsystem = limeLight;
  this.swerveSubsystem = swerve;
  addRequirements(limeLightSubsystem, swerveSubsystem);
  this.swerveDrive = swerveSubsystem.swerveDrive;
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

   
      output = rotationPID.calculate(limeLightSubsystem.getTx(), 20);
 feedforward.calculate(velocidade);
 velocidadeAlinhamento = output + velocidade;
 if(limeLightSubsystem.hasTarget()){
   if (limeLightSubsystem.getTx()>21){
   swerveDrive.drive(new Translation2d(0,velocidadeAlinhamento * swerveDrive.getMaximumVelocity()), 0, false, false);
 } else if(limeLightSubsystem.getTx()<19) {
   swerveDrive.drive(new Translation2d(0,-velocidadeAlinhamento * swerveDrive.getMaximumVelocity()), 0, false, false);
 } else if (limeLightSubsystem.getTx()==20) {
   swerveDrive.drive(new Translation2d(0,0), 0, false, false);
   finalizar = true;
   swerveDrive.updateOdometry();
   
 } else {
   finalizar = true;
   swerveDrive.updateOdometry();
 }
 } else {
   finalizar = true;
   swerveDrive.updateOdometry();
 }
}
    
  

  
   
  

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    finalizar = false;

  }
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
     return finalizar ;
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








