package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import swervelib.SwerveDrive;

/**
 * A classe `Robot` é configurada automaticamente para rodar esta classe, e chamar as funções correspondentes a cada modo, conforme descrito na documentação do `TimedRobot`.
 * Se você mudar o nome desta classe ou do pacote após a criação deste projeto, você deve atualizar o arquivo `build.gradle` no projeto.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand; // Declaração do comando autônomo
  SwerveDrive swerveDrive;

  private RobotContainer m_robotContainer; // Declaração do container do robô

  /**
   * Esta função é executada quando o robô é inicializado pela primeira vez e deve ser usada para qualquer código de inicialização.
   */
  @Override
  public void robotInit() {
    // Instancia o `RobotContainer`. Isso realizará todas as ligações de botões e colocará o chooser autônomo no dashboard.
    m_robotContainer = new RobotContainer();
  }

  /**
   * Esta função é chamada a cada 20 ms, independente do modo. Use isso para itens como diagnósticos que você deseja executar durante os modos desativado, autônomo, teleoperado e de teste.
   * Esta função é executada após as funções periódicas específicas do modo, mas antes da atualização integrada do LiveWindow e do SmartDashboard.
   */
  @Override
  public void robotPeriodic() {
    // Executa o `Scheduler`. Isso é responsável por verificar botões, adicionar comandos recém-agendados, executar comandos já agendados, remover comandos finalizados ou interrompidos e executar métodos periódicos do subsistema.
    // Isso deve ser chamado do bloco periódico do robô para que qualquer coisa no framework baseado em comando funcione.
    CommandScheduler.getInstance().run();
  }

  /** Esta função é chamada uma vez cada vez que o robô entra no modo Desativado. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** Este modo autônomo executa o comando autônomo selecionado pela classe `RobotContainer`. */
  @Override
  public void autonomousInit() {
    // Define os motores para o modo "brake" no início do autônomo
    m_robotContainer.setMotorBrake(true);
    // Obtém o comando autônomo do `RobotContainer`
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // Agendar o comando autônomo (exemplo)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** Esta função é chamada periodicamente durante o modo autônomo. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // Garante que o comando autônomo pare de ser executado quando o teleoperado começa. Se você quiser que o autônomo continue até ser interrompido por outro comando, remova esta linha ou comente-a.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
      
    }
  }

  /** Esta função é chamada periodicamente durante o controle do operador. */
  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    // Cancela todos os comandos em execução no início do modo de teste.
    CommandScheduler.getInstance().cancelAll();
  }

  /** Esta função é chamada periodicamente durante o modo de teste. */
  @Override
  public void testPeriodic() {}

  /** Esta função é chamada uma vez quando o robô é iniciado pela primeira vez. */
  @Override
  public void simulationInit() {}

  /** Esta função é chamada periodicamente durante a simulação. */
  @Override
  public void simulationPeriodic() {}
}
