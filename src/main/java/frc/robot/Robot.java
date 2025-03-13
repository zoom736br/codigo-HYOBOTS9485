// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.cameraserver.CameraServer;
import com.revrobotics.spark.SparkMaxAlternateEncoder;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;
  Constants Constants = new Constants();

//variaveis do elevador
double angulo_elevador;
double setpoint_elevador;
double output_elevador;
double KpElev = 0.01;
double KiElev = 0;
double KdElev = 0;
boolean seguranca_elevador = false;
boolean zerar_elevador=false;
double tolerancia_elevador=30.0;


//variaveis do intake
SparkMax motor_intake = new SparkMax(17, SparkMax.MotorType.kBrushless);
SparkMax motor_da_bola = new SparkMax(18, SparkMax.MotorType.kBrushless);
boolean trava_motor_bola=false;
boolean tem_bola_intake=false;
double speed_intake;
DutyCycleEncoder encoder_intake = new DutyCycleEncoder(1);
double angulo_intake;
double Kp_intake = 0.01;
double Ki_intake = 0;
double Kd_intake= 0;
double setpoint_intake=60;
double output_intake;
  
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
//variaveis de log
double lasttick;

//variaveis para maquina de estado
boolean hablita_maquina_estados = false;
int estado_atual= 0;
double setpoint_1_elevador = 0;
double setpoint_1_intake = 0;
double setpoint_2_elevador = 0;
double setpoint_2_intake = 0;
double setpoint_3_elevador = 0;
double setpoint_3_intake = 0;


//declaração dos componetes
XboxController joystick = new XboxController(1);
SparkMax Elevador14 = new SparkMax(14, SparkMax.MotorType.kBrushless);
SparkMax Elevador15 = new SparkMax(15, SparkMax.MotorType.kBrushless);
Encoder encoderElev = new Encoder(6, 8);
Timer timer = new Timer();
DigitalInput fimDeCurso_baixo = new DigitalInput(2);
PIDController PIDElevador = new PIDController(KpElev, KiElev, KdElev);
DigitalInput fimDeCurso_Cima = new DigitalInput(3);
PIDController PID_inatake = new PIDController(Kp_intake, Ki_intake, Kd_intake);
DigitalInput fim_de_curso_Coral = new DigitalInput(0);
SparkMax motor_cliber = new SparkMax(16, SparkMax.MotorType.kBrushless);

  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
    encoderElev.setDistancePerPulse(360.0/2048.0);
    encoderElev.setReverseDirection(true);
    encoder_intake.setDutyCycleRange(0, 360);
    System.out.println("Robo iniciado\n");
    PIDElevador.setTolerance(tolerancia_elevador);
    PID_inatake.setTolerance(4.0);
    setpoint_elevador = 0.0;
    setpoint_intake = 58.0;
    CameraServer.startAutomaticCapture();
    
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_robotContainer.setMotorBrake(true);
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }

    timer.reset();
    timer.start();
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() 
  {
    if (timer.get() >= 2.0 && timer.get() < 2.4) 
      setpoint_intake = 71.0;
    if(timer.get() >= 2.4 && timer.get() < 3.4){
      setpoint_elevador = 7440.0;
    }
    if (timer.get() >= 3.4&&timer.get()<4.4)
    motor_da_bola.set(0.3);
    if (timer.get()>4.4)
    motor_da_bola.set(0);
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

    timer.reset();
    timer.start();
    lasttick = 0.0;
    System.out.println("Teleop iniciado\n");

    encoderElev.reset();
    setpoint_elevador = 0.0;
    setpoint_intake = 58.0;
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic()
  {
    //ler joystic e ajustar setpoints
    if(joystick.getLeftY() > 0.5 || joystick.getLeftY() < -0.5){ 
      setpoint_elevador= setpoint_elevador-10*joystick.getLeftY() ;
      hablita_maquina_estados = false;
    }
    if(joystick.getRightY() > 0.5 || joystick.getRightY() < -0.5){
      setpoint_intake = setpoint_intake + 1.0 * joystick.getRightY();
      hablita_maquina_estados  = false;
    }

    if(joystick.getPOV() == 0){
      motor_cliber.set(0.7);
    }
    if(joystick.getPOV() == 180){
      motor_cliber.set(-0.7);
    }
    if(joystick.getPOV() == -1.0){
      motor_cliber.set(0.0);
    }

    if(hablita_maquina_estados ==false)
    {
      if(joystick.getAButtonPressed()){//posição que pega o coral

        setpoint_1_elevador=setpoint_elevador;
        setpoint_1_intake = 68.0;
        setpoint_2_elevador = 0;
        setpoint_2_intake = 68.0;
        setpoint_3_elevador = 0.0;
        setpoint_3_intake = 55.0;
        hablita_maquina_estados = true;
        estado_atual = 1;

        motor_da_bola.set(0.2);
        trava_motor_bola=true;
      }
      if(joystick.getBButtonPressed()){//L2
        setpoint_1_elevador=0.0;
        setpoint_1_intake = 72.0;
        setpoint_2_elevador = 210.0;
        setpoint_2_intake = 72.0;
        setpoint_3_elevador = 210.0;
        setpoint_3_intake = 72.0;
        hablita_maquina_estados = true;
        estado_atual = 1;
      }
      if(joystick.getXButtonPressed()){//L4

        setpoint_1_elevador=0;
        setpoint_1_intake = 72.0;
        setpoint_2_elevador = 1480.0;
        setpoint_2_intake = 72.0;
        setpoint_3_elevador = 1480.0;
        setpoint_3_intake = 92.0;
        hablita_maquina_estados = true;
        estado_atual = 1;

      }
      if(joystick.getYButtonPressed()){//L3

        setpoint_1_elevador=0;
        setpoint_1_intake = 72.0;
        setpoint_2_elevador = 769.0;
        setpoint_2_intake = 72.0;
        setpoint_3_elevador = 769.0;
        setpoint_3_intake = 72.0;
        hablita_maquina_estados = true;
        estado_atual = 1;

      }
      if(joystick.getRightBumperButton() && !joystick.getLeftBumperButton()){//ir para L2 da bola
        setpoint_1_elevador=624.0;
        setpoint_1_intake = 225.0;
        setpoint_2_elevador = 624.0;
        setpoint_2_intake = 225.0;
        setpoint_3_elevador = 624.0;
        setpoint_3_intake = 225.0;
        hablita_maquina_estados = true;
        estado_atual = 1;
      }
      if(!joystick.getRightBumperButton() && joystick.getLeftBumperButton()){//ir para L3 da bola
        setpoint_1_elevador=1107.0;
        setpoint_1_intake = 225.0;
        setpoint_2_elevador = 1107.0;
        setpoint_2_intake = 225.0;
        setpoint_3_elevador = 1107.0;
        setpoint_3_intake = 225.0;
        hablita_maquina_estados = true;
        estado_atual = 1;
    }
    if(joystick.getRightBumperButton() && joystick.getLeftBumperButton()){//processador
      setpoint_1_elevador=0.0;
      setpoint_1_intake = 225.0;
      setpoint_2_elevador = 0.0;
      setpoint_2_intake = 225.0;
      setpoint_3_elevador = 0.0;
      setpoint_3_intake = 225.0;
      hablita_maquina_estados = true;
      estado_atual = 1;
  }
    }
    
    
    /////////////////////////////calcular e controlar elevador////////////////////////////////////////////////////////////////
    angulo_elevador = encoderElev.getDistance();
    output_elevador = PIDElevador.calculate(angulo_elevador, setpoint_elevador);
  
    //ler fim de curso de cima
    if(fimDeCurso_Cima.get())
    {
      if(output_elevador>0)output_elevador=0;
      if(setpoint_elevador>1480)setpoint_elevador=1480.0;
    }
    
    //ler fim de curso de baixo
    if(fimDeCurso_baixo.get())
    {
      if(output_elevador<0)output_elevador=0;
      if(setpoint_elevador<0.0)setpoint_elevador=0.0;
      if(zerar_elevador==true&&(angulo_elevador>tolerancia_elevador||angulo_elevador<-1*tolerancia_elevador))
      {
        //encoderElev.reset();   
        zerar_elevador=false;  
      }
    }
    else
    {
      zerar_elevador=true;
    }
  
    //segurança do elevador em relação ao intake
    if(seguranca_elevador == true)output_elevador=0.0;
    
    Elevador14.set(output_elevador);
    Elevador15.set(-output_elevador );
    //////////////////////////////////////////////// fim do código do elevador //////////////////////////////////////////////////////
  
    //////////////////////////////////////// inicio do codigo do intake ////////////////////////////////////////////////////////////
    angulo_intake = encoder_intake.get() * 360.0;
    output_intake = PID_inatake.calculate(angulo_intake, setpoint_intake);
  
      //ler posição minima do intake
      if(angulo_intake < 55)
      {
        if(output_intake<0)output_intake=0.0;
        if(setpoint_intake<55)setpoint_intake = 55.0;
      }
  
      //ler posição maxima do intake 
      if(angulo_intake > 230)
      {
        if(output_intake>0)output_intake=0.0;
        if(setpoint_intake> 230)setpoint_intake = 230.0;
      }

    
    motor_intake.set(output_intake);
  
    if(angulo_intake < 64)seguranca_elevador = true;
    else{seguranca_elevador = false;}
    /////////////////////////////////////////// fim do código do intake //////////////////////////////////////////////////////////
    
  
  
    ///////////////////////////////////////inicio do código do  motor da bola e do coral ////////////////////////////////////////////
    //if(motor_da_bola.getOutputCurrent()>40)tem_bola_intake=true;
    //if(tem_bola_intake)motor_da_bola.set(0.0);
    
    if(trava_motor_bola==true) motor_da_bola.set(0.2);
    if(!fim_de_curso_Coral.get()&&trava_motor_bola==true){
      motor_da_bola.set(0);
      trava_motor_bola=false;
    }
    if((joystick.getRightTriggerAxis()-joystick.getLeftTriggerAxis())>0.8)trava_motor_bola=false;
    if((joystick.getRightTriggerAxis()-joystick.getLeftTriggerAxis())<-0.8)
    {
      trava_motor_bola=false;
      //tem_bola_intake=false;
    }
    if(trava_motor_bola==false)motor_da_bola.set((joystick.getRightTriggerAxis()-joystick.getLeftTriggerAxis()) * 0.5);
    
      
    ///////////////////////////////////////fim do código da bola e do coral ////////////////////////////////////////////
    
    ////////////////////////////////// Inicio do código da maquina de estados////////////////////////////////////////
    if(hablita_maquina_estados==true)
    {
      
      switch(estado_atual)
      {
        case 1:
        {
          setpoint_elevador=setpoint_1_elevador;
          setpoint_intake=setpoint_1_intake;
          if(PIDElevador.atSetpoint() && PID_inatake.atSetpoint())
          {
            estado_atual = 2;
            setpoint_elevador=setpoint_2_elevador;
            setpoint_intake=setpoint_2_intake;
          }
        }break;
        case 2:
        {
          
          if(PIDElevador.atSetpoint() && PID_inatake.atSetpoint())
          {
            estado_atual = 3;
            setpoint_elevador=setpoint_3_elevador;
            setpoint_intake=setpoint_3_intake;
          
          }
        }break;
        case 3:
        {
          if(PIDElevador.atSetpoint() && PID_inatake.atSetpoint())
          {
            hablita_maquina_estados = false;
          }
        }break;
      }
      

    }

    ////////////////////////////////// Fim do código da maquina de estados////////////////////////////////////////
  
    //imprimir os logs
    if(timer.get() > (lasttick + 2.0))
    {
      lasttick = timer.get();
      System.out.printf("Set_point elevador: %2f\n", setpoint_elevador);
      
      System.out.printf("Angulo_elevador: %2f\n", angulo_elevador);
      //System.out.printf("setpoint intake: %2f \n", setpoint_intake);
      //System.out.printf("angulo do intake: %2f\n", angulo_intake);
      //System.out.printf("Corrente motor bola: %2f\n", motor_da_bola.getOutputCurrent());
    }
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}

}