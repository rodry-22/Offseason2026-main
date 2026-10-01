// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import com.stzteam.mars.models.containers.IRobotContainer;
import com.stzteam.mars.operator.ControllerOI;
import com.stzteam.mars.test.TestRoutine;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.configuration.Manifest;
import frc.robot.configuration.Manifest.ControlsBuilder;
import frc.robot.configuration.Manifest.DrivetrainBuilder;
import frc.robot.configuration.Manifest.SuperstructureBuilder;
import frc.robot.configuration.bindings.DriverBindings;
import frc.robot.configuration.bindings.OperatorBindings;
import frc.robot.modules.superstructure.composite.Superstructure;
import frc.robot.modules.superstructure.modules.DumperModule.Dumper;
import frc.robot.modules.superstructure.modules.IndexerModule.Indexer;
import frc.robot.modules.superstructure.modules.IntakeModule.Intake;
import frc.robot.modules.swerve.ChassisTuningDashboard;
import frc.robot.modules.swerve.CommandSwerveDrivetrain;
import frc.tests.EmptyTest;
import frc.robot.modules.superstructure.modules.FlywheelsModule.flywheels;

public class RobotContainer implements IRobotContainer{

  public final ControllerOI driver;
  public final ControllerOI operator;
  public final CommandSwerveDrivetrain drivetrain;

  public final Dumper dumper;
  public final Indexer indexer;
  public final flywheels flywheelsIntake;
  public final flywheels flywheelsShooter;
  public final Intake intake;


  public final Superstructure superstructure;

  private final SendableChooser<Command> autoChooser;

  public RobotContainer() {

    

    this.driver = ControlsBuilder.buildDriver();
    this.operator = ControlsBuilder.buildOperator();

    this.drivetrain = DrivetrainBuilder.buildModule();

    DriverBindings.create(drivetrain, driver).bind();

    ChassisTuningDashboard.publish(drivetrain);

    if (AutoBuilder.isConfigured()) {
      this.autoChooser = AutoBuilder.buildAutoChooser();
    } else {
      this.autoChooser = new SendableChooser<>();
      this.autoChooser.setDefaultOption("None", Commands.none());
    }
    SmartDashboard.putData("Auto Chooser", autoChooser);

    this.dumper = Manifest.buildDumper();
    this.indexer = Manifest.buildIndexer();
    this.flywheelsIntake = Manifest.buildFlywheelsIntake();
    this.flywheelsShooter = Manifest.buildFlywheelsShooter();
    this.intake = Manifest.buildIntake();


    this.superstructure = SuperstructureBuilder.superBuild(
      this.dumper,
      this.intake,
      this.indexer,
      this.flywheelsIntake,
      this.flywheelsShooter
    );

    OperatorBindings.create(operator, superstructure).bind();

  }

  @Override
  public void updateNodes() {

  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  @Override
  public TestRoutine getTestRoutine() {
    return new EmptyTest();
  }


}

