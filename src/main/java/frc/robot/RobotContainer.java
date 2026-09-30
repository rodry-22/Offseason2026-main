// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import com.stzteam.mars.models.containers.IRobotContainer;
import com.stzteam.mars.operator.ControllerOI;
import com.stzteam.mars.test.TestRoutine;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.configuration.Manifest;
import frc.robot.configuration.Manifest.ControlsBuilder;
import frc.robot.configuration.Manifest.DrivetrainBuilder;
import frc.robot.configuration.Manifest.SuperstructureBuilder;
import frc.robot.configuration.bindings.DriverBindings;
import frc.robot.modules.superstructure.composite.Superstructure;
import frc.robot.modules.superstructure.modules.DumperModule.Dumper;
import frc.robot.modules.superstructure.modules.IndexerModule.Indexer;
import frc.robot.modules.swerve.CommandSwerveDrivetrain;
import frc.tests.EmptyTest;
import frc.robot.modules.superstructure.modules.FlywheelsModule.flywheels;

public class RobotContainer implements IRobotContainer{

  public final ControllerOI driver;
  public final CommandSwerveDrivetrain drivetrain;

  public final Dumper dumper;
  public final Indexer indexer;
  public final flywheels flywheels;

  public final Superstructure superstructure;

  public RobotContainer() {

    

    this.driver = ControlsBuilder.buildDriver();

    this.drivetrain = DrivetrainBuilder.buildModule();

    DriverBindings.create(drivetrain, driver).bind();

    this.dumper = Manifest.buildDumper();
    this.indexer = Manifest.buildIndexer();
    this.flywheels = Manifest.buildFlywheels();

    this.superstructure = SuperstructureBuilder.superBuild(
      this.dumper,
      this.indexer,
      this.flywheels
    );

  }

  @Override
  public void updateNodes() {

  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }

  @Override
  public TestRoutine getTestRoutine() {
    return new EmptyTest();
  }


}

