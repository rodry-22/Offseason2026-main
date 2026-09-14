// Copyright (c) 2026 STZ Robotics
// Open Source Software; you can modify and/or share it under the terms of
// the MIT license file in the root directory of this project.

package frc.robot.configuration.bindings;

import com.stzteam.mars.models.containers.Binding;
import com.stzteam.mars.operator.ControllerOI;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.configuration.constants.moduleconstants.SwerveConstants;
import frc.robot.modules.swerve.CommandSwerveDrivetrain;
import frc.robot.modules.swerve.SwerveRequestFactory;

public class DriverBindings implements Binding {

  private final CommandSwerveDrivetrain drivetrain;

  private final ControllerOI driver;

  public double alllianceFactor;

  private DriverBindings(CommandSwerveDrivetrain drivetrain, ControllerOI driver) {
    this.drivetrain = drivetrain;
    this.driver = driver;
  }

  public static DriverBindings create(CommandSwerveDrivetrain drivetrain, ControllerOI driver) {
    return new DriverBindings(drivetrain, driver);
  }

  @Override
  public void bind() {
    var driverLeftStick = driver.getLeftStick();
    var driverRightStick = driver.getRightStick();
    // var driverDPad = driver.getDPadTriggers();
    // var driverSystem = driver.getSystemTriggers();
    var driverBumpers = driver.getBumpers();
    var driverButtons = driver.getActionButtons();

    drivetrain.setDefaultCommand(
        drivetrain.applyRequest(
            () ->
                SwerveRequestFactory.driveFieldCentric() // TODO: MODO ACTUAL - ROJO
                    .withVelocityX(
                        driverLeftStick.y().getAsDouble() // TODO: Rojo Positivo - Azul Negativo
                            * SwerveConstants.MaxSpeed
                            * (driverBumpers.right().getAsBoolean() ? 0.5 : 1.0)
                            * (driverBumpers.left().getAsBoolean() ? 0.25 : 1.0))
                    .withVelocityY(
                        driverLeftStick.x().getAsDouble() // TODO: Rojo Positivo - Azul Negativo
                            * SwerveConstants.MaxSpeed
                            * (driverBumpers.right().getAsBoolean() ? 0.5 : 1.0)
                            * (driverBumpers.left().getAsBoolean() ? 0.25 : 1.0))
                    .withRotationalRate(
                        -driverRightStick.x().getAsDouble() * SwerveConstants.MaxAngularRate)));

    driverButtons.top().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

    driverButtons.bottom().whileTrue(drivetrain.applyRequest(() -> SwerveRequestFactory.brake()));

  }
}
