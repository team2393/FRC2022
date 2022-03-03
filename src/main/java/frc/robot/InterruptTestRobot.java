// Copyright (c) Team 2393, FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import edu.wpi.first.wpilibj.AsynchronousInterrupt;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.TimedRobot;

/** Robot to test interrupt */
public class InterruptTestRobot extends TimedRobot
{
    private final DigitalInput input = new DigitalInput(3);
    private final AsynchronousInterrupt interrupt = new AsynchronousInterrupt(input, this::irq);
    
    private void irq(Boolean rise, Boolean fall)
    {
        if (rise)
            System.out.println("Interrupt: rise ^^^^^^^^^^^^^^");
        if (fall)
            System.out.println("Interrupt: fall \\/\\/\\/\\/\\/\\/\\/\\/");
    }

    @Override
    public void robotInit()
    {
        // Print something that allows us to see on the roboRio what's been uploaded
        System.out.println("***** Team 2393 Interrupt Test *****");
        interrupt.enable();
    }

    @Override
    public void robotPeriodic()
    {
    }

    @Override
    public void teleopInit()
    {
        System.out.println("Should now indicate only rising edge");
        interrupt.setInterruptEdges(true, false);
    }

    @Override
    public void autonomousInit()
    {
        System.out.println("Should now indicate both rising and falling edges");
        interrupt.setInterruptEdges(true, true);
    }
}
