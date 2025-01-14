// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


package frc.robot.devices;
import com.ctre.phoenix.sensors.*;

import java.io.ObjectInputFilter.Config;

import com.ctre.phoenix.platform.can.*;
import com.ctre.phoenix6.*;
import com.ctre.phoenix6.configs.*;


/** Add your docs here. */
public class CANivore {
    private CANivore sensorCANivore;
    private PigeonIMU gyro;

    public CANivore() {
        //Initialize CANivore for non-drivetrain devices
        sensorCANivore = new CANivore("SensorBus");
                //SensorBus is an example name we can give the auxiliary CAN bus
        
                //Example: Initialize non-drivetrain devices on the CANivore
                gyro = new PigeonIMU(2); //CAN id 10 on Sensor bus
                            }
                                
                        
                        private String getName() {
                        // TODO Auto-generated method stub
                        throw new UnsupportedOperationException("Unimplemented method 'getName'");
                    }
                
                
                        public CANivore(String string) {
                //TODO Auto-generated constructor stub
            }
        
        
        public void robotInit() {
    //example: Configure sensors if needed
    PigeonIMU gyro;((CANivore) getConfigurator()).applyDefaultConfiguration();
        }
    
    
            private Object getConfigurator() {
                // TODO Auto-generated method stub
                throw new UnsupportedOperationException("Unimplemented method 'getConfigurator'");
            }

            private Object applyDefaultConfiguration() {
                            return gyro;
            }
}
