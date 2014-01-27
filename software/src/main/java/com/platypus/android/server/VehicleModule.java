/**
 * 
 */
package com.platypus.android.server;

import android.util.JsonReader;

import com.platypus.protobuf.PlatypusCommand;

/**
 * Defines an interface for a vehicle sub-component, allowing different subsystems
 * of the vehicle to be split into different objects. 
 *  
 * @author pkv
 *
 */
public interface VehicleModule {
    
    public void start(VehicleService vehicle);
    public void stop();
    
    public boolean onTransportReceive(PlatypusCommand command, VehicleTransport sender);
    public boolean onAccessoryReceive(String name, JsonReader response);
}
