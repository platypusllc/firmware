/**
 * 
 */
package com.platypus.android.server;

import com.platypus.protobuf.PlatypusCommand;
import com.platypus.protobuf.PlatypusResponse;

/**
 * Defines generic transport protocol by which the server can
 * send and receive commands.
 * 
 * @author pkv
 *
 */
public interface VehicleTransport  {
    public void send(PlatypusResponse response);
    // TODO: should this have an onReceive in the interface?
    
    public interface Receiver {
        public void receive(PlatypusCommand command, VehicleTransport transport);
    }
}
