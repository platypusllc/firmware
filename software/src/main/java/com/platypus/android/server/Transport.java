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
public interface Transport  {
    public void send(PlatypusResponse response);
    
    public interface Receiver {
        public void receive(PlatypusCommand command, Transport transport);
    }
}
