package com.platypus.android.server.transport;

import android.util.Log;

import com.platypus.android.server.VehicleTransport;
import com.platypus.protobuf.PlatypusCommand;
import com.platypus.protobuf.PlatypusResponse;
import com.squareup.wire.Wire;

import org.java_websocket.WebSocket;
import org.java_websocket.handshake.ClientHandshake;
import org.java_websocket.server.WebSocketServer;

import java.io.IOException;
import java.net.InetSocketAddress;
import java.util.Map;
import java.util.TreeMap;

public class WebSocketTransport extends WebSocketServer implements VehicleTransport {
    
    private static final String TAG = WebSocketTransport.class.getName();
    private static final Wire WIRE = new Wire();
    
    private final Receiver receiver_;
    
    private final Map<WebSocket, WebSocketInternalTransport> clients_ = new TreeMap<WebSocket, WebSocketInternalTransport>();
    
    public WebSocketTransport(int port, Receiver receiver) {
        super(new InetSocketAddress(port));
        receiver_ = receiver;
    }

    @Override
    public void onClose(WebSocket socket, int arg1, String arg2, boolean arg3) {
        synchronized (clients_) {
            clients_.remove(socket);   
        }
        
        Log.d(TAG, "Connection lost from '" + socket + "'");
    }

    @Override
    public void onError(WebSocket socket, Exception e) {
        synchronized (clients_) {
            clients_.remove(socket);   
        }
        
        Log.w(TAG, "Connection error from '" + socket + "'", e);
    }

    @Override
    public void onMessage(WebSocket socket, String message) {
        try {
            PlatypusCommand command = WIRE.parseFrom(message.getBytes(), PlatypusCommand.class);
            
            WebSocketInternalTransport transport;
            synchronized (clients_) {
                transport = clients_.get(socket);
            }
             
            receiver_.receive(command, transport);
        } catch (IOException e) {
            Log.w(TAG, "Failed to parse command.", e);
        }
    }

    @Override
    public void onOpen(WebSocket socket, ClientHandshake handshake) {
        synchronized (clients_) {
            clients_.put(socket, new WebSocketInternalTransport(socket));
        }
        
        Log.d(TAG, "Connection made from '" + socket + "'");
    }

    public void send(PlatypusResponse response) {
        byte[] bytes = response.toByteArray();
        
        synchronized (clients_) {
            for (WebSocket socket : clients_.keySet()) {
                socket.send(bytes);
            }
        }
    }

    private static class WebSocketInternalTransport implements VehicleTransport {
        private WebSocket socket_;

        public WebSocketInternalTransport(WebSocket socket) {
            socket_= socket;
        }

        public void send(PlatypusResponse response) {
            socket_.send(response.toByteArray());
        }
    }
}
