
package com.platypus.android.server.transport;

import android.util.Log;

import com.google.common.cache.Cache;
import com.google.common.cache.CacheBuilder;
import com.platypus.android.server.VehicleTransport;
import com.platypus.protobuf.PlatypusCommand;
import com.platypus.protobuf.PlatypusResponse;
import com.squareup.wire.Wire;

import java.io.IOException;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.SocketAddress;
import java.net.SocketException;
import java.util.concurrent.TimeUnit;

public class UdpTransport implements VehicleTransport {

    private static final String TAG = UdpTransport.class.getName();
    private static final Wire WIRE = new Wire();

    private final DatagramSocket socket_;
    private final Receiver receiver_;

    private final Cache<SocketAddress, UdpInternalTransport> clients_ = CacheBuilder.newBuilder()
            .maximumSize(64)
            .expireAfterWrite(2, TimeUnit.MINUTES)
            .build();

    public UdpTransport(int port, Receiver receiver) {
        DatagramSocket socket = null;
        try {
            socket = new DatagramSocket(port);
            new Thread(listener_).start();
        } catch (SocketException e) {
            Log.e(TAG, "Failed to open socket.", e);

        }

        receiver_ = receiver;
        socket_ = socket;
    }

    public void send(PlatypusResponse response) {
        byte[] bytes = response.toByteArray();
        DatagramPacket packet = new DatagramPacket(bytes, bytes.length);
        
        synchronized (clients_) {
            for (SocketAddress client : clients_.asMap().keySet()) {
                packet.setSocketAddress(client);
                try {
                    socket_.send(packet);
                } catch (IOException e) {
                    Log.w(TAG, "Failed to send packet.", e);
                }
            }
        }
    }

    private Runnable listener_ = new Runnable() {
        public void run() {
            byte[] buffer = new byte[576];
            DatagramPacket packet = new DatagramPacket(buffer, buffer.length);

            try {
                while (true) {
                    socket_.receive(packet);
                    SocketAddress sender = packet.getSocketAddress();
                    
                    UdpInternalTransport transport;
                    synchronized (clients_) {
                        transport = clients_.getIfPresent(sender);
                        if (transport == null) {
                            transport = new UdpInternalTransport(sender);
                        }
                        clients_.put(sender, transport);
                    }
                    
                    try {
                        PlatypusCommand command = WIRE.parseFrom(packet.getData(), PlatypusCommand.class);
                        receiver_.receive(command, transport);
                    } catch (IOException e) {
                        Log.w(TAG, "Failed to parse command.", e);
                    }                   
                }
            } catch (IOException e) {
                Log.d(TAG, "Socket stopped receiving.", e);
            }
        }
    };

    private class UdpInternalTransport implements VehicleTransport {
        private SocketAddress address_;

        public UdpInternalTransport(SocketAddress address) {
            address_ = address;
        }

        public void send(PlatypusResponse response) {
            byte[] bytes = response.toByteArray();
            DatagramPacket packet = new DatagramPacket(bytes, bytes.length);
            packet.setSocketAddress(address_);
            
            try {
                socket_.send(packet);
            } catch (IOException e) {
                Log.w(TAG, "Failed to send packet.", e);
            }
        }
    }
}
