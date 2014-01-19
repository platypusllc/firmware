# Platypus Server

The Platypus Server is an android application that connects to the
Platypus Control Board and enables autonomous control of a vehicle.

# Transport

Messages are sent via Google Protocol Buffers.  The server receives PlatypusCommand messages and returns PlatypusResponse messages, packed into erasure coded UDP packets.

## Erasure Coding

Each UDP packet to or from the server is at most 512 bytes (since the minimum maximum MTU for IPv4 is 576 bytes).  In order to more reliably transfer larger payloads, a simple XOR transport is implemented on top of this.

Each UDP packet contains the following:

[ ID (1-byte) ][ BLOCK (1-byte) ][ LEN (1-byte) ][ SEQ (1-byte) ][ PAYLOAD ]

What this means is:
  * This packet is part of a set of packets with the given ID.
  * The packets are encoded with a parity packet every BLOCK packets.
  * The total number of packets in this segment is LEN.
  * The position of this packet in this segment is SEQ.

### Small packets
We can largely ignore the protocol:
  * When writing, simply always append an incrementing ID, and use BLOCK = 1, LEN = 1, SEQ = 1.
  * When reading, simply check that LEN = 1, then skip ahead and read PAYLOAD as a self-contained Protocol Buffer.

### Large packets
#### Receiving
  * Assemble packets into a 256-entry retransmission table.  Always clear the ID to (ID+128 % 256) entries of this table when a packet with a new ID is received.
  * When packets are received, use LEN to allocate their place in the table.
#### Transmitting
  * Divide the payload by the desired MTU (probably 512), then compute LEN = (PAYLOAD / MTU) / BLOCK * (BLOCK + 1).  Every BLOCK packets, compute a parity packet by XORing together the previous BLOCK payloads.
  * Send the packets in any order.  It shouldn't matter.