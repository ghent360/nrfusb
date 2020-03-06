# Theory of operation #

A transmitter and receiver must be programmed with identical 32 bit
radio IDs.  With those IDs, the transmitter and receiver select a set of
23 channels using a pseudorandom number generator.

## Channel Selection ##

23 channels are selected.  Their selection and order is determined as
follows:

The pseudorandom number function is:

```
prn = (prn * 0x0019660D) + 0x3C6EF35F
```

A potential channel is `prn % 125`.  Channels are evaluated in order such that:

* The prn is evaluated once, then modulo 125 to generate a candidate
  channel

* No channel is repeated.  If the candidate channel has already been
  selected, the candidate is discarded.

* No more than a certain number of channels can be chosen from each of
  the different bands.  If a candidate channel would exceed this
  limit, it is discarded.
  * 0-31: 6
  * 32-63: 6
  * 64-95: 6
  * 96-125: 5

## Transmission ##

The transmitter and receiver use the nrf24l01 Enhanced ShockBurst with
the following parameters:

* 5 byte address field, the channel ID mapped into 5 bytes as below
* Dynamic payload length
* 2 byte CRC
* No automatic retransmission
* Auto acknowledgement
* 1Mbps data rate

The transmitter sends a new packet every 20ms, waiting 2ms for an
acknowledgement with data on the same channel.  Each transmitted
packet advances to the next channel in the channel selected list,
wrapping around.

## Reception ##

To initially lock onto a transmitter, the receiver picks a random
channel from the list and listens for 20 channel change times (0.4s).
If no packet is received, it then moves to the next channel in the
list and waits another 20 time periods, continuing this process until
a packet is received.  Once a packet is received it switches to the
following procedure.

In normal reception, immediately after receiving and acknowledging a
packet, the receiver switches to the next channel in the list.  It
waits at least 22ms for the next packet to be received.  If no packet
is received, then the channel is switched to the next one and
listening recommences.  If 5 packets in a row are missed, then the
receiver returns to the initial lock procedure.

## ID selection ##

The ShockBurst address is derived from the 32 bit ID as follows.

1100bbbb bbbbbbbH bbbbbbbH bbbbbbbH bbbbbbH

`b` is a bit from the 32 bit ID.

The `H` bits are selected to be the opposite of whatever the
preceeding bit was.

Rationale: The highest bits are defined to be distinct from the
preamble, which consists of alternating bits.  The H bits are present
to reduce the likelihood that random noise will cause the address to
be identified by the baseband receiver.

## Scheduling ##

The transmitter and receiver have 16 different "slots" to hold
outgoing data.  Each can be configured for a different priority or
transmission rate and each can hold up to 16 bytes of data.

The transmission rate/priority is configured by providing a divisor
indicating that the data should be transmitted every Nth frame.  A
divisor of 1 means the data should be transmitted every frame, a
divisor of 2 means it should be transmitted every other etc.  It is up
to the application to ensure that a given selection of slot sizes and
frame rates is achievable.  If an infeasible selection is configured,
some undetermined data will not be sent.

A priority of 0 is the default, which signifies that the slot should
not be transmitted.

The data in a slot will continue to be transmitted at the specified
frequency whether or not it has been updated by the client recently.
