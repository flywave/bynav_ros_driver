# Message from GPGSV NMEA String

Header header

string message_id

# Number of messages in this sequence
uint8 n_msgs
# This messages number in its sequence. The first message is number 1.
uint8 msg_number

# Number of satellites currently visible
uint8 n_satellites

# Up to 4 satellites are described in each message
Satellite[] satellites