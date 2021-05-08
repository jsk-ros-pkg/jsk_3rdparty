# This message defines audio stream data. Meta information about this audio
# stream is defined with AudioInfo.msg message. 

###########################
# Audio stream byte array #
###########################
#
# layout of each block of the byte array are defined according to 
# meta parameters. ( num_channels and bit_per_sample )
#
# +--------------+----------++----------+----------+----------+----------+
# | bit / sample | channels || 1st byte | 2nd byte | 3rd byte | 4th byte |
# +--------------+----------++----------+----------+----------+----------+
# |      8       | monaural ||   R ch   |                                |
# +--------------+----------++----------+----------+----------+----------+
# |      16      | monaural ||        R ch         |                     |
# +--------------+----------++----------+----------+----------+----------+
# |      8       |  stereo  ||   R ch   |   L ch   |                     |
# +--------------+----------++----------+----------+----------+----------+
# |      16      |  stereo  ||        R ch         |        L ch         |
# +--------------+----------++----------+----------+----------+----------+
#
uint8[] data
