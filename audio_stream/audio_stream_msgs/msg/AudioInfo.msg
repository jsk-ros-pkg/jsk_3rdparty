# This message defines meta information for a audio stream. It should be in a
# audio namespace on topic "audio_info" and accompanied by audio stream 
# topics named as "audio"

##########
# header #
##########
# Currently not in use
Header header


##################################
# number of channels information #
##################################
#
uint8 channels
#
uint8 AUDIOINFO_CHANNELS_MONAURAL = 1
uint8 AUDIOINFO_CHANNELS_STEREO = 2 # NOT SUPPORTED NOW


#############################
# sampling_rate information #
#############################
# Hz
uint32 sampling_rate


#############################
# sampling_rate information #
#############################
# Byte
uint16 width
#
uint16 AUDIOINFO_WIDTH_8BIT = 1
uint16 AUDIOINFO_WIDTH_16BIT = 2
