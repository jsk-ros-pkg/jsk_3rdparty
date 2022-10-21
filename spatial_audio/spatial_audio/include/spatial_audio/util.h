#ifndef SPATIAL_AUDIO_UTIL_H__
#define SPATIAL_AUDIO_UTIL_H__

// Standaerd C++ Library
#include <cmath>
#include <cstdio>
#include <iostream>
#include <string>
// OpenAL headers
#include <AL/al.h>
#include <AL/alc.h>

/**
 * Generate an OpenAL buffer with a sin wave with specified parameters
 *
 * @param[out] buffer OpenAL buffer id
 * @param[in] sampling_rate sampling rate of a sin wave
 * @param[in] frequency frequency of a sin wave
 * @param[in] amplitude amplitude of a sin wave
 * @param[in] period duration of a sin wave
 */
bool genSinWave(ALuint& buffer, unsigned int sampling_rate, unsigned int frequency, unsigned char amplitude,
                double period);

/**
 * Generate an OpenAL buffer with a specified pcm data
 *
 * @param[out] buffer OpenAL buffer id
 * @param[in] data a pointer to a pcm data buffer
 * @param[in] size the size of the pcm data buffer
 * @param[in] sampling_rate sampling rate of the pcm data
 * @param[in] format pcm data format
 */
bool genBufferFromPCM(ALuint& buffer, ALvoid* data, ALsizei size, ALsizei sampling_rate, ALenum format);

#endif
