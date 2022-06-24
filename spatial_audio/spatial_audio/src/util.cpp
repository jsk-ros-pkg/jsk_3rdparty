// Standaerd C++ Library
#include <cmath>
#include <cstdio>
#include <iostream>
#include <string>
// OpenAL headers
#include <AL/al.h>
#include <AL/alc.h>
// User
#include <spatial_audio/util.h>

bool genBufferFromPCM(ALuint& buffer, ALvoid* data, ALsizei size, ALsizei sampling_rate, ALenum format)
{
  alGenBuffers(1, &buffer);
  alBufferData(buffer, format, (ALvoid*)data, size, sampling_rate);
  return true;
}

bool genSinWave(ALuint& buffer, unsigned int sampling_rate, unsigned int frequency, unsigned char amplitude,
                double period)
{
  /* Generation of sin wave */
  int data_length = (int)period * sampling_rate;
  unsigned char offset = 128;
  double sampling_period = 1.0 / sampling_rate;
  double sin_period = 1.0 / frequency;
  unsigned char* data = new unsigned char[data_length];
  ALenum format = AL_FORMAT_MONO8;
  ALsizei size = data_length;
  for (int i = 0; i < data_length; i++)
  {
    int val = offset + amplitude * sin(2 * M_PI * i * sampling_period / sin_period);
    if (val > 255)
    {
      data[i] = 255;
    }
    else if (val < 0)
    {
      data[i] = 0;
    }
    else
    {
      data[i] = val;
    }
  }

  /* Buffer Setup */
  genBufferFromPCM(buffer, data, size, sampling_rate, format);

  return true;
}
