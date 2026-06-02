#ifndef TYPES_H
#define TYPES_H

struct EncoderTracker
{
  long prev_left = 0;
  long prev_right = 0;
};

struct Pose
{
  float x;
  float y;
  float theta;
};

#endif
