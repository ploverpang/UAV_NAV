#ifndef UAVNAV_UAVNAV_H_
#define UAVNAV_UAVNAV_H_

#define CAMERARANGE 10
#define RESOLUTION_M 0.5
#define WIDTH 320
#define HEIGHT 240
#define C_PI (double)3.141592653589793
#define DEG2RAD(DEG) ((DEG) * ((C_PI) / (180.0)))
#define RAD2DEG(RAD) ((RAD) * (180.0) / (C_PI))

inline float wrapToPi(float angle)
{
  angle = std::fmod(angle + C_PI, 2*C_PI);
  if(angle < 0)
    angle += 2 * C_PI;
  return (angle - C_PI);
}

inline float wrapTo2Pi(float angle)
{
  angle = std::fmod(angle, 2*C_PI);
  if(angle < 0)
    angle += 2*C_PI;
  return angle;
}

#endif // UAVNAV_UAVNAV_H_
