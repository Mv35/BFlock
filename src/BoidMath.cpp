#include "BoidMath.h"
#include <iostream>
#include <math.h>
#include <ngl/Transformation.h>


BoidMath::BoidMath()
{
    ///ctr
}

BoidMath::~BoidMath()
{
    ///dtr
}


/// @brief calculate the magnitude/distance between two points to determine collision or if a neighbour is within
/// radius of boid

float BoidMath::distance(ngl::Vec3 a, ngl::Vec3 b)
{
  return sqrt(((a.m_x - b.m_x)*(a.m_x - b.m_x)+(a.m_y - b.m_y)*(a.m_y - b.m_y) + (a.m_z - b.m_z)*(a.m_z - b.m_z)));
}

bool BoidMath::collisionDetect(ngl::Vec3 ahead, ngl::Vec3 a, ngl::Vec3 b, float radius)
{
  ngl::Vec3 ahead2=ahead*0.5;
  return distance(a, ahead) <= radius || distance(a, ahead2) <= radius || distance(a, b) <= radius;
}

bool BoidMath::boxIntersectsSphere(ngl::Vec3 bMin, ngl::Vec3 bMax, ngl::Vec3 C, float r)
{
  float dist_squared = r * r;
  if (C.m_x < bMin.m_x)
    dist_squared -= (C.m_x - bMin.m_x)*(C.m_x - bMin.m_x);
  else if (C.m_x > bMax.m_x)
    dist_squared -= (C.m_x - bMax.m_x)*(C.m_x - bMax.m_x);
  if (C.m_y < bMin.m_y)
    dist_squared -= (C.m_y - bMin.m_y)*(C.m_y - bMin.m_y);
  else if (C.m_y > bMax.m_y)
    dist_squared -= (C.m_y - bMax.m_y)*(C.m_y - bMax.m_y);
  if (C.m_z < bMin.m_z)
    dist_squared -= (C.m_z - bMin.m_z)*(C.m_z - bMin.m_z);
  else if (C.m_z > bMax.m_z)
    dist_squared -= (C.m_z - bMax.m_z)*(C.m_z - bMax.m_z);
  return dist_squared > 0;
}



