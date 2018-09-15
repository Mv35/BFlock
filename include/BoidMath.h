#ifndef BOIDMATH_H
#define BOIDMATH_H
//----------------------------------------------------------------------------------------------------------------------
/// @file BoidMath.h
/// @class BoidMath
/// @brief this class calculates the distance between the boids
//----------------------------------------------------------------------------------------------------------------------


#include <math.h>
#include <ngl/Vec3.h>
#include <ngl/Vec2.h>
#include <ngl/Mat4.h>
#include <vector>

class BoidMath
{
public:

  /// @brief ctor
  BoidMath();

  /// @brief ctor
  ~BoidMath();

  /// @brief method to calculate the distance between the boids in the flock
  static float distance(ngl::Vec3 a, ngl::Vec3 b);

  /// @brief method to calculate detect if a collision has occured
  static bool collisionDetect(ngl::Vec3 ahead, ngl::Vec3 a, ngl::Vec3 b, float radius);

  /// @brief query if a box intersects with a sphere
  /// @param [in] bMin the minimum corner of the box
  /// @param [in] bMax the maximum corner of the box
  /// @param [in] C the centre of the sphere
  /// @param [in] r the radius of the circle
  /// @return true if box intersects with sphere, false if not
  static bool boxIntersectsSphere(ngl::Vec3 bMin, ngl::Vec3 bMax, ngl::Vec3 C, float r);

};

#endif // BOIDMATH_H
