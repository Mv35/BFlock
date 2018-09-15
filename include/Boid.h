#ifndef BOID_H
#define BOID_H

#include <ngl/Vec3.h>
#include <ngl/Colour.h>
#include <ngl/Transformation.h>
#include <ngl/ShaderLib.h>
#include <ngl/VAOPrimitives.h>
#include <time.h>

///@class Boid
///@file Boid.h
///@author Milo Volpicelli
///@brief Boid class holding parameters

class Boid
{
public:
  //----------------------------------------------------------------------------------------------------------------------
  /// @brief ctor
  /// @param [in] _id boid ID
  //----------------------------------------------------------------------------------------------------------------------
  Boid(int _id);
  //--------------------------------------------------------------------------------------------------------------------
  /// @brief dtor
  //----------------------------------------------------------------------------------------------------------------------
  ~Boid();


  bool isDead;
  int m_predatorsNum;
  ///@brief bool flag to indicate if boid is a predator or not
  bool m_isPredator;
  ///@brief predator sight
  int m_predatorSight;
  ///@brief predator attack power
  float m_predatorAttack;
  ///@brief predator max velocity
  float m_predatorMaxVel;
  ///@brief predator radius
  int m_predatorRadius;
  ///@brief predator attack radius
  float m_predatorAttRadius;
  ///@brief predator cohesion weight
  float m_predatorCohesionWeight;
  ///@brief predator separation weight
  float m_predatorSeparationWeight;
  ///@brief predator allignment weight
  float m_predatorAllignmentWeight;
  ///@brief predator randomness weight
  float m_predatorRandomWeight;
  ///@brief predator Min separation
  float m_predatorMinSepa;
  ///@brief predator mass
  float m_predatorMass;
  ///@brief bool flag to indicate if boid is prey
  bool isPrey;
  ///@brief bool flag to indicate that boid is fleeing
  bool isFleeing;
  ///@brief bool flag to indicate that boid is gathered
  bool isGathered;

  ///@brief predator position
  ngl::Vec3 m_pPosition;

  ///@brief predator velocity
  ngl::Vec3 m_pVelocity;

  ///@brief predator speed
  float m_pSpeed;

  ///@brief predator cohesion
  ngl::Vec3 m_pCohesion;

  ///@brief predator alignment
  ngl::Vec3 m_pAlignment;

  ///@brief predator separation
  ngl::Vec3 m_pSeparation;

  ///@brief predator direction
  ngl::Vec3 m_pDirection;

  ///@brief predator steering
  ngl::Vec3 m_pSteering;

  ///@brief predator avoid
  ngl::Vec3 m_pAvoid;

  ///@brief predator attack
  ngl::Vec3 m_pAttack;

  ///@brief predator hunt
  ngl::Vec3 m_hunt;


  /// @brief method to get the sorted neighbours
  /// @return list of sorted neighbours
  std::vector<Boid *> getSortedNeighb() const {return m_sortedNeighbours;}

  /// @brief method to get the flee vector
  /// @return returns the flee vector force
  ngl::Vec3 getFlee() const;

  /// @brief method to get predators vector list
  /// @return returns a vector with the predators
  std::vector<Boid*> getPredators();

  /// @brief method to get the number of neighbours the boid has
  /// @return returns the size of m_neighbours
  int getNeighbours() const {return m_neighbours.size();}

  /// @brief method to get the neighbours of the boid
  /// @return returns the list of m_neighbours
  std::vector<Boid *> getNeighb() const {return m_neighbours;}

  ///@brief prey method to set gathering behaviour
  ///@param [in] _fleePos, the position to flee from
  void getGathered(ngl::Vec3 _fleePos);

  /// @brief method to get the position of the boid
  /// @return returns m_position
  ngl::Vec3 getPosition() const {return m_position;}

  /// @brief method to get the velocity of the boid
  /// @return returns m_velocity
  ngl::Vec3 getVelocity() const {return m_velocity;}


  /// @brief method to get the bounding radius
  /// @return returns m_boundRadius
  float getRadius() const {return m_boundRadius;}

  /// @brief method to get the id of a boid
  /// @return returns m_id
  int getId() const {return m_id;}

  /// @brief method to get the x and y rotations of a boid
  /// @return returns a Vec3 of (m_pitch, m_yaw, 0)
  ngl::Vec3 getRotation() const {return ngl::Vec3(m_pitch, m_yaw, 0);}

  /// @brief method to get the distance a boid will look for neighbours
  /// @return returns m_searchRad
  int getSearchRad() const {return m_searchRad;}

  ///@brief method to set a hunting behaviour according to pre-calculated prey position
  ///@param [in] _position, the position
  ///@param [in] _velocity, the velocity
  void setHunt(ngl::Vec3 _position, ngl::Vec3 _velocity);

  ///@brief method to set a hunting behaviour according to current prey position
  ///@param [in] _position, the position
  void setHunt2(ngl::Vec3 _position);

  ///@brief bool flag to indicate if predator is hunting
  bool m_isHunting;
  //void setPAvoid();

  ///@brief predator method to set the steering
  void setPSteering();

  ///@brief predator method to set the Rotation
  void setPRotate();

  ///@brief predator method to set the direction
  ///@param [in] _direction, the direction
  void setPDirection(ngl::Vec3 _direction);

  ///@brief predator method to set it's direction accoring to a pre-calculated position
  ///@param [in] _direction, the direction
  ///@param [in], _velocity, the velocity
  void setPredatorDirection(ngl::Vec3 _direction, ngl::Vec3 _velocity);


  ///@brief predator method to set the Chill! behaviour for the predators
  void setWander();

  ///@brief prey method to evade to current flock's centroid
  ///@param [in] _boid, the boid
  void evadeToFlock(Boid _boid);

  ///@brief predator method to set gathering behaviour
  ///@param [in] _position, the position
  ///@param [in] _radius , the gathering radius
  void setGather(ngl::Vec3 _position, float _radius);

  ///@brief predator cohesion setter method
  void setPCohesion();

  ///@brief predator alignment setter method
  void setPAlignment();

  ///@brief predator separation setter method
  void setPSeparation();


  ///@brief predator avoid setter method
  void setPAvoid();


  ///@brief predator attack setter method
  void setAttack();

  ///@brief prey flee method
  ///@param [in] _fleePos, position to flee from
  void setFlee(ngl::Vec3 _fleePos);

  ///@brief prey evade method
  ///@param [in] _boid, the boid to evade
  void evade(Boid _boid);

  ///@brief predator method to move
  void movePredator();

  ///@brief predator method to update pos
  void updatePPosition();

  ///@brief method to get boids speed
  float getSpeed();

  ///@brief method to get boid direction
  ///@return the direction
  ngl::Vec3 getDirection() const {return m_direction;}

  /// @brief method to st the boid position
  /// @param [in] _x the x value ot be set
  /// @param [in] _y the y value ot be set
  /// @param [in] _z the z value ot be set

  void setPos(float _x, float _y, float _z);

  /// @brief method to set a neighbour to the boid
  /// @param [in] boid the neighbouring boid
  /// @return appends boid to m_neighbours

  void setNeighbour(Boid *boid);

  /// @brief method to cclear the boid's neighbours
  /// @return clears m_neighbours

  void clearNeighbour();

  /// @brief method to set the boid's velocity
  /// @param [in] _x the x value ot be set
  /// @param [in] _y the y value ot be set
  /// @param [in] _z the z value ot be set

  void setVelocity(float _x, float _y, float _z);

  /// @brief method to set the weight on the separation force
  /// @param [in] _separationWeight the weight value to set

  void setSWeight(int _separationWeight);

  /// @brief method to set the weight on the cohesion force
  /// @param [in] _cohesionWeight the weight value to set

  void setCWeight(int _cohesionWeight);

  /// @brief method to set the weight on the alignment force
  /// @param [in] _alignWeight the weight value to set

  void setAWeight(int _alignWeight);

  /// @brief method to set the mass of the boid
  /// @param [in] _mass the value to set

  void setMass(int _mass);


  /// @brief method to set the speed of the boid
  /// @param _speed the speed value to set

  void setSpeed(float _speed);

  /// @brief method to set the separation distance
  /// @param [in] _sepDist the value to be set

  inline void setSepDist(int _sepDist){m_separationDistance = _sepDist;}


  /// @brief method to get the nearest neighbours the boid has
  ///@param [in] _boid, the boid
  void nearestNeighbour(Boid * _boid);

  /// @brief method to sort in ascending order the closest neighbour
  /// @return returns sorted neighbours
  std::vector<Boid *> m_sortedNeighbours;

  /// @brief method to set the direction to the nearest neighbour
  ///@param [in] _boid, the boid
  void setDirectiontoNearestN(Boid * _boid);

  /// @brief method to set a boid as predator
  ///@param [in] _boid, the boid to be predator
  void setPredators(Boid *_boid);

  /// @brief method to calculate and set steering force vector
  void setSteering();

  /// @brief method to calculate and set cohesion force vector
  void setCohesion();

  /// @brief method to calculate and set the alignment force vector
  void setAlignment();

  /// @brief method to calculate and set the separation force vector
  void setSeparation();

  /// @brief method to calculate and set the avoidance force vector
  void setAvoid();

  /// @brief method to calculate and set the reposition force vector
  /// @param [in] _reposition the point to reposition from
  void setReposition(ngl::Vec3 _reposition);

  /// @brief method to calculate and set the boid's rotation based on its velocity
  void setRotate();

  /// @brief method to set the direction force
  void setDirection();

  /// @brief method to calculate and set the new position based on the current steering
  /// vector, current velocity and speed
  void updatePosition();


  /// @brief method to move the boid by combining all the steering methods and
  /// updatePosition
  void move();

  /// @brief method to check if boids are colliding
  void Collision(ngl::Vec3 _pos, float _rad);

  /// @brief method to calculate and set the reposition force vector away from the walls
  void repositionBounds();



private:

  /// @brief the bounding radius of the boid
  float m_boundRadius;

  /// @brief the boids position
  ngl::Vec3 m_position;

  /// @brief vector containing pointers to all the boid's neighbours
  std::vector<Boid *> m_neighbours;

   /// @brief vector containing pointers to all the predators
  std::vector<Boid *> m_predators;

  /// @brief the boid's ID
  int m_id;

  /// @brief the boid's current speed
  float m_speed;

  /// @brief the initial speed set to the boid, this acts as a minimum
  float m_setSpeed;

  /// @brief the weight of the cohesion force acting on the boid
  float m_cohesionWeight;

  /// @brief the weight of the alignment force acting on the boid
  float m_alignmentWeight;

  /// @brief the weight of the separation force acting on the boid
  float m_separationWeight;

  /// @brief the distance at which boids begin to feel the separation force
  float m_separationDistance;

  /// @brief the mass of the boid, determines how quickly boid can change direction or move
  int m_mass;

  /// @brief the distance the boid sees ahead of itself
  float m_viewRange;

  /// @brief the weight of the avoidance force
  int m_avoidWeight;

  /// @brief the position of the collision
  ngl::Vec3 m_collisionPos;

  /// @brief the boids veclocity
  ngl::Vec3 m_velocity;

  /// @brief the cohesion force vector
  ngl::Vec3 m_cohesion;

  /// @brief the alignment force vector
  ngl::Vec3 m_alignment;

  /// @brief the separation force vector
  ngl::Vec3 m_separation;

  /// @brief the avoidance force vector
  ngl::Vec3 m_avoid;

  ///@brief wander force vector
  ngl::Vec3 m_wander;

  ///@brief flee force vector
  ngl::Vec3 m_flee;

  /// @brief the direction vector that is the result of the weighted
  /// average of all the steering forces
  ngl::Vec3 m_direction;

  /// @brief vector from the current velocity to the direction vector.
  ngl::Vec3 m_steering;

  /// @brief the reposition force vector
  ngl::Vec3 m_reposition;

  /// @brief the yaw (y-axis rotation) of the boid
  float m_yaw;

  /// @brief the pitch (x-axis rotation) of the boid
  float m_pitch;

  /// @brief the distance that the boid searches for neighbours
  int m_searchRad;

  ///@brief max speed
  float m_maxVel;

  ///@brief target to seek or pursuit
  ngl::Vec3 m_target;
};


#endif // BOID_H
