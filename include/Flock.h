//----------------------------------------------------------------------------------------------------------------------
/// @file Flock.h
/// @class Flock
/// @brief a class for creating the Flock, managing boids and their neighbours
//----------------------------------------------------------------------------------------------------------------------
#ifndef Flock_H
#define Flock_H

#include <stdlib.h>
#include <cmath>
#include <vector>
#include <Boid.h>
#include <Octree.h>
#include <SimExport.h>
#include <memory>
class Flock
{
public:

  /// @brief ctor
  /// @param [in] _numBoids the number of boids in the Flock
  Flock(int _numBoids, int _numPredators);

  /// @brief copyctor to set custom steering force weights, speed and mass as well as the number of boids
  /// @param [in] _numBoids the number of boids in the Flock
  /// @param [in] _cohesion the custom cohesion force weight to set
  /// @param [in] _separation the custom spearation force weight to set
  /// @param [in] _alignment the custom alignment force weight to set
  /// @param [in] _speed the custom speed value to be set
  /// @param [in] _mass the custom mass value to be set
  Flock(int _numBoids, int _cohesion, int _separation, int _alignment, float _speed, int _mass, int _numPredators, int _predatorSight, float _predatorAttack, int _predatorAttRadius);

  /// @brief dtor
  ~Flock();

  /// @brief method to add a boid to the Flock
  /// @param [in] _cohesion the cohesion force weight to set to the boid
  /// @param [in] _separation the spearation force weight to set to the boid
  /// @param [in] _alignment the alignment force weight to set to the boid
  /// @param [in] _speed the speed value to set to the boid
  /// @param [in] _mass the mass value to set to the boid 
  void createFlock(int _cohesion, int _separation, int _alignment, float _speed, int _mass);

  /// @brief method to remove a boid from the Flock 
  void removeBoid(int _id);

  /// @brief method to st the boid position
  /// @param [in] _id the id of the boid to set neighbours to
  void setNeighbours(int _id);

  /// @brief method to return the size of the Flock
  int getSize();

  /// @brief method to update the Flock, neighbours,
  /// collisions and positions of the boids
  void updateFlock();

  /// @brief vector to contain all boids within the Flock
  std::vector<Boid> m_Flock;

  ///@brief vector containing the predator boids
  ///@return vector of predators
  std::vector<Boid*> m_predators;

  ///@brief method to get the herding radius
  ///@return a float value of the herding radius
  float getHerdBoundingRadius();

  ///@brief the herding radius
  float m_herdingRadius;

  ///@brief method to export a frame of the simulation
  void exportFrame();

  ///@brief method to create predators
  void createPredators(int _cohesion, int _separation, int _alignment, float _speed, int _mass, int _predatorSight, float _predatorAttack, float _predatorAttRadius );

  ///@brief method to set the EfficientHunt hunting method for predators
  void setEfficientHunt();

  ///@brief method to sort neighbours-boid distances in ascending order
  ///@param [in] _distances, the distances
  ///@return sorted distances
  std::vector<float> sortDistances(std::vector<float> _distances);

  ///@brief method to set the Phunt hunting method for predators
  void Phunt();

  ///@brief method to set the ExtremeHunt hunting behavior for predators (unused)
  ///@return a hunting direction
  ngl::Vec3 ExtremeHunt();

  ///@brief method to calculate centroid
  ///@return centroid
  ngl::Vec3 getCentroid();

  ///@brief method to calculate the avarage velocity of the flock
  void calculateAvVelocity();

  ///@brief method to complete the hunt
  ///@return returns true if the hunt was successfull
  bool setPAttack(Boid *m_predator);

  ///@brief the centroid
  ngl::Vec3 m_centroid;

  ///@brief the avarage velocity
  ngl::Vec3 m_AvVelocity;

  ///@brief distance from the centroid of the flock and the predator
  float getCentroidPredatorDistance();

  ///@brief method to get the predators
  ///@return the vector containing the predators
  std::vector<Boid *> getPredators();

  ///@brief int to store the number of killed boid
  int kill_counter=0;

  ///@brief the distance from the flock centroid and the predator
  float m_CentroidPredatorDistance;

  ///@brief the list of neighbours
  std::vector<Boid*> m_neighbours;

  ///@brief method to enable evasion as a wild prey
  ///@param [in] _boid, the boid to evade
  void Evade(Boid _boid);

  ///@brief method to enable evasion as herdable sheep
  ///@param [in] _boid, the boid to evade
  void Herd(Boid _boid);

  ///@brief method to enable the protect position behavior from predators
  ///@param [in] _position, the position to protect
  ///@param [in] _perimeterRadius,  the radius
  void ProtectPosition(ngl::Vec3 _position, float _perimeterRadius);

  ///@brief protection radius
  float m_protRadius;

  ///@brief protection position
  ngl::Vec3 m_protPos;

  ///flags behaviours controllers/togglers to/from ui
  ///@brief bool flag to indicate centroid position based hunting behaviour
  bool m_HuntA;
  ///@brief bool flag to indicate if centroid and avarage velocity based hunting behaviour
  bool m_HuntB;
  ///@brief bool flag to indicate if evasion for herdable sheeps
  bool m_evadeSheep;
  ///@brief bool flag to indicate if evasion for not domesticated prays
  bool m_evadePrey;
  ///@brief bool flag to indicate if predators  do nothing
  bool m_chill;
  ///@brief bool flag to indicate if predators protect
  bool m_protectPosition;
  ///@brief bool flag to indicate if predators gathers
  bool m_gather;

  ///@brief bool flag to toggle export
  bool m_export;

  ///@brief int iteration counts for export
  int m_itr;

private:

  /// @brief method to update the octree. deletes the previous octree,
  /// creates a new one and populates it with new data

  void updateOctree();

  /// @brief a pointer to the octree class for neighbours handling

  Octree *m_octree;

  SimExport m_SimExport;
  //int poscount;
  //int dircount;
};

#endif // Flock_H
