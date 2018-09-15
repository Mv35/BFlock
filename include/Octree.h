//----------------------------------------------------------------------------------------------------------------------
/// @file Octree.h
/// Modified from:
/// Alexander la Tourelle,Flock
/// Available from:
/// https://github.com/mainConfetti/Flock[online]
/// @class Octree
/// @brief I have used this class to find the neighbours of a boid. I have made small modification
/// from the original class s

#ifndef OCTREE_H
#define OCTREE_H

#include <stdlib.h>
#include <ngl/AbstractVAO.h>
#include <vector>
#include <ngl/Vec4.h>
#include <ngl/Vec3.h>



class Octree
{
public:

  /// @brief ctor
  /// @param [in] _origin the physical centre of the node
  /// @param [in] _halfD half the width, depth, or height of the node
  /// @param [in] _height the height of the octree

  Octree(ngl::Vec3 _origin, float _halfD, int _height);

  /// @brief dtor recursivley destroys octants
  ~Octree();

  /// @brief sets the m_data member variable
  /// @param [in] _data the data to assign to m_data
  void setData(ngl::Vec4 _data);

  /// @brief Determine which octant of the tree contains "point"
  /// @param [in] point the point to check
  /// @return int denoting the octant containing "point"
  int getOctantContainingPoint(ngl::Vec3 point);

  /// @brief checks whether the current node is a leaf node
  /// @return true if node is leaf or false if not
  bool isLeaf();

  /// @brief if this node doesnt have a data point; insert one or if
  /// is does; split this node into 8 children and insert old and new data points
  /// @param [in] _data the data to insert
  void insert(ngl::Vec4 _data);

  /// @brief query the tree for points within a bounding sphere
  /// @param [in] centre the centre of the bounding sphere
  /// @param [in] radius the radius of the bounding sphere
  /// @return returns and appends the points inside the bounding sphere to m_resultsData
  ngl::Vec4 getPointsInsideSphere(ngl::Vec3 centre, float radius);

  /// @brief resets m_resultsData
  void clearResults();

  /// @brief builds a simple VAO object of the octrree for visualisation purposes
  void buildVAO();

  /// @brief resets m_resultsData
  std::vector<ngl::Vec4> m_resultsData;
private:

  /// @brief the physical centre of this node
  ngl::Vec3 m_origin;

  /// @brief half the width, depth or height of this node
  float m_halfD;

  /// @brief the height of the current node
  int m_height;

  /// @brief pointers to child octants
  std::vector <Octree> m_children;

  /// @brief  array of data points to be stored in tree, has x,y,z position and w Id of data's parent object
  std::vector<ngl::Vec4> m_data;
  /// @brief  a member variable to store how many data points have passed through the node when the tree is created.
  /// This is used to know how many times to itterate over each node when retreiving the data stored in
  /// its children.
  int m_tempDataIndex;

  /// @brief  A member variable to store an incrememnted value that coresponds with the index of the next data point to be
  /// returned when finding points inside a sphere
  int m_inc;

  /// @brief  a member variable storing the index of the current data point being queried
  int m_dataIndex;

  /// @brief  VAO containing the node's eight corners
  ngl::AbstractVAO* m_boidVAO;

  /// @brief  a function to return how many data points are stored in the node to its parent
  int getDataSize();

  /// @brief  a method to return m_dataIndex to the node's parent
  int getTempDataIndex();


};

#endif // OCTREE_H
