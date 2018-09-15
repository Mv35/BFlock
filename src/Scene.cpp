#include "Scene.h"

///@file Scene.cpp
///@brief Class that handles the Boundings of the Simulation

void Scene::setSimulationBBShape(ngl::Obj *m_obj)
{

    ngl::BBox _bb = m_obj->getBBox();
    m_simBox.m_maxx = _bb.maxX();
    m_simBox.m_minx = _bb.minX();
    m_simBox.m_maxy = _bb.maxY();
    m_simBox.m_miny = _bb.minY();
    m_simBox.m_maxz = _bb.maxZ();
    m_simBox.m_minz = _bb.minZ();
}
void Scene::addWall(ngl::Vec3 m_point, float m_size, ngl::Vec3 m_normal, bool m_draw)
{
    Wall * newWall= new Wall;
    m_normal.normalize();
    newWall->center = m_point;
    newWall->size = m_size;
    newWall->a = m_normal.m_x;
    newWall->b = m_normal.m_y;
    newWall->c = m_normal.m_z;
    newWall->d = -(newWall->a * m_point.m_x + newWall->b * m_point.m_y + newWall->c * m_point.m_z);
    newWall->draw_flag = m_draw;

    m_walls.push_back(newWall);

}
void Scene::init()
{
    //size
    float _size = (m_simBox.m_maxx+m_simBox.m_maxy+m_simBox.m_maxz)/3 - (m_simBox.m_minx+m_simBox.m_miny+m_simBox.m_minz)/3;
    //adding walls
    //left wall
    addWall(ngl::Vec3(m_simBox.m_minx,0,0), _size, ngl::Vec3(1.0, 0.0, 0.0),true);
    //right wall
    addWall(ngl::Vec3(m_simBox.m_maxx,0,0), _size, ngl::Vec3(-1.0, 0.0, 0.0),true);
    //bottom wall
    addWall(ngl::Vec3(0,m_simBox.m_maxy,0), _size, ngl::Vec3(0.0, -1.0, 0.0),true);
    //top wall
    addWall(ngl::Vec3(0,m_simBox.m_miny,0), _size, ngl::Vec3(0.0, 1.0, 0.0),true);
    //front wall
    addWall(ngl::Vec3(0,0,m_simBox.m_maxz), _size, ngl::Vec3(0.0, 0.0, -1.0),false);
    //back wall
    addWall(ngl::Vec3(0,0,m_simBox.m_minz), _size, ngl::Vec3(0.0, 0.0, 1.0),true);
}

//clear the walls
void Scene::clearWalls()
{
  //BOOST_FOREACH(Wall *w, m_walls)
  for(auto w : m_walls)
  {
    delete w;
  }
  m_walls.clear();
}
