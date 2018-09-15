#include "Flock.h"
#include "Boid.h"
#include <math.h>
#include <stdlib.h>
#include <vector>
#include <time.h>
#include <BoidMath.h>
#include <ngl/NGLStream.h>
#include <fstream>
#include <sstream>

Flock::Flock(int _numBoids, int _numPredators)
{
  for(int i=0; i < _numBoids; i++)
  {
    createFlock(0, 10, 10,5.5, 15);
  }
  for(int j=0; j<_numPredators; j++)
  {
      createPredators(200,200,50,10,20,10,5,5);
      //std::cout<<m_Flock.size()<<'\n';
  }
  m_octree=new Octree(ngl::Vec3(0,0,0), 150, 6);
  m_evadePrey=false;
  m_evadeSheep=false;
  m_HuntA=false;
  m_HuntB=false;
  m_chill=true;
  m_protectPosition=false;
  m_gather=false;
  m_protRadius=200.0f;
  m_protPos=(300,300,300);
  m_itr=0;
}

Flock::Flock(int _numBoids, int _cohesion, int _separation, int _alignment, float _speed, int _mass, int _numPredators, int _predatorSight,float  _predatorAttack, int _predatorAttRadius)
{
  for(int i=0; i < _numBoids; i++)
  {
    createFlock(_cohesion, _separation, _alignment, _speed, _mass);
  }
  for(int j=0; j<_numPredators; j++)
  {
      createPredators(_cohesion,_separation, _alignment,_speed,_mass, _predatorSight, _predatorAttack, _predatorAttRadius);
      //std::cout<<m_predators.size()<<'\n';
  }
  m_octree=new Octree(ngl::Vec3(0,0,0), 150, 6);
}

Flock::~Flock()
{
  delete m_octree;
}
void Flock::createPredators(int _cohesion, int _separation, int _alignment, float _speed, int _mass, int _predatorSight, float _predatorAttack, float _predatorAttRadius )
{
    float min = -1.0, max = 1.0;
    int r;
    float fraction;
    r = rand();
    fraction = ((float) r / RAND_MAX) * (max - min);
    float x = min + fraction;
    r = rand();
    fraction = ((float) r / RAND_MAX) * (max - min);
    float y = min + fraction;
    r = rand();
    fraction = ((float) r / RAND_MAX) * (max - min);
    float z =min + fraction;
    int id = m_Flock.size() + 1;
    Boid predator(id); //current boid's id
    predator.setPos(0,0,0);
    // set the steering force weights from user
/*
    predator.m_pSpeed=_speed;
    predator.m_predatorMass=_mass;
    predator.m_predatorSight=_predatorSight;
    predator.m_predatorAttRadius=_predatorAttRadius;
    predator.m_predatorAttack=_predatorAttack;
    predator.m_isPredator = true;
    predator.m_predatorAllignmentWeight=_alignment;
    predator.m_predatorCohesionWeight=_cohesion;
    predator.m_predatorSeparationWeight= _separation;
    //et the randomised velocity
    //predator.setVelocity(x,y,z);
    predator.m_pVelocity=(x,y,z);*/
    //predator.setPos(0,0,0);
    //predatoret the steering force weights from user
    predator.setCWeight(_cohesion);
    predator.setSWeight(_separation);
    predator.setAWeight(_alignment);
    predator.setSpeed(_speed);
    predator.setMass(_mass);
    //et the randomised velocity
    predator.setVelocity(x,y,z);
    predator.m_isPredator = true;
    predator.m_predatorAttRadius=_predatorAttRadius;
    // add the boid to the Flock array
    m_Flock.push_back(predator);
}
void Flock::createFlock(int _cohesion, int _separation, int _alignment, float _speed, int _mass)
{
  // randomise the velocity and set initial position to the origin
  float min = -1.0, max = 1.0;
  int r;
  float fraction;
  r = rand();
  fraction = ((float) r / RAND_MAX) * (max - min);
  float x = min + fraction;
  r = rand();
  fraction = ((float) r / RAND_MAX) * (max - min);
  float y = min + fraction;
  r = rand();
  fraction = ((float) r / RAND_MAX) * (max - min);
  float z =min + fraction;
  int id = m_Flock.size() + 1;
  Boid boid(id); //current boid's id
  boid.setPos(0,0,0);
  // set the steering force weights from user
  boid.setCWeight(_cohesion);
  boid.setSWeight(_separation);
  boid.setAWeight(_alignment);
  boid.setSpeed(_speed);
  boid.setMass(_mass);
  // set the randomised velocity
  boid.setVelocity(x,y,z);
  boid.isPrey=false;
  boid.isDead=false;
  // add the boid to the Flock array
  m_Flock.push_back(boid);

}

void Flock::removeBoid( int _id)
{

  if(m_Flock.size()>0)
  {
      for(int i=0; i<(int)m_Flock.size(); i++)
      {
           if(m_Flock[i].getId()==_id)
               m_Flock.erase(m_Flock.begin()+m_Flock[i].getId());
        }
  }
}

int Flock::getSize()
{
  return m_Flock.size();
}

void Flock::updateOctree()
{
  // delete the old octree
  delete m_octree;
  // create a new octree
  m_octree=new Octree(ngl::Vec3(0,0,0), 150, 6);
  // populate the new octree with all the new data points
  ngl::Vec4 dataPoint;
  for(int i=0;i<(int)m_Flock.size();++i)
  {
    dataPoint.set(m_Flock[i].getPosition().m_x,m_Flock[i].getPosition().m_y,m_Flock[i].getPosition().m_z,m_Flock[i].getId());
    m_octree->insert(dataPoint);
  }
}

void Flock::setNeighbours(int _id)
{
  // centre of sphere to check is the position of the boid who's
  // neighbours are being set
  ngl::Vec3 centre = (m_Flock[_id].getPosition());
  // boid's search radius
  int rad = m_Flock[_id].getSearchRad();
  // clear the old neighbours
  m_Flock[_id].clearNeighbour();
  // search the octree for boids in the search sphere
  m_octree->getPointsInsideSphere(centre, rad);
  for(int i=0;i<(int)m_octree->m_resultsData.size();++i)
  {
    // the neighbour id is the 4th element of the data vector
    float id = (m_octree->m_resultsData[i].m_w);
    // set the boid to be a neighbour only if it is not the current boid
    if(id!=m_Flock[_id].getId())
    {
      m_Flock[_id].setNeighbour(&m_Flock[(int)id-1]);
    }
  }
}
ngl::Vec3  Flock::getCentroid()
{
    ngl::Vec3 sum(0.0,0.0,0.0);
    for(unsigned int i=0; i< m_Flock.size(); ++i)
    {
        if(m_Flock[i].m_isPredator==false)
        {
            sum=sum + m_Flock[i].getPosition();

        }

    }
    sum/=(m_Flock.size()-m_predators.size());
    m_centroid=sum;
    return sum;

}

void Flock::calculateAvVelocity()
{
    ngl::Vec3 sum(0.0,0.0,0.0);
    for(unsigned int i=0; i< m_Flock.size(); ++i)
        {
            if(m_Flock[i].m_isPredator==false)
            {
                sum=sum + m_Flock[i].getVelocity();
                sum=sum/m_Flock.size();
            }
        }
    m_AvVelocity=sum;
}

float Flock::getCentroidPredatorDistance()
{
    return m_CentroidPredatorDistance;
}
std::vector<float> Flock::sortDistances(std::vector<float> _distances)
{
    //unsigned int i,j;
    float temp;
    for(unsigned int i=0;i< _distances.size(); i++)
    {
        for(unsigned int j=0; j<_distances.size() ; j++)
        {
            if(_distances[i]<_distances[j])
            {
                temp=_distances[i];
                _distances[i]=_distances[j];
                _distances[j]= temp;
            }
        }
    }
    return _distances;
}
void Flock::setEfficientHunt()
{
    getPredators();
    ngl::Vec3 _centroid= getCentroid();
    calculateAvVelocity();
    //std::cout<<m_AvVelocity.m_x<<" "<<'\n';
    int kill_count=0;
    for(unsigned int i=0; i< m_predators.size(); ++i)
    {
        if(m_Flock.size()-m_predators.size()==1)
        {
            //std::cout<<"prova"<<'\n';

            ngl::Vec3 temp_dir= m_Flock[0].getPosition()+m_Flock[i].getDirection()+m_Flock[0].getVelocity()*m_Flock[0].getSpeed();
            m_predators[i]->setPDirection(temp_dir);
            m_predators[i]->setSpeed(12);
        }
        m_predators[i]->setHunt(m_centroid, m_AvVelocity);
        ngl::Vec3 diff = _centroid - m_predators[i]->getPosition();
        float dist= diff.length();
        //m_CentroidPredatorDistance=dist;
        //ngl::Vec3 diff2 = ExtremeHunt()-m_predators[i]->getPosition();
        //float dist2 = diff2.length();

        //if(dist<m_predators[i]->getSearchRad() )
        //{


            //m_predators[i]->setHunt2(ExtremeHunt());
            if(dist<=m_predators[i]->getSearchRad())
            {
                m_predators[i]->setSpeed(12);
                //m_predators[i]->m_isHunting==true;
                setPAttack(m_predators[i]);
                //m_predators[i]->setAttack();

                //std::cout<<"here"<<'\n';
                std::cout<<m_predators[i]->getSpeed()<<"ATTSPEED"<<'\n';
            }
            else
            {
                m_predators[i]->setPDirection(diff);
                //std::cout<<"punto centroid"<<getCentroid().m_x<<getCentroid().m_y<<getCentroid().m_z<<'\n';
                //std::cout<<getCentroid().length()<<'\n';

                m_predators[i]->setSpeed(5.5);
                std::cout<<m_predators[i]->getSpeed()<<"NORMSPEED"<<'\n';
            }


            /*setPAttack(m_predators[i]);
            m_predators[i]->setPDirection(ExtremeHunt()-m_predators[i]->getPosition());
            //std::cout<<m_predators[i]->getDirection().m_x<<" "<<m_predators[i]->getDirection().m_y<<" "<<m_predators[i]->getDirection().m_z<<'\n';
            m_predators[i]->setSpeed(20.0);
            if(setPAttack(m_predators[i]))
                 kill_count++;
            else
                m_predators[i]->setSpeed(5.5);*/
        /*}
        else
        {

            m_predators[i]->setHunt(m_centroid, m_AvVelocity);
            if(dist<=m_predators[i]->getSearchRad() )
            {
                setPAttack(m_predators[i]);
                //m_predators[i]->setAttack();
                m_predators[i]->setSpeed(10);
                //std::cout<<"there"<<'\n';
                //std::cout<<m_predators[i]->getSpeed()<<"ATTSPEED"<<'\n';
            }
            else
            {
                m_predators[i]->setSpeed(5.5);
                //std::cout<<m_predators[i].getSpeed()<<"NORMSPEED"<<'\n';
            }

        }*/

        //float dist = diff.length();


        //


        //std::cout<<kill_count<<'\n';

    }



}

ngl::Vec3 Flock::ExtremeHunt()
{
    std::vector<ngl::Vec3> _objs;
    std::vector<ngl::Vec3> _objp;
    std::vector<float> _distances;
    ngl::Vec3 _directions;
    std::vector<ngl::Vec3> _directions2;
    getPredators();
    int kill_count=0;
    _distances.clear();
    //_directions.clear();
    _directions2.clear();
    _objp.clear();
    _objs.clear();
    float extremeHuntRadius= 5.0f;

    _directions.set(0.0,0.0,0.0);
    for(unsigned int i=0; i< m_Flock.size(); ++i)
    {
        if(m_Flock[i].m_isPredator==false && m_Flock[i].isPrey==true && m_Flock[i].isDead==false)
        {
            _directions = m_Flock[i].getPosition();
            return _directions;
        }

        /*else
        {


            _directions-=m_Flock[i].getPosition();
            //_directions.normalize();

            m_Flock[i].setPDirection(_directions);
            m_Flock[i].setSpeed(10);
            //m_Flock[i].setSpeed(10);
            float dist = _directions.length();
            if(dist<50)
            {
                    std::cout<<"here"<<_directions.m_x<<'\n';

                    setPAttack(&m_Flock[i]);
                    if(setPAttack(&m_Flock[i]))
                    {
                         kill_count++;
                         dircount--;
                    }
            }
            else
                m_Flock[i].setSpeed(5.5);


        }*/
    }


    //for(unsigned int j=0; j< m_predators.size(); ++j)
    //{
/*
 *
        for(unsigned int i=0; i< m_Flock.size(); ++i)
        {
            if(m_Flock[i].m_isPredator==false && m_Flock[i].isDead==false)
            {
                ngl::Vec3 d_obj=m_Flock[i].getDirection();//getPosition();
                ngl::Vec3 p_obj=m_Flock[i].getPosition();
                //Boid obj(i);
                //obj.setPos(p_obj.m_x,p_obj.m_y,p_obj.m_z);
                _objs.push_back(d_obj);
                _objp.push_back(p_obj);
                poscount++;
            }


            if(m_Flock[i].m_isPredator==true)
            {

               ngl::Vec3 _direction = _objp[poscount]-m_Flock[i].getPosition();
               //if(_direction.length()!=0) _direction.normalize();
               _directions.push_back(_direction);
               //sort(_directions.begin(),_directions.end());

               float dist = _direction.length();
               _distances.push_back(dist);
               dircount++;*/

               //std::vector<float> _distz = sortDistances(_distances);
               /*
               if(_directions2.size()>0)
                    m_Flock[i].setPDirection(_directions2[dircount]);
               //_directions2


               if(dist<m_Flock[i].getSearchRad())
               //if(_distz[kill_count]<getCentroidPredatorDistance())
               {

                       setPAttack(m_predators[j]);
                       if(setPAttack(m_predators[j]))
                            kill_count++;
                            dircount--;
                       m_predators[j]->setSpeed(10);
               }
                       //std::cout<<kill_count<<'\n';
                   else
                       m_predators[j]->setSpeed(5.5);

              */


               //ngl::Vec3 diff = _objs[kill_count]-m_predators[j]->getPosition();
               //m_predators[j].setPDirection(diff);



               /*ngl::Vec3 _direction = m_neighbours[i]->getPosition()-m_Flock[i].getPosition();
               _direction.normalize();
               m_Flock[i].setPDirection(_direction);
               float dist = _direction.length();*/

          //  }

        //}
   // }
        //m_predators[j].setPDirection();//_objs[kill_count]);

        /*
    for(auto &d : _distances)
        std::cout<<"not"<<d<<"    "<<dircount<<'\n';
    sort(_distances.begin(),_distances.end());

    for(auto &d : _distances)
        std::cout<<"sorted"<<d<<"    "<<dircount<<'\n';
    for(unsigned int i=0;i<_distances.size();++i)
    {
        for(unsigned int j=0; j< _directions.size(); ++j)
        {
            if(_distances[i]==_directions[j].length())
            {
                _directions2.push_back(_directions[j]);
            }
        }
    }

    for(unsigned int i=0;i<m_predators.size();++i)
    {
        for(auto &dir : _directions2)
        {
            m_predators[i]->setPDirection(dir);
        }
    }

*/

/*
    for(unsigned int i=0; i< m_predators.size(); ++i)
    {
       //std::cout<<"SPEED"<<'\n';
       ngl::Vec3 diff = _centroid-m_predators[i].getPosition();
       float dist = diff.length();

       if(dist<m_predators[i].getSearchRad())
       {
           m_predators[i].setSpeed(100);
           //std::cout<<m_predators[i].getSpeed()<<"ATTSPEED"<<'\n';
       }
       else
       {
           m_predators[i].setSpeed(-100);
           //std::cout<<m_predators[i].getSpeed()<<"NORMSPEED"<<'\n';
       }
    }
*/

}

void Flock::Phunt()
{

    getCentroid();

    for(unsigned int i=0; i< m_Flock.size(); ++i)
    {
        if(m_Flock[i].m_isPredator==true)
        {
            m_Flock[i].setPDirection(m_centroid);
        }
    }
    getPredators();
    //std::cout<<"hunting centroi"<<getCentroid().m_x<<" "<<getCentroid().m_y<<" "<<getCentroid().m_z<<'\n';
    for(unsigned int i=0; i< m_predators.size(); ++i)
    {

       //std::cout<<"SPEED"<<'\n';
       ngl::Vec3 diff = getCentroid()-m_predators[i]->getPosition();
       float dist = diff.length();
       m_predators[i]->setPDirection(diff);
       /*for(unsigned int i=0; i< m_Flock.size(); ++i)
           {
               if(m_Flock[i].m_isPredator==true)
               {
                   m_Flock[i].setPDirection(diff);
                   //m_Flock[i].setPredatorDirection(diff,m_Flock[i].getVelocity());
               }
           }*/

       if(dist<=m_predators[i]->getSearchRad())
       {
           m_predators[i]->setSpeed(12);
           setPAttack(m_predators[i]);
           //m_predators[i]->setAttack();
           std::cout<<m_predators[i]->getSpeed()<<"ATTSPEED"<<'\n';
           //m_predators[i]->nearestNeighbour(m_predators[i]);
           //std::cout<<m_predators[i]->getSpeed()<<"ATTSPEED"<<'\n';
       }
       else
       {

           //m_predators[i]->setPDirection(diff);
           m_predators[i]->setSpeed(5.5);
           std::cout<<m_predators[i]->getSpeed()<<"NORMSPEED"<<'\n';
           //std::cout<<m_predators[i].getSpeed()<<"NORMSPEED"<<'\n';
       }


    }

}

bool Flock::setPAttack(Boid * m_predator)
{
    ngl::Vec3 _pAttack;
    getCentroid();
    bool kill_flag=false;
    int live_count;
    //std::cout<<"PRED ATT RADIUS"<<m_predator->m_predatorAttRadius<<'\n';
    //int kill_counter=0;
    for(unsigned int i=0; i< m_Flock.size(); ++i)
    {


            if(m_Flock[i].m_isPredator==false && m_Flock[i].isPrey==true)
            {
                //std::cout<<"HERE"<<m_Flock[i].getPosition().length()<<" "<<m_predator->getPosition().length()<<'\n';
                ngl::Vec3 diff = m_Flock[i].getPosition() - m_predator->getPosition();
                diff/=(m_Flock.size()-m_predators.size());
                //diff/=10;
                //diff/=(1+m_predators.size());
                //diff/=2;
                //std::cout<<diff.length()<<'\n';
                float dist = diff.length();
                //float dist = m_Flock[i].getPosition().length() - m_predator->getPosition().length();
                //(m_centroid-m_predator->getPosition()).length()<<'\n';

                if(dist<m_predator->m_predatorRadius/*/(m_Flock.size()-m_predators.size())<(m_predator->m_predatorRadius)*/)
                {
                    if(dist!=0)
                        diff.normalize();
                    ///hunts first boid to get near the predator
                    m_predator->setPDirection(diff);
                    ///anticipates flock, might be usefull for herding
                    //m_predator->setPredatorDirection(diff,m_Flock[i].getVelocity());
                    std::cout<<"hunting first in range"<<m_predator->getDirection().m_x<<" "<<m_predator->getDirection().m_y<<" "<<m_predator->getDirection().m_z<<'\n';
                    ///hunts the nearest within the predators proximity
                    //m_predator->nearestNeighbour(m_predator);
                    //std::cout<<"dist"<<(m_Flock[i].getPosition()-m_predator->getPosition()).length()<<" "<<"id"<< m_Flock[i].getId()<<'\n';

                    //std::cout<<"qui"<<'\n';
                    if(dist<m_Flock[i].m_predatorAttRadius)
                    {

                        m_Flock[i].isDead=true;

                        //m_Flock[i].setVelocity(0.0,-9.8,0.0);
                        _pAttack = _pAttack - diff;
                        //std::cout<<"mpattack"<< dist<<m_Flock[i].getId()<<'\n';
                        kill_flag=true;
                        kill_counter++;

                        std::cout<<"mpattack"<<dist<<"   killkounter"<< kill_counter<<'\n';
                    }
                    else
                        kill_flag=false;

                }


            }



    }

    if(_pAttack.length()!=0)
        _pAttack.normalize();
    _pAttack = _pAttack*100;
    m_predator->m_pAttack=_pAttack;
    if(kill_flag==true)
        return true;
    else
        return false;
}


std::vector<Boid*> Flock::getPredators()
{
   std::vector<Boid*> temp;

   for(int i=0; i<(int)m_Flock.size();++i)
   {
       if(m_Flock[i].m_isPredator==true)
           temp.push_back(&m_Flock[i]);


   }
   m_predators=temp;
   //std::cout<<"predator size"<<m_predators.size()<<m_predators[1]->getSpeed()<<'\n';
   return m_predators;

}
void Flock::Evade(Boid _boid)
{
    //other option is for(Flock[i]  and if  predator
    /*for(int i=0; i<(int)m_predators.size();++i)
    {
        _boid->evade(m_predators[i]);
    }*/
    for(unsigned int i=0; i<m_Flock.size();++i)
    {
        if(_boid.m_isPredator==true && m_Flock[i].m_isPredator==false)
            m_Flock[i].evade(_boid);
    }
}
void Flock::Herd(Boid _boid)
{
    for(unsigned int i=0; i<m_Flock.size();++i)
    {
        if(_boid.m_isPredator==true)
            m_Flock[i].evadeToFlock(_boid);
    }
}

void Flock::ProtectPosition(ngl::Vec3 _position, float _perimeterRadius)
{
    for(unsigned int i=0; i<m_Flock.size();++i)
    {
        if(m_Flock[i].m_isPredator==true)
            m_Flock[i].setGather(_position,_perimeterRadius);
    }
}
float Flock::getHerdBoundingRadius()
{
    m_herdingRadius=400;
    for(unsigned int i=0;i<m_Flock.size();++i)
    {
        if(m_Flock[i].m_isPredator==false)
        {

            ngl::Vec3 _position = m_Flock[i].getPosition();
            ngl::Vec3 _centroid = getCentroid();
            //ngl::Vec3 _origin(0,0,0);
            float dist=(_position-_centroid).length();
            if(dist>m_herdingRadius)
            {
                m_herdingRadius=dist+150;
                return m_herdingRadius;
            }





        }
    }
    return m_herdingRadius;
}

void Flock::updateFlock()
{
  // update octree
  updateOctree();
  // for each boid
  //int live_count=0;

  if(m_export==true)
  {

          //timer.restart();
          m_SimExport.init(m_itr);
          m_SimExport.writeintro(m_Flock.size());
  }
  for(int i=0; i<(int)m_Flock.size();++i)
  {
    // clear the octree results array
    m_octree->clearResults();
    // set the neighbours to the boud
    setNeighbours(i);
    // move the boid

    if(m_Flock[i].m_isPredator==true && getPredators().size()>0)
    {
        ngl::Vec3 origin(300,300,300);

        if(m_HuntA==true)
            Phunt();
        if(m_HuntB==true)
            setEfficientHunt();
        if(m_evadeSheep==true)
            Herd(m_Flock[i]);
        if(m_evadePrey==true)
            Evade(m_Flock[i]);
        if(m_chill==true)
           m_Flock[i].setWander();
        if(m_protectPosition==true)
            ProtectPosition(m_protPos,m_protRadius);
        if(m_gather==true)
            m_Flock[i].setGather(getCentroid(),getHerdBoundingRadius());


        //setEfficientHunt();
        //Phunt();
        //Phunt();
        //m_Flock[i].setSpeed(5.5);
        //ExtremeHunt();
        //m_Flock[i].setWander();


        //m_Flock[i].setGather(getCentroid(),getHerdBoundingRadius());
        //ProtectPosition(m_protPos,200);
        m_Flock[i].movePredator();
        //m_predators[i]->movePredator();
        //getCentroid();

        //Herd(m_Flock[i]);
        //Evade(m_Flock[i]);


    }
    else
    {

        //Evade(&m_Flock[i]);

        m_Flock[i].move();
        //if(m_chill==true)
            //std::cout<<"CHILLING"<<'\n';
        //std::cout<<getCentroid().m_x<<getCentroid().m_y<<getCentroid().m_z<<'\n';
        //ngl::Vec3 ddd = m_Flock[i].getPosition();
        //if(ddd.m_x<10&&ddd.m_y<10&&ddd.m_z<10&&ddd.m_x>-10&&ddd.m_y>-10&&ddd.m_z>-10)
            //std::cout<<"SSsss"<<ddd.m_x<<" "<<ddd.m_y<<" "<<ddd.m_z<<'\n';
        //if(m_Flock[i].getSpeed()>6)
            //std::cout<<m_Flock[i].getSpeed()<<'\n';
        //m_Flock[i].evade(m_predators[i]);

    }

    if(m_export==true)
    {

        //timer.restart();
        m_SimExport.writeStep(m_Flock[i]);

        //exportFrame();
    }

  }
  if(m_export==true)
  {
      m_SimExport.finish();
      ++m_itr;
  }



}

void Flock::exportFrame()
{
    static int s_frame=0;
    char fname[50];
    std::sprintf(fname,"/Users/milovolpicelli/Desktop/Personal Inquiry/BFlock/geo/boid.%03d.geo",s_frame++);
    // we will use a stringstream as it may be more efficient
    std::stringstream ss;
    std::ofstream file;
    file.open(fname);
    if (!file.is_open())
    {
        std::cout << "failed to Open file "<<fname<<'\n';
        exit(EXIT_FAILURE);
    }
    // write header see here http://www.sidefx.com/docs/houdini15.0/io/formats/geo
    ss << "PGEOMETRY V5\n";
    ss << "NPoints " << m_Flock.size() << " NPrims 1\n";
    ss << "NPointGroups 0 NPrimGroups 1\n";
    // this is hard coded but could be flexible we have 1 attrib which is Colour
    ss << "NPointAttrib 1  NVertexAttrib 0 NPrimAttrib 2 NAttrib 0\n";
    // now write out our point attrib this case Cd for diffuse colour
    ss <<"PointAttrib \n";
    // default the colour to white
    ss <<"Cd 3 float 1 1 1\n";
    // now we write out the particle data in the format
    // x y z 1 (attrib so in this case colour)
    for(unsigned int i=0; i<m_Flock.size(); ++i)
    {
      ss<<m_Flock[i].getPosition().m_x<<" "<<m_Flock[i].getPosition().m_y<<" "<<m_Flock[i].getPosition().m_z << " 1 ";

    }
    // now write out the index values
    ss<<"PrimitiveAttrib\n";
    ss<<"generator 1 index 1 location1\n";
    ss<<"dopobject 1 index 1 /obj/AutoDopNetwork:1\n";
    ss<<"Part "<<m_Flock.size()<<" ";
    for(size_t i=0; i<m_Flock.size(); ++i)
    {
      ss<<i<<" ";
    }
    ss<<" [0	0]\n";
    ss<<"box_object1 unordered\n";
    ss<<"1 1\n";
    ss<<"beginExtra\n";
    ss<<"endExtra\n";
    // dump string stream to disk;
    file<<ss.rdbuf();
    file.close();

}
