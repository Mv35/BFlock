#include <ngl/VAOPrimitives.h>
#include <ngl/Random.h>
#include <ngl/ShaderLib.h>
#include "Boid.h"
#include "BoidMath.h"

Boid::Boid(int _id)
{
  //set forces to 0
  setPos(0.0, 0.0, 0.0);
  m_position.m_z = 0.0;
  m_velocity.set(1.0, 0.0, 0.0);
  //m_direction.set(0.0,0.0,0.0);
  m_reposition.set(0,0,0);
  m_id=_id;
  //set weights
  m_alignmentWeight=50;
  m_separationWeight=100;
  m_cohesionWeight=50;
  m_speed=5.5f;
  m_mass=250;
  m_viewRange =10;
  m_avoidWeight = 10;
  m_boundRadius=5;
  m_collisionPos =(0.0,0.0,0.0); //position of collision vec
  m_searchRad = 50;
  m_separationDistance = 10;
  isPrey =false;
  m_maxVel=20.0;

  m_isPredator=false;
  //predator param
  m_predatorsNum = 5;
  m_predatorAttack= 2.0f;
  m_predatorRadius= 10.0f;
  m_predatorAttRadius= 2.0f;
  m_predatorMaxVel= 0.35f;
  m_predatorSight=30.0f;
  m_predatorAllignmentWeight = 100;
  m_predatorCohesionWeight = 100;
  m_predatorSeparationWeight= 100;
  m_pVelocity.set(1.0,0.0,0.0);
  m_pSpeed=5.5f;
  m_predatorMass=250;



}

Boid::~Boid()
{

}

void Boid::nearestNeighbour(Boid * _boid)
{
    std::vector<Boid *> _neighbours=_boid->getNeighb();
    if(_neighbours.size()>0)
    {

        Boid * temp;
        for(unsigned int i=0; i<_neighbours.size(); ++i)
        {
            if(_neighbours[i]->isPrey==true)
            {
                for(unsigned int j=0; j<_neighbours.size(); ++j)
                {
                    float dist1= (_neighbours[i]->getPosition()-_boid->getPosition()).length();
                    float dist2= (_neighbours[j]->getPosition()-_boid->getPosition()).length();

                    if(dist1<dist2)
                    {
                        temp=_neighbours[i];
                        _neighbours[i]=_neighbours[j];
                        _neighbours[j]=temp;
                    }
                    //std::cout<<"has a neighbour with dist:"<<dist1<<'\n';
                    //for(auto &i : _neighbours)
                        //std::cout<<(i->getPosition()-_boid->getPosition()).length()<<" "<<"id"<<" "<<i->getId()<<'\n';
                }
            }

        }
        //if((_neighbours[0]->getPosition()-_boid->getPosition()).length()<m_predatorRadius)


    }
    m_sortedNeighbours=_neighbours;
    setDirectiontoNearestN(_boid);

}

void Boid::setDirectiontoNearestN(Boid *_boid)
{
    for(unsigned int i=0; i<m_sortedNeighbours.size();++i)
    {
        if(m_sortedNeighbours[i]->isPrey==true)
        {
            _boid->setPDirection(m_sortedNeighbours[i]->getPosition());
            std::cout<<"hunting nearest neighbour"<<_boid->getDirection().m_x<<" "<<_boid->getDirection().m_y<<" "<<_boid->getDirection().m_z<<'\n';
        }
        /*if(m_sortedNeighbours[i]->getSpeed()>5.5)
        {
            _boid->setPDirection(m_sortedNeighbours[i+1]->getPosition());
            std::cout<<"next neighbour"<<'\n';
        }*/
        //std::cout<<m_sortedNeighbours[i]->getPosition().m_x<<" "<<m_sortedNeighbours[i]->getPosition().m_y<<" "<<m_sortedNeighbours[i]->getPosition().m_z<<'\n';
        std::cout<<(m_sortedNeighbours[i]->getPosition()- _boid->getPosition()).length()<<'\n';

    }
}
float Boid::getSpeed()
{
    return m_speed;
}
ngl::Vec3 Boid::getFlee() const
{
    return m_flee;
}
std::vector<Boid *> Boid::getPredators()
{
    return m_predators;
}

void Boid::setPos(float _x, float _y, float _z)
{
  m_position.set(_x,_y,_z);
}

void Boid::setNeighbour(Boid *boid)
{

    m_neighbours.push_back(boid);

}
void Boid::setPredators(Boid *_boid)
{
    if(m_isPredator==true)
        m_predators.push_back(_boid);

}
void Boid::clearNeighbour()
{
  //clear the m_neighbours vector
  m_neighbours.clear();
}

void Boid::setVelocity(float _x, float _y, float _z)
{
  m_velocity.set(_x,_y,_z);
}

void Boid::setSWeight(int _separationWeight)
{
  m_separationWeight = _separationWeight;
}

void Boid::setCWeight(int _cohesionWeight)
{
  m_cohesionWeight = _cohesionWeight;
}

void Boid::setAWeight(int _alignWeight)
{
  m_alignmentWeight = _alignWeight;
}

void Boid::setMass(int _mass)
{
  m_mass = _mass;
}
void Boid::setSpeed(float _speed)
{
  //set minimum speed
  m_setSpeed=_speed;
  m_speed=m_setSpeed;
}
/*std::vector<Boid*> Boid::getNeighPred(std::vector<Boid *> _neighbours)
{
    for(unsigned int i = 0; i< _neighbours.size(); ++i)
    {
        if(_neighbours[i]->m_isPredator==true)
            m_neighboursPred.push_back(_neighbours[i]);
    }
    return m_neighboursPred;
}*/
void Boid::setPCohesion()
{
    //int count;
    if(m_neighbours.size()>0)
    {

      for(int i=0;i<(int)m_neighbours.size();i++)
      {
          if(m_neighbours[i]->m_isPredator==true)
            m_pCohesion += m_neighbours[i]->getPosition();
      }    //count++;

      m_pCohesion/=m_neighbours.size();
      if(m_cohesion.length()!=0)
          m_cohesion.normalize();
              //m_cohesion= (m_cohesion-m_neighbours[i]->getPosition())/;
              //setSteering();

          // if the boid has no neighbours then has no cohesion force

      //std::cout<<m_cohesion.m_x<<"cohesion"<<'\n';
      //m_cohesion = -(m_position+m_neighbours.size())/2;

    }
    else m_cohesion.set(0.0f,0.0f,0.0f);

}

void Boid::setCohesion()
{

  //int count;


    for(int i=0;i<(int)m_neighbours.size();i++)
    {
        if(m_neighbours[i]->m_isPredator==false)
        {
            m_cohesion = m_neighbours[i]->getPosition()-m_position;

        }
        if(m_neighbours.size()>0)
            m_cohesion/=m_neighbours.size();
    }    //count++;




    if(m_cohesion.length()!=0)
        m_cohesion.normalize();
            //m_cohesion= (m_cohesion-m_neighbours[i]->getPosition())/;
            //setSteering();

        // if the boid has no neighbours then has no cohesion force

    //std::cout<<m_cohesion.m_x<<"cohesion"<<'\n';
    //m_cohesion = -(m_position+m_neighbours.size())/2;


    else m_cohesion.set(0.0f,0.0f,0.0f);

}

void Boid::setPAlignment()
{
    if(m_neighbours.size()>0)
    {
      for(int i=0;i<(int)m_neighbours.size();++i)
      {
        if(m_neighbours[i]->m_isPredator==true)
            m_pAlignment+=m_neighbours[i]->getVelocity();
      }
      m_pAlignment *= m_neighbours.size();
      if(m_pAlignment.length()!=0)
      {
        m_pAlignment.normalize();
      }
    }
    // if the boid has no neighbours it has no alignment force
    else
      m_pAlignment.set(0,0,0);
}
void Boid::setAlignment()
{
  // alignment force is calculated by averaging the boid's neighbours's velocities
  if(m_neighbours.size()>0)
  {

    for(int i=0;i<(int)m_neighbours.size();++i)
    {
      if(m_neighbours[i]->m_isPredator==false)
      {
        m_alignment+=m_neighbours[i]->getVelocity();
      }
    }
    m_alignment /= m_neighbours.size();
    if(m_alignment.length()!=0)
    {
      m_alignment.normalize();
    }
  }
  // if the boid has no neighbours it has no alignment force
  else
    m_alignment.set(0,0,0);
}

void Boid::setAttack()
{
    //Qtime currentTime;
    //currentTime.currenTime

        m_pAttack=(0.0f,0.0f,0.0f);
        if(m_neighbours.size()>0)
          {
            for(int i=0;i<(int)m_neighbours.size();++i)
            {


                    ngl::Vec3 diff = m_neighbours[i]->getPosition() - m_position;
                    diff/=m_neighbours.size();
                    float dist = diff.length();
                if(isPrey==true)
            {
                if(m_neighbours[i]->m_isPredator==false)
                {
                    if(dist<m_predatorAttRadius )
                    {
                        m_neighbours[i]->isDead=true;
                        m_pAttack = m_pAttack - diff;
                        //std::cout<<"mpattack"<<this->isDead<<'\n';
                    }
                }
            }

         if(m_pAttack.length()!=0)
            m_pAttack.normalize();
         }
        m_pAttack = m_pAttack*100;
    }


}
void Boid::setHunt2(ngl::Vec3 _position)
{
    m_isHunting=true;
    m_hunt= ngl::Vec3(0,0,0);
    m_maxVel = 5.5f;
    //m_slowingRadius = 10.0f;
    ngl::Vec3 _huntingVec = m_hunt-_position;
    float frames= _huntingVec.length() / m_maxVel;
    //m_target= _position + _velocity * frames;

    //ngl::Vec3 _huntVel = m_target - _position;
    //float dist = _huntVel.lenght();
    //if(distance!=0) _huntVel.normalize();

    //_huntVel*= m_maxVel;

    //m_hunt= _huntVel - m_velocity;
    m_hunt= _huntingVec;

}
void Boid::setHunt(ngl::Vec3 _position, ngl::Vec3 _velocity)
{
    //m_isHunting=true;
    m_hunt= ngl::Vec3(0,0,0);
    m_maxVel = 5.5f;
    //m_slowingRadius = 10.0f;
    ngl::Vec3 _huntingVec = m_hunt-_position;
    float frames= _huntingVec.length() / m_maxVel;
    m_target= _position + _velocity * frames;

    ngl::Vec3 _huntVel = m_target - _position;
    //float dist = _huntVel.lenght();
    //if(distance!=0) _huntVel.normalize();

    _huntVel*= m_maxVel;

    m_hunt= _huntVel - m_velocity;

    m_direction+=m_hunt;


}
void Boid::setPSeparation()
{
    m_pSeparation.set(0,0,0);
    for(int i=0;i<(int)m_neighbours.size();++i)
    {

      // if the neighbour is within the boid's separation radius then add a vector from the
      // neighbour to the boid's position and separation vector.
          if(BoidMath::distance(m_position, m_neighbours[i]->getPosition())>m_separationDistance)
          {
            ngl::Vec3 pos(m_neighbours[i]->getPosition()); //current position of neighbour
            ngl::Vec3 dir = (m_position-pos); //the direction
            m_pSeparation += dir; //add direction to separation
          }

    }
    if(m_neighbours.size()>0)
    {
      m_pSeparation /= m_neighbours.size();
    }
    if(m_pSeparation.length()!=0)
    {
      m_pSeparation.normalize();
    }
}
void Boid::setSeparation()
{
  // separation is calculated by taking a vector from each neighbours position to the boid's
  // position and averaging them.

    m_separation.set(0,0,0);
    for(int i=0;i<(int)m_neighbours.size();++i)
    {
      if(m_neighbours[i]->m_isPredator==false)
      {
          // if the neighbour is within the boid's separation radius then add a vector from the
          // neighbour to the boid's position and separation vector.
          if(BoidMath::distance(m_position, m_neighbours[i]->getPosition())>m_separationDistance)
          {
            ngl::Vec3 pos(m_neighbours[i]->getPosition()); //current position of neighbour
            ngl::Vec3 dir = (m_position-pos); //the direction
            m_separation += dir; //add direction to separation
            //std::cout<<"here"<<'\n';
          }
      }

    }
    if(m_neighbours.size()>0)
    {
      m_separation /= m_neighbours.size();
      //m_separation *=(-1);
    }
    if(m_separation.length()!=0)
    {
      m_separation.normalize();
    }

}
void Boid::setFlee(ngl::Vec3 _fleePos)
{
    /*for(int i=0; i<(int)m_neighbours.size();++i)
    {
      //Collision(m_neighbours[i]->getPosition(), m_neighbours[i]->getRadius());

      if(m_neighbours[i]->m_isPredator==true)
      {
        ngl::Vec3 diff= m_neighbours[i]->getPosition() - m_position;
        float dist = diff.length();
        if(dist< m_predatorAttRadius)
        {
            m_flee= (m_flee-diff)*m_avoidWeight;
        }




       }
      m_flee=(0.0f,0.0f,0.0f);
    }
    */

    if((m_position - _fleePos).length()< 100 && m_isPredator==false)
    {
        m_flee = m_position - _fleePos;
        //m_flee-=_fleePos;
        //if(m_cohesionWeight<200 && isPrey==true)
            //m_cohesionWeight+=0.1;
        m_speed=10.0f;
        //setSpeed(10);
        //m_velocity+=m_flee;
        //m_speed*=100;//m_speed=500;
        //if(m_flee.length()!=0)
        isFleeing=true;
        //m_cohesionWeight+=1;
        //std::cout<<m_cohesionWeight<<'\n';
        std::cout<<"fleeing"<<'\n';
    }
    else
    {
        m_flee.set(0,0,0);
        m_speed=5.5;
        isFleeing=false;
    }
    if(m_flee.length()!=0) m_flee.normalize();
    //std::cout<<m_flee.m_x<<m_flee.m_y<<m_flee.m_z<<"m_flee"<<" "<<"id"<<m_id<<'\n';
    //std::cout<<"fleeing"<<'\n';
}

void Boid::getGathered(ngl::Vec3 _fleePos)
{
    if((m_position - _fleePos).length()< 100 && m_isPredator==false)
    {
        m_flee = m_position - _fleePos;
        m_flee-=_fleePos;
        if(m_cohesionWeight<200 && isPrey==true)
            m_cohesionWeight+=0.1;
        m_speed=10.0f;
        //setSpeed(10);
        //m_velocity+=m_flee;
        //m_speed*=100;//m_speed=500;
        //if(m_flee.length()!=0)
        isFleeing=true;
        //m_cohesionWeight+=1;
        //std::cout<<m_cohesionWeight<<'\n';

    }
    else
    {
        m_flee.set(0,0,0);
        m_speed=5.5;
        isFleeing=false;
    }
    if(m_flee.length()!=0) m_flee.normalize();
}
void Boid::evadeToFlock(Boid _boid)
{
    //std::cout<<_boid.getPosition().m_x<<'\n';
    if(_boid.m_isPredator==true)
    {
        ngl::Vec3 dist = _boid.getPosition() - m_position;
        int frames = dist.length() / m_maxVel;
        m_target = _boid.getPosition() + _boid.getVelocity() * frames;
        //setFlee(_boid.getPosition());
        getGathered(_boid.getPosition());
    }


    //std::cout<<"evading"<<_boid.getId()<<_boid.m_isPredator<<" "<<'\n';

}

void Boid::evade(Boid _boid)
{
    //std::cout<<_boid.getPosition().m_x<<'\n';
    if(_boid.m_isPredator==true)
    {
        ngl::Vec3 dist = _boid.getPosition() - m_position;
        int frames = dist.length() / m_maxVel;
        m_target = _boid.getPosition() + _boid.getVelocity() * frames;
        setFlee(_boid.getPosition());
    }


    //std::cout<<"evading"<<_boid.getId()<<_boid.m_isPredator<<" "<<'\n';

}
void Boid::setPAvoid()
{
    if(m_velocity.length()!=0)
    {
      m_velocity.normalize();
    }
}
void Boid::setAvoid()
{
  //check if boids collide with eachother
  for(int i=0; i<(int)m_neighbours.size();++i)
  {
    if(m_neighbours[i]->m_isPredator==false)
        Collision(m_neighbours[i]->getPosition(), m_neighbours[i]->getRadius());
  }
  if(m_velocity.length()!=0)
  {
    m_velocity.normalize();
  }
  // if there is a collision, calculate an avoidance force as a vector
  // from the collision position to the ahead vector
  m_avoid=(0,0,0);
    //  ngl::Vec3 ahead(m_position + m_velocity * m_viewRange * m_speed);
  if(m_collisionPos.length() != 0)
  {
    m_avoid=((m_position+m_velocity*(m_viewRange*2)*(m_speed/2))-m_collisionPos);
    if(m_avoid.length()!=0)
    {
      m_avoid.normalize();
    }
  }
  else
  {
    m_avoid*=10;
  }
  m_collisionPos = 0;
}

void Boid::setReposition(ngl::Vec3 _reposition)
{
  // the reposition force is calculated as a vector from the reposition position
  // to the boid
  if(BoidMath::distance(_reposition,m_position)<50)
  {
    m_reposition=-(_reposition-m_position);
    if(m_reposition.length()!=0)
    {
      m_reposition.normalize();
    }
  }
}

void Boid::repositionBounds()
{
  // if a boid raoms outside of the BBox,it repositions itself to turn around
  if(m_position.m_x<=-350 || m_position.m_x>=350)
  {
    m_reposition.m_x-=(m_position.m_x/2);
  }
  if(m_position.m_y<=-350 || m_position.m_y>=350)
  {
    m_reposition.m_y-=(m_position.m_y/2);
  }
  if(m_position.m_z<=-350 || m_position.m_z>=350)
  {
    m_reposition.m_z-=(m_position.m_z/2);
  }
  // if a boid wanders too far out of BBox range,it spawns at it's origin(0,0,0)

  /*if(m_position.m_x<=-750 || m_position.m_x>=750)
  {
    m_reposition.set(0,0,0);
    m_position.m_x=m_reposition.m_x;
  }
  if(m_position.m_y<=-750 || m_position.m_y>=750)
  {
      m_reposition.set(0,0,0);
      m_position.m_y=-(m_reposition.m_y);
  }
  if(m_position.m_z<=-750 || m_position.m_z>=750)
  {
      m_reposition.set(0,0,0);
    m_position.m_z=-(m_reposition.m_z);
  }*/

}
void Boid::setGather(ngl::Vec3 _position, float _radius)
{
    m_direction.set(0,0,0);
    // scale by avoid weight
    m_avoid*=m_avoidWeight;
    // the direction is calculated by the reposition and avoidance
    //m_direction=m_reposition;//+m_avoid;

    float m_wanderAngle=ngl::Random::instance()->randomNumber() * 10;
    float m_wanderAzimuth=ngl::Random::instance()->randomNumber() * 5;

    //m_velocity+=(sphereRadius-center).cross(sphereRadius).cross((sphereRadius-center));
    ngl::Vec3 sphereCenter = m_velocity;
    //std::cout<<m_velocity.m_x<<" "<<m_velocity.m_y<<" "<<m_velocity.m_z<<'\n';
        if (sphereCenter.length() !=0)
        {
            sphereCenter.normalize();
        }
    sphereCenter *= _radius;
    ngl::Vec3 _displace;

    _displace.m_x +=  _position.m_x + _radius * cos(m_wanderAngle) * sin(m_wanderAzimuth);
    _displace.m_y +=  _position.m_y + _radius * sin(m_wanderAngle)  * sin(m_wanderAzimuth);
    _displace.m_z +=  _position.m_z + _radius * cos(m_wanderAzimuth);

    for(double i=0.0;i<360; i+=0.1)
    {
        m_wanderAngle += i;//0.01;

    }

    for(double j=-90.0;j<90;j+=0.1)
    {
        m_wanderAzimuth +=j;
        //std::cout<<"J"<<j<<'\n';
    }

    m_wander=_displace-m_position+sphereCenter;

    m_direction+= m_wander;
}
void Boid::setWander()
{
    ngl::Vec3 center(0,0,0);
    m_direction.set(0,0,0);
    // scale by avoid weight
    m_avoid*=m_avoidWeight;
    // the direction is calculated by the reposition and avoidance
    m_direction=m_reposition;//+m_avoid;
    float m_wanderAngleChange=2.0;
    float m_wanderAzimuthChange=4.0f;
    float m_wanderSphereRadius=500.0f;
    ngl::Vec3 sphereRadius(5,5,5);
    float m_wanderAngle=ngl::Random::instance()->randomNumber() * 10;
    float m_wanderAzimuth=ngl::Random::instance()->randomNumber() * 5;
    //ngl::Vec3 sphereCenter = m_velocity;

    //m_velocity.set(0,0,0);
    //m_velocity+=(sphereRadius-center).cross(sphereRadius).cross((sphereRadius-center));

    ngl::Vec3 sphereCenter = m_velocity;
    //std::cout<<m_velocity.m_x<<" "<<m_velocity.m_y<<" "<<m_velocity.m_z<<'\n';
        if (sphereCenter.length() !=0)
        {
            sphereCenter.normalize();
        }
    sphereCenter *= 50.0f;
    ngl::Vec3 _displace;

    _displace.m_x += 0.0 + m_wanderSphereRadius * cos(m_wanderAngle) * sin(m_wanderAzimuth);
    _displace.m_y += 0.0 + m_wanderSphereRadius * sin(m_wanderAngle)  * sin(m_wanderAzimuth);
    _displace.m_z += 0.0 + m_wanderSphereRadius * cos(m_wanderAzimuth);


    //_displace.m_x = 0.0 + cos(m_wanderAngle)*50*sin(m_wanderAzimuth);
    //_displace.m_y = 0.0 + sin(m_wanderAngle)*50*sin(m_wanderAzimuth);
    //_displace.m_z = 0.0 + 50* cos(m_wanderAzimuth);
    //if(m_wanderAngle<360 && m_wanderAngle>0)

    //double i=0.0;
    for(double i=0.0;i<360; i+=0.01)
    {
        m_wanderAngle += i;//0.01;
        //std::cout<<"I"<<i<<'\n';

    }
    //ngl::Random::instance()->randomNumber() * m_wanderAngleChange - m_wanderAngleChange * 0.5f;
    //if(m_wanderAzimuth>360 && m_wanderAzimuth< 0)
        //m_wanderAzimuth +=  0.01;//ngl::Random::instance()->randomNumber() * m_wanderAzimuthChange - m_wanderAzimuthChange * 1.0f;
    //double j=0.0;
    for(double j=-90.0;j<90;j+=0.01)
    {
        m_wanderAzimuth +=j;
        //std::cout<<"J"<<j<<'\n';
    }
    //m_wander= _displace;//+sphereCenter;
    m_wander=_displace+sphereCenter;

    m_direction+= m_wander;//+sphereCenter;

    //std::cout<<_displace.length()<<'\n';
    //if(m_position.m_x<100 && m_position.m_x>-100 && m_position.m_y <100 && m_position.m_y>-100 && m_position.m_z<100 && m_position.m_z>-100)
      //  std::cout<<"predator in flock"<<m_id<<'\n';


    //std::cout<<"dist"<<(m_position-center).length()<<'\n';
    //std::cout<<sphereCenter.m_x<<" "<<sphereCenter.m_y<<" "<<sphereCenter.m_z<<'\n';
}

void Boid::setPDirection(ngl::Vec3 _direction)
{
    m_direction.set(0,0,0);
    // scale by avoid weight
    m_avoid*=m_avoidWeight;
    // the direction is calculated by the reposition and avoidance
    m_direction=m_reposition+m_avoid;
    // scale the behaviour forces by their weights
    //m_separation*=m_separationWeight;
    //m_alignment*=m_alignmentWeight;
    //m_cohesion*=m_cohesionWeight;
    //m_flee*=m_avoidWeight;
    // set the direction by combining the weighted forces

    m_direction+=_direction;//+m_reposition+m_avoid;

    //m_direction+=/*m_separation+m_cohesion+m_alignment+m_avoid+m_reposition+m_pAttackm_reposition;*/
}

void Boid::setPredatorDirection(ngl::Vec3 _direction, ngl::Vec3 _velocity)
{
    m_direction.set(0,0,0);
    int frames = m_direction.length()/m_speed;
    m_target= _direction+_velocity*frames;

    ngl::Vec3 desiredVel = m_target = m_position;
    desiredVel*=m_speed;

    ngl::Vec3 desiredDir = desiredVel - m_velocity;

    // the direction is calculated by the reposition and avoidance
    m_direction=m_reposition;

    /*if(m_isHunting==true)
    {
        m_direction+=m_hunt;
    }
    else*/
    m_direction+=m_direction+desiredDir;
}


void Boid::setDirection()
{
m_direction.set(0,0,0);
//m_flee.set(0,0,0);
// scale by avoid weight
m_avoid*=m_avoidWeight;
// the direction is calculated by the reposition and avoidance
m_direction=m_reposition+m_avoid;
// scale the behaviour forces by their weights
m_separation*=m_separationWeight;
m_alignment*=m_alignmentWeight;
m_cohesion*=m_cohesionWeight;
//getPredators();
//for(unsigned int i=0; i < getPredators().size(); ++i)
//{

   //evade(m_predators[i]);
   //std::cout<<"ev"<<'\n';

//}

if(isFleeing==true)
{
    m_flee*=m_avoidWeight;
    m_direction+=m_flee;
}
else
{
    m_flee*=0;
    m_direction+=m_separation+m_cohesion+m_alignment+m_avoid+m_reposition+m_flee;
}
// set the direction by combining the weighted forces
//std::cout<<"ev"<<m_predators.size()<<'\n';
//m_flee.set(1,1,1);


}

void Boid::setSteering()
{
  // set a steering vector from the current velocity to the dir
  m_steering = m_direction-m_velocity;
  if(m_steering.length()!=0)
    m_steering.normalize();
}

void Boid::setPSteering()
{
    m_steering = m_direction;//-m_pVelocity;
    if(m_steering.length()!=0)
      m_steering.normalize();
}

void Boid::updatePosition()
{
  // update the velocity by adding the steering vector divided by the mass
  m_velocity = m_velocity+(m_steering/m_mass);
  // scale it by the speed
  if(m_velocity.length()!=0)
  {
    m_velocity.normalize();
    m_velocity*=m_speed;
  }
  // add the velocity to the current position to update it
  m_position = m_position+m_velocity;
}

void Boid::updatePPosition()
{
  // update the velocity by adding the steering vector divided by the mass
  m_pVelocity = m_pVelocity+(m_pSteering/m_predatorMass);
  // scale it by the speed
  if(m_pVelocity.length()!=0)
  {
    m_pVelocity.normalize();
    m_pVelocity*=m_pSpeed;
  }
  // add the velocity to the current position to update it
  m_position = m_position+m_pVelocity;
}

void Boid::setPRotate()
{
  m_yaw = atan2(m_pVelocity.m_x,m_pVelocity.m_z)*180/M_PI+180;
  m_pitch = atan2(m_pVelocity.m_y,sqrt(m_pVelocity.m_x*m_pVelocity.m_x+m_pVelocity.m_z*m_pVelocity.m_z))*180/M_PI;
}
void Boid::setRotate()
{
  m_yaw = atan2(m_velocity.m_x,m_velocity.m_z)*180/M_PI+180;
  m_pitch = atan2(m_velocity.m_y,sqrt(m_velocity.m_x*m_velocity.m_x+m_velocity.m_z*m_velocity.m_z))*180/M_PI;
}
    //combine all movement methods

void Boid::move()
{

    m_reposition.set(0,0,0);
    //find neighbours

    if(m_neighbours.size()<=1 && m_searchRad<350)
    {
    m_searchRad*=2;
    }

    if(m_searchRad<=!10 && m_neighbours.size()>=5)
    {
    m_searchRad/=2;
    }

    //set behaviour
    setSeparation();
    setAlignment();
    setCohesion();
    //collision control
    setAvoid();


    //evade();
    //setFlee();
    repositionBounds();

    //calculate new direction if a collision has occured
    setDirection();
    setSteering();
    //update
    updatePosition();
    setRotate();
    //std::cout<<m_flee.m_x<<m_flee.m_y<<m_flee.m_z<<"m_flee"<<" "<<"id"<<m_id<<'\n';
    //std::cout<<m_cohesion.m_x<<m_cohesion.m_y<<m_cohesion.m_z<<"m_cohesion"<<" "<<"id"<<m_id<<'\n';

}

void Boid::movePredator()
{
    m_reposition.set(0,0,0);
    //find neighbours

    if(m_neighbours.size()<=1 && m_searchRad<350)
    {
    m_searchRad*=2;
    }

    if(m_searchRad<=!10 && m_neighbours.size()>=5)
    {
    m_searchRad/=2;
    }
    //setAlignment();
    //setCohesion();
    //setSeparation();
    //setAvoid();
    //setFlee();
    //setAttack();
    setPAvoid();

    repositionBounds();
    //calculate new direction if a collision has occured
    //setPDirection(_centerofmass);
    //setPredatorDirection();

    setSteering();
    //update
    updatePosition();
    setRotate();
}

void Boid::Collision(ngl::Vec3 _pos, float _rad)
{
  ngl::Vec3 ahead(m_position + m_velocity * m_viewRange * m_speed);
  bool collision = BoidMath::collisionDetect(ahead, _pos, m_position, _rad);
  //if a a collision is detected
  if(collision==true  && (m_collisionPos.length()==0 || BoidMath::distance(m_position, _pos) < BoidMath::distance(m_position, m_collisionPos)))
  {
    m_collisionPos=_pos;
  }
}

