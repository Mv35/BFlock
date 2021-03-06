#include "NGLScene.h"
#include <iostream>
#include <ngl/Vec3.h>
#include <ngl/Light.h>
#include <ngl/NGLInit.h>
#include <ngl/VAOPrimitives.h>
#include <ngl/Material.h>
#include <ngl/ShaderLib.h>
#include <ngl/SimpleIndexVAO.h>
#include <QColorDialog>
#include <BoidMath.h>

//----------------------------------------------------------------------------------------------------------------------
NGLScene::NGLScene( QWidget *_parent ) : QOpenGLWidget( _parent )
{

  // set this widget to have the initial keyboard focus
  setFocus();
  // re-size the widget to that of the parent (in this case the GLFrame passed in on construction)
  this->resize(_parent->size());
    //m_wireframe=false;
    m_rotation=0.0;
    m_scale=1.0;
    m_position=0.0;

    m_selectedObject=0;
}

// This virtual function is called once before the first call to paintGL() or resizeGL(),
//and then once whenever the widget has been assigned a new QGLContext.
// This function should set up any required OpenGL context rendering flags, defining display lists, etc.

//----------------------------------------------------------------------------------------------------------------------
void NGLScene::initializeGL()
{

  ngl::NGLInit::instance();
  glClearColor(0.4f, 0.4f, 0.4f, 1.0f);			   // Grey Background
  // enable depth testing for drawing
  glEnable(GL_DEPTH_TEST);
  /// create our camera
  ngl::Vec3 eye(0.0f,0.0f,-1000.0f);
  ngl::Vec3 look(0.0f,0.0f,0.0f);
  ngl::Vec3 up(0.0f,1.0f,0.0f);

  m_cam.set(eye,look,up);
  m_cam.setShape(45,float(1024/720),0.1,300);
  // now to load the shader and set the values
  // grab an instance of shader manager
  ngl::ShaderLib *shader=ngl::ShaderLib::instance();
/*
  // we are creating a shader called Phong
  shader->createShaderProgram("Phong");
  // now we are going to create empty shaders for Frag and Vert
  shader->attachShader("PhongVertex",ngl::ShaderType::VERTEX);
  shader->attachShader("PhongFragment",ngl::ShaderType::FRAGMENT);
  // attach the source
  shader->loadShaderSource("PhongVertex","shaders/PhongVertex.glsl");
  shader->loadShaderSource("PhongFragment","shaders/PhongFragment.glsl");
  // compile the shaders
  shader->compileShader("PhongVertex");
  shader->compileShader("PhongFragment");
  // add them to the program
  shader->attachShaderToProgram("Phong","PhongVertex");
  shader->attachShaderToProgram("Phong","PhongFragment");
  // now bind the shader attributes for most NGL primitives we use the following
  // layout attribute 0 is the vertex data (x,y,z)
  shader->bindAttribute("Phong",0,"inVert");
  // attribute 1 is the UV data u,v (if present)
  shader->bindAttribute("Phong",1,"inUV");
  // attribute 2 are the normals x,y,z
  shader->bindAttribute("Phong",2,"inNormal");

  // now we have associated this data we can link the shader
  shader->linkProgramObject("Phong");
  // and make it active ready to load values
  (*shader)["Phong"]->use();
  shader->setUniform("Normalize",1);
  shader->setUniform("viewerPos",m_cam.getEye().toVec3());
  // now pass the modelView and projection values to the shader
  // the shader will use the currently active material and light0 so set them
  ngl::Material m(ngl::STDMAT::POLISHEDSILVER);
  m.loadToShader("material");
  // we need to set a base colour as the material isn't being used for all the params
  shader->setUniform("Colour",0.23125f,0.23125f,0.23125f,1.0f);
*/
  //ngl::ShaderLib *shader=ngl::ShaderLib::instance();
  (*shader)["nglDiffuseShader"]->use();
  shader->setUniform("Colour",1.0f,1.0f,0.0f,1.0f);
  shader->setUniform("lightPos",1.0f,1.0f,1.0f);
  shader->setUniform("lightDiffuse",1.0f,1.0f,1.0f,1.0f);

  ngl::Light light(ngl::Vec3(2,2,2),ngl::Colour(1,1,1,1),ngl::Colour(1,1,1,1),ngl::LightModes::POINTLIGHT);
   //now create our light this is done after the camera so we can pass the
   //transpose of the projection matrix to the light to do correct eye space
   //transformations
  ngl::Mat4 iv=m_cam.getViewMatrix();
  iv.transpose();
  light.setTransform(iv);
  light.setAttenuation(1,0,0);
  light.enable();
   //load these values to the shader as well
  light.loadToShader("light");

  m_boidNum=50;
  m_predatorNum=20;
  m_Flock = new Flock(m_boidNum,m_predatorNum);
  //setSeparation(100);
  //setAlignment(500);
  //setCohesion(100);



  ngl::VAOPrimitives *prim=ngl::VAOPrimitives::instance();
  prim->createSphere("sphere",10,40);
  prim->createCone( "cone",10,-25,4,1);
  m_text.reset(  new  ngl::Text(QFont("Arial",18)));
  m_text->setScreenSize(this->size().width(),this->size().height());
  m_text->setColour(1.0,1.0,0.0);

  startTimer(10);
  m_currentTime= m_currentTime.currentTime();
  // as re-size is not explicitly called we need to do this.
  //glViewport(0,0,m_width,m_height);



  update();
}

//----------------------------------------------------------------------------------------------------------------------
//This virtual function is called whenever the widget has been resized.
// The new size is passed in width and height.
void NGLScene::resizeGL( int _w, int _h )
{
  m_cam.setShape( 45.0f, static_cast<float>( _w ) / _h, 0.05f, 350.0f );
  m_win.width  = static_cast<int>( _w * devicePixelRatio() );
  m_win.height = static_cast<int>( _h * devicePixelRatio() );
}


void NGLScene::loadMatricesToShader()
{
  ngl::ShaderLib *shader=ngl::ShaderLib::instance();
  (*shader)["nglDiffuseShader"]->use();

  ngl::Mat4 MV;
  ngl::Mat4 MVP;
  ngl::Mat3 normalMatrix;
  ngl::Mat4 M;
  M=m_transform.getMatrix();
  MV=m_cam.getViewMatrix()*M;
  MVP=m_cam.getProjectionMatrix()*m_mouseGlobalTX *MV;
  normalMatrix=MV;
  normalMatrix.inverse();
  shader->setUniform("MV",MV);
  shader->setUniform("normalMatrix",normalMatrix);
  shader->setUniform("M",M);
  shader->setUniform("MVP",MVP);

}

//----------------------------------------------------------------------------------------------------------------------
//This virtual function is called whenever the widget needs to be painted.
// this is our main drawing routine
void NGLScene::paintGL()
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glViewport(0,0,m_win.width,m_win.height);


    QTime newTime = m_currentTime.currentTime();
    int msecpassed = m_currentTime.msecsTo(newTime);
    //m_currentTime = newTime;

    ngl::Mat4 rotX;
    ngl::Mat4 rotY;
    // create the rotation matrices
    rotX.rotateX(m_win.spinXFace);
    rotY.rotateY(m_win.spinYFace);

    ngl::VAOPrimitives *prim=ngl::VAOPrimitives::instance();
    /*if(m_wireframe == true)
    {
      glPolygonMode(GL_FRONT_AND_BACK,GL_LINE);
    }
    else
    {
      glPolygonMode(GL_FRONT_AND_BACK,GL_FILL);
    }*/
    ngl::ShaderLib *shader=ngl::ShaderLib::instance();
    (*shader)["nglDiffuseShader"]->use();
    // (*shader)["Phong"]->use();

    m_mouseGlobalTX=rotY*rotX;
    // add the translations
    m_mouseGlobalTX.m_m[3][0] = m_modelPos.m_x;
    m_mouseGlobalTX.m_m[3][1] = m_modelPos.m_y;
    m_mouseGlobalTX.m_m[3][2] = m_modelPos.m_z;
    //starting condition


    for(int i=0; i<m_Flock->getSize(); ++i)
    {
        if(msecpassed>3000)
            m_Flock->m_Flock[i].isPrey=true;

        m_transform.reset();
        m_transform.setPosition(m_Flock->m_Flock[i].getPosition());
        m_transform.addRotation( m_Flock->m_Flock[i].getRotation());
        loadMatricesToShader();

        if(m_Flock->m_Flock[i].m_isPredator==true)
        {

            shader->setUniform("Colour", 0.5f, 5.5f, 0.31f,1.0f);
            prim->draw("cone");

        }
        else
        {

            shader->setUniform("Colour", 1.0f, 0.0f, 0.0f,1.0f);
            //drawBoid();
            prim->draw("cone");
            if(m_Flock->m_Flock[i].isDead==true)
            {

                shader->setUniform("Colour", 0.0f, 0.0f, 1.0f,1.0f);
                m_Flock->m_Flock[i].setSpeed(0.0);
                m_Flock->m_Flock.erase(m_Flock->m_Flock.begin()+i);
                //m_Flock->removeBoid(i);
                //prim->draw("sphere");

                std::cout<<"boidremoved  "<<msecpassed<<" "<<m_Flock->m_Flock[i].getId()<<'\n';
            }
        }

    }

}








NGLScene::~NGLScene()
{
}


void NGLScene::createFlock(int _cohesion, int _separation,
                       int _alignment, float _speed,
                       int _mass)
{
  m_Flock->createFlock(_cohesion, _separation, _alignment, _speed, _mass);
  setFocus();
}
//----------------------------------------------------------------------------------------------------------------------
void NGLScene::removeBoid()
{
  //m_Flock->removeBoid();
  setFocus();
}
//----------------------------------------------------------------------------------------------------------------------
void NGLScene::newFlock(int _numBoids, int _cohesion, int _separation,
                        int _alignment, float _speed, int _mass, int _numPredators, int _predatorSight,float  _predatorAttack, int _predatorAttRadius)
{
  delete m_Flock;
  m_Flock=new Flock(_numBoids, _cohesion, _separation,
                    _alignment, _speed, _mass, _numPredators,_predatorSight, _predatorAttack, _predatorAttRadius);
  setFocus();
}
//----------------------------------------------------------------------------------------------------------------------

void NGLScene::setCohesion(int _cohesion)
{
  for(auto i=0; i<(int)m_Flock->m_Flock.size(); ++i)
  {
    m_Flock->m_Flock[i].setCWeight(_cohesion);
  }
  setFocus();
}
//----------------------------------------------------------------------------------------------------------------------
void NGLScene::setSeparation(int _separation)
{
  for(auto i=0; i<(int)m_Flock->m_Flock.size(); ++i)
  {
    m_Flock->m_Flock[i].setSWeight(_separation);
  }
  setFocus();
}
//----------------------------------------------------------------------------------------------------------------------
void NGLScene::setAlignment(int _align)
{
  for(int i=0; i<(int)m_Flock->m_Flock.size(); ++i)
  {
    m_Flock->m_Flock[i].setAWeight(_align);
  }
  setFocus();
}
//----------------------------------------------------------------------------------------------------------------------
void NGLScene::setSpeed(int _speed)
{
  float speed = _speed/10.0;
  for(int i=0; i<(int)m_Flock->m_Flock.size(); ++i)
  {
    m_Flock->m_Flock[i].setSpeed(speed);
  }
  setFocus();
}
//----------------------------------------------------------------------------------------------------------------------
void NGLScene::setMass(int _mass)
{
  for(int i=0; i<(int)m_Flock->m_Flock.size(); ++i)
  {
    m_Flock->m_Flock[i].setMass(_mass);
  }
  setFocus();
}

void NGLScene::setSepDist(int _sepDist)
{
  for(int i=0;i<(int)m_Flock->m_Flock.size();++i)
  {
    m_Flock->m_Flock[i].setSepDist(_sepDist);
  }
  setFocus();
}

void NGLScene::setEvadeP(bool _evadePray)
{
    if(_evadePray==false)
        m_Flock->m_evadePrey=false;
    else
        m_Flock->m_evadePrey=true;
}

void NGLScene::setEvadeS(bool _evadeSheep)
{



    if(_evadeSheep==false)
    {
        m_Flock->m_evadeSheep=false;
        setCohesion(0);

    }

    else
        m_Flock->m_evadeSheep=true;
}

void NGLScene::setHuntA(bool _huntA)
{
    if(_huntA==false)
    {
        m_Flock->m_HuntA=false;
        m_Flock->m_chill=true;
    }
    else
    {
        m_Flock->m_HuntA=true;
        m_Flock->m_chill=false;
    }
}

void NGLScene::setHuntB(bool _huntB)
{
    if(_huntB==false)
    {
        m_Flock->m_HuntB=false;
        m_Flock->m_chill=true;
    }
    else
    {
        m_Flock->m_HuntB=true;
        m_Flock->m_chill=false;
    }
}

void NGLScene::setChill(bool _chill)
{
    if(_chill==false)
        m_Flock->m_chill=false;
    else
        m_Flock->m_chill=true;
}

void NGLScene::setProtectPosition(bool _pPosition)
{
    if(_pPosition==false)
    {
        m_Flock->m_protectPosition=false;
        m_Flock->m_chill=true;
    }
    else
    {
        m_Flock->m_protectPosition=true;
        m_Flock->m_chill=false;
    }
}

void NGLScene::setGather(bool _gather)
{
    if(_gather==false)
    {
        m_Flock->m_gather=false;
        m_Flock->m_chill=true;
    }
    else
    {
        m_Flock->m_gather=true;
        m_Flock->m_chill=false;
    }
}

void NGLScene::setBoidNum(int _boidNum)
{
    m_boidNum=_boidNum;
    m_Flock->~Flock();
    m_Flock = new Flock(m_boidNum,m_predatorNum);
    update();
}

void NGLScene::setPredatorNum(int _predatorNum)
{
    m_predatorNum=_predatorNum;
    m_Flock->~Flock();
    m_Flock = new Flock(m_boidNum,m_predatorNum);
    update();
}

void NGLScene::setProtectPositionX(double _Xpos)
{
    m_Flock->m_protPos.m_x=_Xpos;
    update();
}

void NGLScene::setProtectPositionY(double _Ypos)
{
    m_Flock->m_protPos.m_y=_Ypos;
    update();
}

void NGLScene::setProtectPositionZ(double _Zpos)
{
    m_Flock->m_protPos.m_z=_Zpos;
    update();
}

void NGLScene::setProtectRadius(double _radius)
{
    m_Flock->m_protRadius=_radius;
    update();
}

void NGLScene::reset()
{
    m_Flock->~Flock();
    m_boidNum=50;
    m_predatorNum=20;
    m_Flock = new Flock(m_boidNum,m_predatorNum);
    update();
}
