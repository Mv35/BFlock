#ifndef NGLSCENE_H_
#define NGLSCENE_H_

#include <ngl/Camera.h>
#include <ngl/Transformation.h>
#include <ngl/Vec3.h>
#include <ngl/Text.h>
#include "WindowParams.h"
#include <QEvent>
#include <QTime>
#include <QResizeEvent>
#include <QOpenGLWidget>
#include <memory>
#include "Flock.h"


/// @file NGLScene.h
/// @brief a basic Qt GL window class for ngl demos
/// @author Jonathan Macey
/// @version 1.0
/// @date 10/10/10
/// Revision History :
/// Initial Version 10/10/10 (Binary day ;-0 )
/// @class GLWindow
/// @brief our main glwindow widget for NGL applications all drawing elements are
/// put in this file
class NGLScene : public QOpenGLWidget
{
Q_OBJECT        // must include this if you use Qt signals/slots
public :
  /// @brief Constructor for GLWindow
  //----------------------------------------------------------------------------------------------------------------------
  /// @brief Constructor for GLWindow
  /// @param [in] _parent the parent window to create the GL context in
  //----------------------------------------------------------------------------------------------------------------------
  NGLScene(QWidget *_parent );

		/// @brief dtor
	~NGLScene();
 public slots :


    void createFlock (int _cohesion, int _separation,
                           int _alignment, float _speed,
                           int _mass);
    void newFlock(int _numBoids, int _cohesion, int _separation, int _alignment, float _speed, int _mass, int _numPredators, int _predatorSight, float _predatorAttack, int _predatorAttRadius);



protected:
  //----------------------------------------------------------------------------------------------------------------------
  /// @brief the windows params such as mouse and rotations etc
  //----------------------------------------------------------------------------------------------------------------------
  WinParams m_win;
  /// @brief  The following methods must be implimented in the sub class
  /// this is called when the window is created
  void initializeGL();

  /// @brief this is called whenever the window is re-sized
  /// @param[in] _w the width of the resized window
  /// @param[in] _h the height of the resized window
  void resizeGL(int _w , int _h);
  /// @brief this is the main gl drawing routine which is called whenever the window needs to
  // be re-drawn
  void paintGL();

  /// @brief our model position
  ngl::Vec3 m_modelPos;
  /// @brief our camera
	ngl::Camera m_cam;
	/// @brief our transform for objects
	ngl::Transformation m_transform;

      ngl::Mat4 m_mouseGlobalTX;




private :
  /// @brief this method is called every time a mouse is moved
  /// @param _event the Qt Event structure

  void mouseMoveEvent (QMouseEvent * _event   );
  /// @brief this method is called everytime the mouse button is pressed
  /// inherited from QObject and overridden here.
  /// @param _event the Qt Event structure

  void mousePressEvent ( QMouseEvent *_event  );

  /// @brief this method is called everytime the mouse button is released
  /// inherited from QObject and overridden here.
  /// @param _event the Qt Event structure
  void mouseReleaseEvent (QMouseEvent *_event );
  void wheelEvent( QWheelEvent* _event );
  void timerEvent(QTimerEvent *_event);
  void loadMatricesToShader( );

  ///@brief instance of the flock
  Flock *m_Flock;


  QTime m_currentTime;


  /// @brief m_wireframe mode
  bool m_wireframe;
  /// @brief rotation data
    ngl::Vec3 m_rotation;
  /// @brief scale data
    ngl::Vec3 m_scale;
  /// @brief position data
    ngl::Vec3 m_position;
  /// @brief our object to draw
  int m_selectedObject;

  ///@brief number of boids (preys)
  int m_boidNum;

  ///@brief number of predators
  int m_predatorNum;

  ///@brief text for rendering
  std::unique_ptr<ngl::Text> m_text;
public slots:

  ///@brief method to reset the simulation
  void reset();

  ///@brief method to set the desired unmber of boids in the simulation
  ///@param [in] _boidNum, numeber of boids
  void setBoidNum(int _boidNum);

  ///@brief method to set the desired unmber of predators in the simulation
  ///@param [in] _predatorNum, numeber of predators
  void setPredatorNum(int _predatorNum);

  ///@brief method to set the X value of the protect position behavior
  ///@param [in] _Xpos, X value
  void setProtectPositionX(double _Xpos);

  ///@brief method to set the Y value of the protect position behavior
  ///@param [in] _Ypos, Y value
  void setProtectPositionY(double _Ypos);

  ///@brief method to set the Z value of the protect position behavior
  ///@param [in] _Zpos, Z value
  void setProtectPositionZ(double _Zpos);

  ///@brief method to set the radius value of the protect position behavior
  ///@param [in] _radius, radius value
  void setProtectRadius(double _radius);

  /// @brief A slot to remove a boid
  void removeBoid();

  /// @brief A slot to set cohesion weight
  /// @param[in] _cohesion the value to set
  void setCohesion(int _cohesion);

  /// @brief A slot to set separation
  /// @param[in] _separation the value to set
  void setSeparation(int _separation);

  /// @brief A slot to set alignment
  /// @param[in] _align the value to set
  void setAlignment(int _align);

  /// @brief A slot to set speed
  /// @param[in] _speed the value to set
  void setSpeed(int _speed);

  /// @brief A slot to set mass
  /// @param[in] _mass the value to set

  void setMass(int _mass);

  /// @brief A slot to set the separation distance for the separation method
  ///@param [in] _sepDist, method to set the separation distance
  void setSepDist(int _sepDist);

  /// @brief A slot to set the centroid based hunting behaviour
  ///@param [in] _huntA, huntA toggle
  void setHuntA(bool _huntA);

  /// @brief A slot to set the centroid and avaraged velocities based hunting behaviour
  ///@param [in] _huntB, huntB toggle
  void setHuntB(bool _huntB);

  /// @brief A slot to set the domesticated sheep evasion behaviour
  ///@param [in] _evadeSheep, evade as domesticated sheep
  void setEvadeS(bool _evadeSheep);

  /// @brief A slot to set the prey evasion behaviour
  ///@param [in] _evadePray, evade as wild prey toggle
  void setEvadeP(bool _evadePray);

  /// @brief A slot to set the do nothing behaviour for predators
  ///@param [in] _chill, chill toggle
  void setChill(bool _chill);

  /// @brief A slot to set the protect position behaviour for predators
  ///@param [in] _pPosition, protect position toggle
  void setProtectPosition(bool _pPosition);

  /// @brief A slot to set the gathering behaviour for predators
  ///@param [in] _gather, gathering toggle
  void setGather(bool _gather);

  ///@brief method to toggle export
  void toggleExport() { m_Flock->m_export^=true;}
};

#endif
