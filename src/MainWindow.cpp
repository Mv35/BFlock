#include "MainWindow.h"
#include "ui_MainWindow.h"

MainWindow::MainWindow(QWidget *parent) :QMainWindow(parent), m_ui(new Ui::MainWindow)
{
  m_ui->setupUi(this);

  m_gl=new  NGLScene(this);

  m_ui->s_mainWindowGridLayout->addWidget(m_gl,0,0,2,1);

  connect(m_ui->cohesion,SIGNAL(valueChanged(int)),m_gl,SLOT(setCohesion(int)));
  connect(m_ui->separation,SIGNAL(valueChanged(int)),m_gl,SLOT(setSeparation(int)));
  connect(m_ui->alignment,SIGNAL(valueChanged(int)),m_gl,SLOT(setAlignment(int)));

  connect(m_ui->evadeP,SIGNAL(toggled(bool)),m_gl,SLOT(setEvadeP(bool)));
  connect(m_ui->evadeS,SIGNAL(toggled(bool)),m_gl,SLOT(setEvadeS(bool)));
  connect(m_ui->huntA,SIGNAL(toggled(bool)),m_gl,SLOT(setHuntA(bool)));
  connect(m_ui->huntB,SIGNAL(toggled(bool)),m_gl,SLOT(setHuntB(bool)));
  connect(m_ui->gather,SIGNAL(toggled(bool)),m_gl,SLOT(setGather(bool)));
  connect(m_ui->chill,SIGNAL(toggled(bool)),m_gl,SLOT(setChill(bool)));
  connect(m_ui->protPos,SIGNAL(toggled(bool)),m_gl,SLOT(setProtectPosition(bool)));

  connect(m_ui->boidnum,SIGNAL(valueChanged(int)),m_gl,SLOT(setBoidNum(int)));
  connect(m_ui->predatornum,SIGNAL(valueChanged(int)),m_gl,SLOT(setPredatorNum(int)));
  connect(m_ui->m_positionX,SIGNAL(valueChanged(double)),m_gl,SLOT(setProtectPositionX(double)));
  connect(m_ui->m_positionY,SIGNAL(valueChanged(double)),m_gl,SLOT(setProtectPositionY(double)));
  connect(m_ui->m_positionZ,SIGNAL(valueChanged(double)),m_gl,SLOT(setProtectPositionZ(double)));
  connect(m_ui->protRadius,SIGNAL(valueChanged(double)),m_gl,SLOT(setProtectRadius(double)));

  connect(m_ui->reset,SIGNAL(clicked(bool)),m_gl,SLOT(reset()));

  connect(m_ui->exportData,SIGNAL(clicked(bool)),m_gl,SLOT(toggleExport()));

}

MainWindow::~MainWindow()
{
    delete m_ui;
}
