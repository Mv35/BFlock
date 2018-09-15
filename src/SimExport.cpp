#include "SimExport.h"

//ctor
SimExport::SimExport()
{

  m_filename = "/Users/milovolpicelli/Desktop/Personal Inquiry/BFlock/geo/test";
}

//dtor
SimExport::~SimExport()
{
  //the program is closed, write everything into the file now
  m_file.close();
}

//initialise, open the file to write
void SimExport::init(int io_itr)
{
  std::string fileStepName = m_filename+ToString(io_itr)+".geo";
  m_file.open(fileStepName.c_str());
}

void SimExport::writeintro(int _boidNum)
{
        m_file << "PGEOMETRY V5\n";
        m_file << "NPoints " << _boidNum << " NPrims 1\n";
        m_file << "NPointGroups 0 NPrimGroups 1\n";
        //this is hard coded but could be flexible we have 1 att
        m_file << "NPointAttrib 1  NVertexAttrib 0 NPrimAttrib 2 NAttrib 0\n";
        //now write out our point attrib this case Cd for diffus
        m_file << "PointAttrib \n";
        //default the colour to white
        m_file <<"Cd 3 float 1 1 1\n";
        //now we write out the particle data in the format
        //x y z 1 (attrib so in this case colour)
}
//finished, close the file
void SimExport::finish()
{
  m_file.close();
}

//write the data per iteration
void SimExport::writeStep(Boid  _boid)
{
  //std::cout << " " << _boid->getCurrVelo().m_x << " " << _boid->getCurrVelo().m_y << " " << io_p->getCurrVelo().m_z << " " <<  std::endl;
  m_file << _boid.getPosition().m_x << " " << _boid.getPosition().m_y << " " << _boid.getPosition().m_z << " " << std::endl;
  m_file.clear();
}

//convert int value to string
std::string SimExport::ToString(int io_val)
{
    std::stringstream stream;
    stream << io_val;
    return stream.str();
}
