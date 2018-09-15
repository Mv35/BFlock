#ifndef SIMEXPORT
#define SIMEXPORT

#include "Boid.h"

#include "fstream"
#include <string>
#include <sstream>

/// @class SimExport
/// @file SimExport.h

/// @brief SimExport class for writing out the position data to be used in a secondary
/// software in order to visualise such as Houdini. Output files are created per iteration therefore
/// they need to be opened(initialised) and closed every iteration
class SimExport
{
public:

  /// @brief ctor of Class SimExport
  SimExport();

  /// @brief dtor
  ~SimExport();

  /// @brief initialisation of output
  /// @param [in] io_itr, input iteration count
  void init(int io_itr);

  /// @brief closing the output file
  void finish();

  ///@brief write the Houdini format intro
  ///@param [in] _boidNum, total boids number
  void writeintro(int _boidNum);


  /// @brief write the data at each iteration
  /// @param [in] _p, current particle
  void writeStep(Boid _boid);

  /// @brief convert int to string
  /// @param [in] val, integer value to be converted
  /// @param [out] returns the string of the given integer value
  std::string ToString(int io_val);
private:

  /// @brief member variable for file name
  std::string m_filename;

  /// @brief fstream variable to write data in
  std::ofstream m_file;
};

#endif // SIMEXPORT

