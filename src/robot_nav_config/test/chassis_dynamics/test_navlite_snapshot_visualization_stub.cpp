// TEST ONLY: this exact std-allocator visualization overload is unresolved in
// the already-deployed plugin; installed Humble exports its xsimd variant.
// visualize=false is mandatory for this test. Never link this file into production.
#include "nav2_mppi_controller/tools/trajectory_visualizer.hpp"
#include <stdexcept>
namespace mppi
{
void TrajectoryVisualizer::add(const xt::xtensor<float,2> &, const std::string &)
{throw std::logic_error("test forbids visualization; installed visualization ABI gap");}
}
