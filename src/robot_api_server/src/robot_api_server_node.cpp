#include "robot_api_server/infrastructure/process/robot_api_process.hpp"

int main(int argc, char **argv) {
  return robot_api_server::infrastructure::process::
      run_robot_api_server_process(argc, argv);
}
