#include "utils.hpp"

#include <cstdlib>

std::string makeBashCommand(const std::string cmd) {
  return "/bin/bash -c '" + cmd + "'";
}

int runBashCommand(const std::string cmd) {
  const std::string bash_cmd = makeBashCommand(cmd);
  return std::system(bash_cmd.c_str());
}