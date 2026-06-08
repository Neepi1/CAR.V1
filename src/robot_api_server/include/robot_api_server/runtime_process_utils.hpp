#pragma once

#include <filesystem>
#include <string>

#include <sys/types.h>

namespace robot_api_server
{

void set_close_on_exec(int fd);
void close_inherited_fds();
void prepare_child_process(const std::string & log_file);

bool is_pid_directory(const std::filesystem::path & path);
std::string read_proc_cmdline(pid_t pid);
std::string read_proc_environ(pid_t pid);
bool process_group_has_live_process(pid_t pgid);
bool process_pid_is_live(pid_t pid);
bool signal_process_group(pid_t pgid, int signal);

}  // namespace robot_api_server
