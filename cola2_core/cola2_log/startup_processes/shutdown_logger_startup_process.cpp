/*
 * Copyright (c) 2020 Iqua Robotics SL - All Rights Reserved
 *
 * This file is subject to the terms and conditions defined in file
 * 'LICENSE.txt', which is part of this source code package.
 */

#include <cola2_lib/utils/filesystem.h>
#include <csignal>
#include <cstdio>
#include <cstring>
#include <ctime>
#include <chrono>
#include <exception>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <thread>

class ShutdownLogger
{
 private:
  const std::string write_path_;
  const std::string file_name_;

  void write(const std::string& event) const
  {
    try
    {
      if (!cola2::utils::isFileAccessible(write_path_ + file_name_))
        cola2::utils::createDirectory(write_path_);
      std::ofstream log_file;
      const std::string time_stamp = getTimeStamp();
      log_file.open(write_path_ + file_name_, std::ios::out | std::ios::app);
      log_file << event << "_" << time_stamp << "\n";
      log_file.close();
    }
    catch (const std::exception& ex)
    {
      std::cerr << "Write shutdown file failed" << std::endl;
    }
  }

  std::string getTimeStamp() const
  {
    time_t now = time(0);
    char buf[80];
    const struct tm tstruct = *localtime(&now);
    strftime(buf, sizeof(buf), "%Y_%m_%d_%X", &tstruct);
    return std::string(buf);
  }

 public:
  explicit ShutdownLogger(const std::string& user)
    : write_path_(std::string("/home/") + user + std::string("/logs/cola2_log/"))
    , file_name_("shutdown_logger.txt")
  {
    write("boot");
  }

  ~ShutdownLogger()
  {
    write("shutdown");
  }
};

sig_atomic_t volatile shutdown_requested = 0;

void stopHandler(int)
{
  shutdown_requested = 1;
}

int main(int argc, char* argv[])
{
  // Take user from the first argument
  std::string user("user");
  if (argc > 1)
  {
    try
    {
      user = std::string(argv[1]);
    }
    catch (const std::exception& ex)
    {
      std::cout << "Unable to parse user (" << ex.what() << "). Using: " << user << std::endl;
    }
  }

  // Catch shutdown signal
  struct sigaction sa;
  std::memset(&sa, 0, sizeof(struct sigaction));
  sa.sa_handler = stopHandler;
  sa.sa_flags = 0;  // not SA_RESTART
  sigaction(SIGINT, &sa, NULL);
  sigaction(SIGQUIT, &sa, NULL);
  sigaction(SIGABRT, &sa, NULL);
  sigaction(SIGSTOP, &sa, NULL);
  sigaction(SIGTERM, &sa, NULL);
  sigaction(SIGTSTP, &sa, NULL);

  // Create log class
  ShutdownLogger shutdown_logger(user);

  // Wait until shutdown
  while (!shutdown_requested)
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  return 0;
}
