/*
 * Copyright (c) 2020 Iqua Robotics SL - All Rights Reserved
 *
 * This file is subject to the terms and conditions defined in file
 * 'LICENSE.txt', which is part of this source code package.
 */

#include "ros/ros.h"
#include "sensor_msgs/Joy.h"

#include <termios.h>
#include <unistd.h>

#include <csignal>
#include <cstdio>
#include <cstring>
#include <vector>

// Flag used to stop the node
sig_atomic_t volatile keyboard_requested_shutdown = 0;

class Keyboard
{
protected:
  // Node handle
  ros::NodeHandle nh_;

  // Publisher
  ros::Publisher pub_;

  // Get char internal method
  int getChar()
  {
    struct termios oldt, newt;
    tcgetattr(STDIN_FILENO, &oldt);
    std::memcpy(&newt, &oldt, sizeof(struct termios));
    newt.c_lflag &= ~(ICANON | ECHO);
    newt.c_lflag |= ISIG;
    newt.c_cc[VEOL] = 1;
    newt.c_cc[VEOF] = 2;
    newt.c_cc[VMIN] = 1;
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    const int ch = getchar();  // This call blocks until a key is pressed
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    return ch;
  }

public:
  Keyboard()
  {
    pub_ = nh_.advertise<sensor_msgs::Joy>("joy", 1);
    ROS_INFO("Initialized");
  }

  void readKeyboardHits()
  {
    // Forward, backward, turn left and turn right
    const int KEY_W = 87;
    const int KEY_w = 119;
    const int KEY_S = 83;
    const int KEY_s = 115;
    const int KEY_A = 65;
    const int KEY_a = 97;
    const int KEY_D = 68;
    const int KEY_d = 100;

    // For yaw and heave velocities
    const int KEY_T = 84;
    const int KEY_t = 116;
    const int KEY_G = 71;
    const int KEY_g = 103;
    const int KEY_F = 70;
    const int KEY_f = 102;
    const int KEY_H = 72;
    const int KEY_h = 104;

    // Up and down
    const int KEY_UP = 65;
    const int KEY_DOWN = 66;
    const int KEY_RIGHT = 67;
    const int KEY_LEFT = 68;

    // Pitch
    const int KEY_O = 79;
    const int KEY_o = 111;
    const int KEY_K = 75;
    const int KEY_k = 107;

    // Pitch mode
    const int KEY_M = 77;
    const int KEY_m = 109;
    const int KEY_N = 78;
    const int KEY_n = 110;

    // Esc and Space control actions
    const int KEY_ESC = 27;
    const int KEY_OBRACKET = 91;
    const int KEY_SPACE = 32;
    const int KEY_DOT = 46;
    const int KEY_COMA = 44;

    while (!keyboard_requested_shutdown)
    {
      // Initialize buttons state
      std::vector<int> buttons(19, 0);  // 19 buttons

      // Find which button is pressed
      const int key = getChar();
      if (keyboard_requested_shutdown)
      {
        break;
      }

      if (key == KEY_SPACE)
      {
        buttons[0] = 1;
      }
      else if (key == KEY_W || key == KEY_w)
      {
        buttons[1] = 1;
      }
      else if (key == KEY_S || key == KEY_s)
      {
        buttons[2] = 1;
      }
      else if (key == KEY_A || key == KEY_a)
      {
        buttons[3] = 1;
      }
      else if (key == KEY_D || key == KEY_d)
      {
        buttons[4] = 1;
      }
      else if (key == KEY_ESC)
      {
        if (getChar() == KEY_OBRACKET)
        {
          const int arrow_key = getChar();
          if (keyboard_requested_shutdown)
          {
            break;
          }

          if (arrow_key == KEY_UP)
          {
            buttons[5] = 1;
          }
          else if (arrow_key == KEY_DOWN)
          {
            buttons[6] = 1;
          }
          else if (arrow_key == KEY_RIGHT)
          {
            buttons[7] = 1;
          }
          else if (arrow_key == KEY_LEFT)
          {
            buttons[8] = 1;
          }
        }
      }
      else if (key == KEY_O || key == KEY_o)
      {
        buttons[9] = 1;
      }
      else if (key == KEY_K || key == KEY_k)
      {
        buttons[10] = 1;
      }
      else if (key == KEY_DOT)
      {
        buttons[11] = 1;
      }
      else if (key == KEY_COMA)
      {
        buttons[12] = 1;
      }
      else if (key == KEY_T || key == KEY_t)
      {
        buttons[13] = 1;
      }
      else if (key == KEY_G || key == KEY_g)
      {
        buttons[14] = 1;
      }
      else if (key == KEY_F || key == KEY_f)
      {
        buttons[15] = 1;
      }
      else if (key == KEY_H || key == KEY_h)
      {
        buttons[16] = 1;
      }
      else if (key == KEY_M || key == KEY_m)
      {
        buttons[17] = 1;
      }
      else if (key == KEY_N || key == KEY_n)
      {
        buttons[18] = 1;
      }
      else
      {
        ROS_INFO_STREAM("Key code is " << key << ". Ignoring key");
        ros::Duration(0.01).sleep();  // Avoid filling the hard drive if getchar returns immediately on error
        continue;
      }

      // Publish buttons
      sensor_msgs::Joy msg;
      msg.header.stamp = ros::Time::now();
      msg.header.frame_id = "keyboard";
      msg.buttons.resize(buttons.size());
      for (std::size_t i = 0; i < buttons.size(); ++i)
      {
        msg.buttons[i] = buttons[i];
      }
      pub_.publish(msg);
      ros::spinOnce();
      msg.header.stamp = ros::Time::now();
      for (std::size_t i = 0; i < buttons.size(); ++i)
      {
        msg.buttons[i] = 0;
      }
      pub_.publish(msg);  // Everything back to 0
      ros::spinOnce();
    }
  }
};

// Signal handler
void keyboardStopHandler(int)
{
  keyboard_requested_shutdown = 1;
}

int main(int argc, char **argv)
{
  // Init node without signal handler
  ros::init(argc, argv, "keyboard", ros::init_options::NoSigintHandler);

  // Handle signals so that getchar returns and we can restore the terminal
  struct sigaction sa;
  std::memset(&sa, 0, sizeof(struct sigaction));
  sa.sa_handler = keyboardStopHandler;
  sa.sa_flags = 0;  // not SA_RESTART
  sigaction(SIGINT, &sa, NULL);
  sigaction(SIGTERM, &sa, NULL);

  // Read keyboard hits
  Keyboard keyboard;
  keyboard.readKeyboardHits();

  // Quit
  ros::shutdown();
  return 0;
}
