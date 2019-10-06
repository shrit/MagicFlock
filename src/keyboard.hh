#pragma once

/*  C includes */
extern "C" {
#include <termios.h>
#include <cstdlib>
#include <errno.h>
}
/* Posix includes */
#include <unistd.h>

/* C++ includes */
#include <memory>

class Keyboard {

public:

  Keyboard(int fd);

  int enable_raw_mode(int fd);
  void disable_raw_mode(int fd);

  int poll_event(int fd);

  enum class Special_keys {
			   KEY_NULL = 0,
			   CTRL_C = 3,
			   CTRL_D = 4,
			   CTRL_F = 6,
			   CTRL_H = 8,
			   TAB = 9,
			   CTRL_L = 12,
			   ENTER = 13,
			   CTRL_Q = 17,
			   CTRL_S = 19,
			   CTRL_U = 21,
			   ESC = 27,
			   BACKSPACE =  127,
			   KEY_LEFT = 1000,
			   KEY_RIGHT,
			   KEY_UP,
			   KEY_DOWN,
			   DEL_KEY,
			   HOME_KEY,
			   END_KEY,
			   PAGE_UP,
			   PAGE_DOWN,
  };


  Keyboard(Keyboard const&) = delete;

  Keyboard(Keyboard &&) = default;


private:

  bool raw_mode_;

};
