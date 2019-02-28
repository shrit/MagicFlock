#include "log.hh"
#include "global_include.h"


#define ANSI_COLOR_RED "\x1b[31m"
#define ANSI_COLOR_GREEN "\x1b[32m"
#define ANSI_COLOR_YELLOW "\x1b[33m"
#define ANSI_COLOR_BLUE "\x1b[34m"
#define ANSI_COLOR_GRAY "\x1b[37m"
#define ANSI_COLOR_RESET "\x1b[0m"


namespace dronecode_sdk {

void set_color(Color color)
{
  switch (color) {
  case Color::RED:
    std::cout << ANSI_COLOR_RED;
    break;
  case Color::GREEN:
    std::cout << ANSI_COLOR_GREEN;
    break;
  case Color::YELLOW:
    std::cout << ANSI_COLOR_YELLOW;
    break;
  case Color::BLUE:
    std::cout << ANSI_COLOR_BLUE;
    break;
  case Color::GRAY:
    std::cout << ANSI_COLOR_GRAY;
    break;
  case Color::RESET:
    std::cout << ANSI_COLOR_RESET;
    break;
  }
}

} // namespace dronecode_sdk
