#include "keyboard.hh"

Keyboard::Keyboard(int fd)
{
  enable_raw_mode(fd);
}

Keyboard* ptr;
void call_back_exit(void) {
  ptr->disable_raw_mode(STDIN_FILENO);
}

static struct termios orig_termios; /* In order to restore at exit.*/

int Keyboard::enable_raw_mode(int fd)
{
  struct termios raw;

  if (raw_mode_) return 0; /* Already enabled. */
  if (!isatty(STDIN_FILENO)) goto fatal;
  atexit(call_back_exit);
  if (tcgetattr(fd, &orig_termios) == -1) goto fatal;

  raw = orig_termios;  /* modify the original mode */
  /* input modes: no break, no CR to NL, no parity check, no strip char,
   * no start/stop output control. */
  raw.c_iflag &= ~(BRKINT | ICRNL | INPCK | ISTRIP | IXON);
  /* output modes - disable post processing */
  raw.c_oflag &= ~(OPOST);
  /* control modes - set 8 bit chars */
  raw.c_cflag |= (CS8);
  /* local modes - choing off, canonical off, no extended functions,
   * no signal chars (^Z,^C) */
  raw.c_lflag &= ~(ECHO | ICANON | IEXTEN | ISIG);
  /* control chars - set return condition: min number of bytes and timer. */
  raw.c_cc[VMIN] = 0; /* Return each byte, or zero for timeout. */
  raw.c_cc[VTIME] = 1; /* 100 ms timeout (unit is tens of second). */

  /* put terminal in raw mode after flushing */
  if (tcsetattr(fd,TCSAFLUSH,&raw) < 0) goto fatal;
  raw_mode_ = 1;
  return 0;

 fatal:
  errno = ENOTTY;
  return -1;

}

void Keyboard::disable_raw_mode(int fd)
{
  /* Don't even check the return value as it's too late. */
  if (raw_mode_) {
    tcsetattr(fd,TCSAFLUSH,&orig_termios);
    raw_mode_ = 0;
  }
}

int Keyboard::poll_event(int fd)
{
  int nread;
  char c;
  while ((nread = read(fd,&c,1)) == 0);
  if (nread == -1)
    exit(1);

  return c;

}
