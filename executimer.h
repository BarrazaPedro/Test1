#ifndef EXECUTIMER_H
#define EXECUTIMER_H

#include <Arduino.h>

class executimer {
public:
  executimer();
  void    start_timer(void);
  void    stop_timer(void);
  int64_t get_elapsed_time(void);

private:
  bool    timer_running;
  int64_t start_time;
  int64_t stop_time;
  int64_t elapsed_time;
};

#endif
