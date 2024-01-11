#include "executimer.h"

executimer::executimer() {
  timer_running = false;
  start_time = 0;
  stop_time = 0;
  elapsed_time = 0;
}

void executimer::start_timer(void)
{
  if (!timer_running) {
    start_time = millis(); 
    timer_running = true;
  }
}

void executimer::stop_timer(void)
{
  if (timer_running) {
    stop_time = millis();
    timer_running = false;
  }
}

int64_t executimer::get_elapsed_time(void)
{
  elapsed_time = stop_time - start_time;
  Serial.printf("Elapsed time: %d ms\n",elapsed_time);
  return elapsed_time;
}
