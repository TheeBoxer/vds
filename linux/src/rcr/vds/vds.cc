#include <chrono>
#include <thread>

#include <stdio.h>

// justin, put whatever you want here

void test_motor() {
  std::chrono::milliseconds half_second{ 500 };

  std::this_thread::sleep_for(half_second);
  // actuate to 1/4 full extend
  // motor.open_to(EncoderPos::50);

  std::this_thread::sleep_for(half_second);
  // actuate to 2/4 full extend
  // motor.open_to(EncoderPos::100);

  std::this_thread::sleep_for(half_second);
  // actuate to 3/4 full extend
  // motor.open_to(EncoderPos::150);

  std::this_thread::sleep_for(half_second);
  // actuate to full extend
  // motor.open_to(EncoderPos::50);
}



int main() {
  while(1) {
    test_motor();
  }
}
