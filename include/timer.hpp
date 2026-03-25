/* ----------------------------------------------------------------------------
 * Copyright 2025, Kota Kondo, Aerospace Controls Laboratory
 * Massachusetts Institute of Technology
 * All Rights Reserved
 * Authors: Kota Kondo, et al.
 * See LICENSE file for the license information
 * -------------------------------------------------------------------------- */

#pragma once

#include <chrono>
#include <iostream>

namespace timer {

/** @brief Simple high-resolution stopwatch for measuring elapsed time. */
class Timer {
  typedef std::chrono::high_resolution_clock high_resolution_clock;
  typedef std::chrono::milliseconds milliseconds;

 private:
  high_resolution_clock::time_point _start;

 public:
  /** @brief Construct a Timer, optionally starting it immediately.
   *  @param run If true, start the timer upon construction.
   */
  explicit Timer(bool run = false) {
    if (run) Reset();
  }

  /** @brief Reset the timer to the current time. */
  void Reset() { _start = high_resolution_clock::now(); }

  /** @brief Get elapsed time since last reset in milliseconds.
   *  @return Elapsed time in milliseconds.
   */
  double getElapsedMs() const {
    return (std::chrono::duration_cast<milliseconds>(high_resolution_clock::now() - _start))
        .count();
  }

  /** @brief Print elapsed time to stdout with a label.
   *  @param str Label to print before the elapsed time.
   */
  void printMs(const std::string& str) {
    std::cout << str << " " << getElapsedMs() << " ms" << std::endl;
  }

  /** @brief Get elapsed time since last reset in microseconds.
   *  @return Elapsed time in microseconds.
   */
  double getElapsedMicros() const {
    return std::chrono::duration_cast<std::chrono::microseconds>(high_resolution_clock::now() -
                                                                 _start)
        .count();
  }

  /** @brief Stream insertion operator that outputs elapsed time in milliseconds. */
  template <typename T, typename Traits>
  friend std::basic_ostream<T, Traits>& operator<<(std::basic_ostream<T, Traits>& out,
                                                   const Timer& timer) {
    return out << timer.getElapsedMs();
  }
};

}  // namespace timer
