#ifndef TIMER_H_
#define TIMER_H_

#include <glog/logging.h>
#include <algorithm>
#include <chrono>
#include <fstream>
#include <map>
#include <numeric>
#include <string>

class Timer {
 public:
  struct TimerRecord {
    TimerRecord() = default;
    TimerRecord(const std::string& name, double time_usage) {
      func_name_ = name;
      time_usage_in_ms_.emplace_back(time_usage);
    }
    std::string func_name_;
    std::vector<double> time_usage_in_ms_;
  };

  template <class F>
  void Evaluate(F&& func, const std::string& func_name) {
    auto t1 = std::chrono::high_resolution_clock::now();
    std::forward<F>(func)();
    auto t2 = std::chrono::high_resolution_clock::now();
    auto time_used =
        std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1)
            .count() *
        1000;
    if (time_used > 100.0) {
      LOG(WARNING) << "> [ " << func_name << " ] slow stage: " << time_used
                   << " ms";
    }
    if (records_.find(func_name) != records_.end()) {
      records_[func_name].time_usage_in_ms_.emplace_back(time_used);
    } else {
      records_.insert({func_name, TimerRecord(func_name, time_used)});
    }
  }

  void PrintAll() {
    LOG(INFO) << ">>> ===== Printing run time =====";
    for (const auto& r : records_) {
      const auto& samples = r.second.time_usage_in_ms_;
      const double average = std::accumulate(samples.begin(), samples.end(), 0.0) /
                             static_cast<double>(samples.size());
      const double maximum = *std::max_element(samples.begin(), samples.end());
      auto sorted_samples = samples;
      std::sort(sorted_samples.begin(), sorted_samples.end());
      const std::size_t p99_index = std::min(
          sorted_samples.size() - 1,
          static_cast<std::size_t>(0.99 * static_cast<double>(sorted_samples.size())));
      LOG(INFO) << "> [ " << r.first << " ] average time usage: " << average
                << " ms , p99: " << sorted_samples[p99_index]
                << " ms , max: " << maximum
                << " ms , called times: " << samples.size();
    }
  }

  /// clean the records
  void Clear() { records_.clear(); }

 private:
  std::map<std::string, TimerRecord> records_;
};

#endif
