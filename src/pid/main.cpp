#include <cmath>
#include <iostream>

#include <experimental/optional>

#include <uWS/uWS.h>

#include "json/json.hpp"

#include "pid.h"

using std::string;
using std::experimental::optional;

// for convenience
using json = nlohmann::json;

constexpr const int kSpeed = 50;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_last_of("]");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}

template <typename T>
double AbsSum(const T& container) {
  return std::accumulate(container.begin(), container.end(), 0.0,
                         [](double a, double b) { return a + fabs(b); });
}

template <typename T>
void print(const T& container) {
  std::cout << '[';
  bool first = true;

  for (const auto& e : container) {
    if (!first)
      std::cout << ", ";
    else
      first = false;

    std::cout << e;
  }

  std::cout << ']' << std::endl;
}

class Wrapper {
 public:
  Wrapper(uWS::Hub* hub) : hub_(hub) {}

  void Train() { train_ = true; }

  void Run() {
    if (!train_) {
      pid_.Init(0.124612, 0.0703084, 0);
      hub_->run();
      return;
    }

    std::array<double, 3> p = {0.124612, 0.0736716, 0};
    std::array<double, 3> dp = {0.03, 0.015, 0.001};
    double best_err = DoRun(p);

    while (best_err > 0.5) {
      for (int i = 0; i != 2; ++i) {
        p[i] += dp[i];
        DoRun(p);
        if (err_ < best_err) {
          best_err = err_;
          dp[i] *= 1.1;
          continue;
        }

        p[i] -= 2 * dp[i];
        DoRun(p);
        if (err_ < best_err) {
          best_err = err_;
          dp[i] *= 1.1;
          continue;
        }

        p[i] += dp[i];
        dp[i] *= 0.9;
      }
    }

    std::cout << "Final coefficients: ";
    print(p);
    std::cout << "Final error: " << best_err << std::endl;

    hub_->uWS::Group<uWS::SERVER>::terminate();
    hub_->run();
  }

  optional<double> Update(double value, double cte) {
    if (train_) {
      if (steps_ == max_steps_) {
        uv_stop(hub_->getLoop());
        return std::experimental::nullopt;
      } else if (steps_ >= max_steps_ / 2) {
        err_ += std::pow(cte, 2);
      }

      ++steps_;
    }

    return pid_.Update(value, cte);
  }

 private:
  double DoRun(std::array<double, 3> p) {
    steps_ = 0;
    err_ = 0;

    std::cout << "Trying ";
    print(p);
    pid_.Init(p[0], p[1], p[2]);

    hub_->run();
    std::cout << "Error: " << err_ << std::endl;

    return err_;
  }

  uWS::Hub* hub_;
  Pid pid_;
  bool train_ = false;
  int steps_ = 0;
  int max_steps_ = 400;
  double err_ = 0;
};

int main(int argc, char* argv[]) {
  uWS::Hub h;
  Wrapper wrapper(&h);

  Pid throttle_controller;
  throttle_controller.Init(2, 0, 0);

  if (argc >= 2 && argv[1] == string("train")) wrapper.Train();

  h.onMessage([&wrapper, &throttle_controller](uWS::WebSocket<uWS::SERVER> ws,
                                               char* data, size_t length,
                                               uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (!length || length <= 2 || data[0] != '4' || data[1] != '2') return;

    auto s = hasData(string(data));
    if (s.empty()) {
      string msg = "42[\"manual\",{}]";
      ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      return;
    }

    auto j = json::parse(s);
    string event = j[0].get<string>();
    if (event != "telemetry") return;

    // j[1] is the data JSON object
    double cte = std::stod(j[1]["cte"].get<string>());
    double speed = std::stod(j[1]["speed"].get<string>());
    double angle = std::stod(j[1]["steering_angle"].get<string>());

    /*
     * TODO: Calcuate steering value here, remember the steering value is
     * [-1, 1].
     * NOTE: Feel free to play around with the throttle and speed. Maybe
     * use
     * another PID controller to control the speed!
     */
    auto steer_value = wrapper.Update(angle, cte);
    auto throttle = throttle_controller.Update(speed, speed - kSpeed);

    if (steer_value) {
      json msgJson;
      msgJson["steering_angle"] = *steer_value;
      msgJson["throttle"] = throttle;
      auto msg = "42[\"steer\"," + msgJson.dump() + "]";
      ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
    } else {
      string reset_msg = "42[\"reset\",{}]";
      ws.send(reset_msg.data(), reset_msg.length(), uWS::OpCode::TEXT);
      ws.close();
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse* res, uWS::HttpRequest req, char* data,
                     size_t, size_t) {
    const string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {});

  h.onDisconnection([](uWS::WebSocket<uWS::SERVER> ws, int code, char* message,
                       size_t length) {});

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }

  wrapper.Run();
}
