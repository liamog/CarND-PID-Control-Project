#include <uWS/uWS.h>

#include <math.h>
#include <algorithm>
#include <iostream>

#include "PID.h"
#include "json.hpp"

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
std::string hasData(std::string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_last_of("]");
  if (found_null != std::string::npos) {
    return "";
  } else if (b1 != std::string::npos && b2 != std::string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}

int main() {
  uWS::Hub h;

  PID pid_steering("Steering");
  // 0.2005,0.001405,0.02
  pid_steering.SetControlParamsWithTwiddle(
      /*p=*/0.2, /*i=*/0.001, /*d=*/0.01,
      /*delta_Kp=*/0.05, /*delta_Ki=*/0.0005, /*delta_Kd=*/0.005);

  // P only
  // pid_steering.SetControlParams(/*p=*/4.0, /*i=*/0.0, /*d=*/0.0);

  // P + D
   // pid_steering.SetControlParams(/*p=*/0.2, /*i=*/0.0, /*d=*/0.1);

  // P + I + D
  // pid_steering.SetControlParams(/*p=*/0.2, /*i=*/0.005, /*d=*/0.02);



  // FINAL P + I + D
  // pid_steering.SetControlParams(/*p=*/0.2, /*i=*/0.001, /*d=*/0.02);

  PID pid_speed("Speed");
  pid_speed.SetControlParams(/*p=*/0.4, /*i=*/0.0, /*d=*/0.01);

  double target_speed = 30.00;

  int count = 0;

  h.onMessage([&](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                  uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {
      auto s = hasData(std::string(data).substr(0, length));
      if (s != "") {
        auto j = json::parse(s);
        std::string event = j[0].get<std::string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          double cte = std::stod(j[1]["cte"].get<std::string>());
          double speed = std::stod(j[1]["speed"].get<std::string>());
          //          double angle =
          //          std::stod(j[1]["steering_angle"].get<std::string>());
          count++;
          pid_steering.UpdateError(cte);
          double pid_error = pid_steering.TotalError();

          // Clamp the steer angle to [-1, 1]
          double steer_value = std::min<double>(pid_error, 1.0);
          ;
          steer_value = std::max<double>(steer_value, -1.0);

          double speed_error = target_speed - speed;
          pid_speed.UpdateError(speed_error);
          double throttle = -pid_speed.TotalError();
          throttle = std::min<double>(throttle, 0.3);
          throttle = std::max<double>(throttle, -0.3);

          // DEBUG
          // std::cout << "\r(" << count
          //           << ")CTE: " << cte
          //           << " Steering Value: " << steer_value
          //           << " Incoming Angle: " << angle
          //           << " Pid Error: " << pid_error
          //           << " Avg Pid Error: " << pid_steering.best_error()
          //           << "\r";

          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          // std::cout << msg << std::endl;
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                     size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}
