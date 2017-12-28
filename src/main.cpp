#include <uWS/uWS.h>
#include <iostream>
#include "json.hpp"
#include "PID.h"
#include <math.h>
#include <string>
#include <fstream>

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
  }
  else if (b1 != std::string::npos && b2 != std::string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}

int main()
{
  uWS::Hub h;

  PID pid;
  //set a logfile to get the pid parameter
  std::ofstream logfile;
  std::string logfilename = "./pid.log";
  logfile.open(logfilename);
  if (logfile.is_open()) {
        std::cout << "Writing log to" << logfilename << std::endl;
        logfile << "Start PID control\n";
   }
  else
   {
        std::cout << "Failed to open log file " << logfilename << std::endl;
        return 0;
   }
   //Gradient Descent Method
   //double p[3] = {0.0, 0.0, 0.0};
   double p[3] = {0.64473,0.00847472,75.161};
    
   double dp[3] = {0.005, 1e-6, 1.0};
   double gradients[3] = {0.0, 0.0, 0.0};
    // custom step size for 3 different dimensions, increases training speed
    double step_size[3] = {0.05, 1e-8, 1000.0};
    double orig_err;
    int cycle = 0;    // num of trained cycles
    int step = 0;     // num steps trained in this cycle
    //int step_2= 500;
    int step_2=9999;
    int phase=0;
    
    
  // TODO: Initialize the pid variable.

  h.onMessage([&pid, &cycle, &step, &p, &dp, &step_size, &step_2, &gradients, &orig_err, &logfile, &phase](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2')
    {
      auto s = hasData(std::string(data).substr(0, length));
        if (s != "") {
            if (step == 0) {
            
            // force reset at the beginning of each cycle
            std::string reset_msg = "42[\"reset\", {}]";
            ws.send(reset_msg.data(), reset_msg.length(), uWS::OpCode::TEXT);
            
            // reset steering angle and throttle
            json msgJson;
            msgJson["steering_angle"] = 0.0;
            msgJson["throttle"] = 0.4;
            auto msg = "42[\"steer\"," + msgJson.dump() + "]";
            ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
            
            if (phase == 0) {
                pid.Init(p[0], p[1], p[2]);
                logfile << "Start cycle: " << cycle << std::endl;
                
            } else {
                int direction = (phase - 1);
                p[direction] += dp[direction];
                pid.Init(p[0], p[1], p[2]);
                p[direction] -= dp[direction];
            }
            logfile << "\tphase: " << phase <<" pid.Init(" << pid.Kp << ",";
            logfile << pid.Ki << "," << pid.Kd << ")" << std::endl;
        }
            
            // Train PID control
            auto j = json::parse(s);
            std::string event = j[0].get<std::string>();
            if (event == "telemetry") {
                // j[1] is the data JSON object
                double cte = std::stod(j[1]["cte"].get<std::string>());
                double speed = std::stod(j[1]["speed"].get<std::string>());
                double angle = std::stod(j[1]["steering_angle"].get<std::string>());
                /*
                 * TODO: Calcuate steering value here, remember the steering value is
                 * [-1, 1].
                 * NOTE: Feel free to play around with the throttle and speed. Maybe use
                 * another PID controller to control the speed!
                 */
                pid.UpdateError(cte, speed, angle);
                
                json msgJson;
                msgJson["steering_angle"] = pid.steer_value;
                msgJson["throttle"] = pid.throttle;
                auto msg = "42[\"steer\"," + msgJson.dump() + "]";
                ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
            }
            
            step += 1;
            // debug logging
            if (step % 100 == 0) {
                std::cout << "cycle: " << cycle <<  " step: " << step << std::endl;
            }
            
            // on cycle finish, calculate the gradient
            if (step == step_2) {
                // average loss since step_2 increases over cycle
                double pid_err = pid.TotalError() / step_2;
                logfile << "\t  err:" << pid_err << std::endl;
                
                // calculate gradient and save for later update step
                if (phase == 0) {
                    orig_err = pid_err;
                } else {
                    int direction = (phase - 1);
                    double gradient = (pid_err - orig_err) / dp[direction];
                    gradients[direction] = gradient;
                }
                
                // end of cycle, update p[] by gradient
                if (phase == 3) {
                    logfile << "\tupdate p by";
                    for(int i=0; i<3; i++) {
                        double s = step_size[i];
                        double g = gradients[i];
                        p[i] -= s * g;
                        logfile << " " << -s*g << ",";
                    }
                    logfile << std::endl;
                    
                    // setup for next cycle
                    cycle += 1;
                    phase = 0;
                    step = 0;
                    step_2 += 20;
                    return;
                }
                
                phase += 1;
                step = 0;
            }
        }
    else
      {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
    
  });

  // We don't need this since we're not using HTTP but if it's removed the program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data, size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1)
    {
      res->end(s.data(), s.length());
    }
    else
    {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port))
  {
    std::cout << "Listening to port " << port << std::endl;
  }
  else
  {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}
