#ifndef __QUADROTOR_MSGS_QUADROTOR_MSGS_H__
#define __QUADROTOR_MSGS_QUADROTOR_MSGS_H__

#include <quadrotor_msgs/Gains.h>
#include <quadrotor_msgs/SO3Command.h>
#include <quadrotor_msgs/TRPYCommand.h>
#include <stdint.h>

#include <vector>

namespace quadrotor_msgs {

void encodeSO3Command(const quadrotor_msgs::SO3Command& so3_command,
                      std::vector<uint8_t>& output);
void encodeTRPYCommand(const quadrotor_msgs::TRPYCommand& trpy_command,
                       std::vector<uint8_t>& output);

void encodePPRGains(const quadrotor_msgs::Gains& gains,
                    std::vector<uint8_t>& output);
}  // namespace quadrotor_msgs

#endif
