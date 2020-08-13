#include <ros/ros.h>

// Service Robot
namespace hsr { 

enum Exit {
  SUCCESS = 0, FAIL, INVALID
};

// Specifies current status of the Service Robot
namespace Status {
  enum class ID {IDLE = 0, STARTING, SERVICE, FINISH };

  std::map<ID, std::string> Id2Name = {
    {ID::IDLE, "IDLE"},
    {ID::STARTING, "STARTING"},
    {ID::SERVICE, "SERVICE"},
    {ID::FINISH, "FINISH"}
  };

  std::map<std::string, ID> Name2Id = {
    {"IDLE", ID::IDLE},
    {"STARTING", ID::STARTING},
    {"SERVICE", ID::SERVICE},
    {"FINISH", ID::FINISH}
  };
} // End of namespace Status
} // End of namespace hsr