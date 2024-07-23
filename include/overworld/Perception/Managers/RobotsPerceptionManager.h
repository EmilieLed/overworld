#ifndef OWDS_ROBOTSPERCEPTIONMANAGER_H
#define OWDS_ROBOTSPERCEPTIONMANAGER_H

#include "overworld/Perception/Managers/AgentPerceptionManager.h"

namespace owds {

class RobotsPerceptionManager : public AgentPerceptionManager
{
public:
  explicit RobotsPerceptionManager(ros::NodeHandle* nh): AgentPerceptionManager(nh){}
  ~RobotsPerceptionManager() {}

  Agent* getAgent(const std::string& agent_name, bool auto_create = true) { return AgentPerceptionManager::getAgent(agent_name, AgentType_e::ROBOT, auto_create); }

private:
  void getPercepts( std::map<std::string, BodyPart>& percepts) override;
};

} // namespace owds

#endif // OWDS_ROBOTSPERCEPTIONMANAGER_H