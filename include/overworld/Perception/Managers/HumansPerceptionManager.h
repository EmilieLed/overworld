#ifndef OWDS_HUMANSPERCEPTIONMANAGER_H
#define OWDS_HUMANSPERCEPTIONMANAGER_H

#include "overworld/Perception/Managers/AgentPerceptionManager.h"

namespace owds {

class HumansPerceptionManager : public AgentPerceptionManager
{
public:
  explicit HumansPerceptionManager(ros::NodeHandle* nh): AgentPerceptionManager(nh){}
  
  Agent* getAgent(const std::string& agent_name, bool auto_create = true) { return AgentPerceptionManager::getAgent(agent_name, AgentType_e::HUMAN, auto_create); }

private:
  void getPercepts( std::map<std::string, BodyPart>& percepts) override;
};

} // namespace owds

#endif // OWDS_HUMANSPERCEPTIONMANAGER_H