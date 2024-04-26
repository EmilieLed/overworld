#ifndef OVERWORLD_RELATIONSSENDER_H
#define OVERWORLD_RELATIONSSENDER_H

#include "overworld/Perception/Managers/ObjectsPerceptionManager.h"
#include "overworld/Perception/Managers/HumansPerceptionManager.h"
#include "overworld/Perception/Managers/RobotsPerceptionManager.h"
#include "overworld/Facts/Fact.h"

#include "overworld/GetRelations.h"

#include <ros/service_server.h>

namespace owds {

struct ToCompute_t
{
  ToCompute_t(bool egocentric = false) : deitic_relation(false),
                                         intrinsic_relation(false),
                                         egocentric_relation(egocentric) {}
  bool deitic_relation;
  bool intrinsic_relation;
  bool egocentric_relation;
  std::string subject;
};

struct ComputedRelation_t
{
  ComputedRelation_t() : last_complete_use(0) {}
  uint64_t last_complete_use;
  std::map<std::string, uint64_t> last_indiv_use;
};

struct ComputedRelations_t
{
  ComputedRelations_t() : last_use(0) {}
  uint64_t last_use;
  ComputedRelation_t computed_egocentric_relation;
  ComputedRelation_t computed_deictic_relation;
  ComputedRelation_t computed_intrinsic_relation;
};

class RelationsSender
{
public:
  RelationsSender(ros::NodeHandle* nh,
                  const std::string& agent_name,
                  ObjectsPerceptionManager* objects_manager,
                  HumansPerceptionManager* humans_manager,
                  RobotsPerceptionManager* robots_manager);

  void posesUpdated() { frames_++; }

private:
  uint64_t frames_;
  std::unordered_map<std::string, ComputedRelations_t> last_use_;
  std::unordered_map<std::string, std::unordered_set<Fact>> last_facts_;// nul

  ros::ServiceServer get_relations_service_;

  std::string agent_name_;
  ObjectsPerceptionManager* objects_manager_;
  HumansPerceptionManager* humans_manager_;
  RobotsPerceptionManager* robots_manager_;
  BodyPart* agent_; // the agent is represented by one of its body part

  void getAgent();

  bool onGetRelationService(overworld::GetRelations::Request& request,
                            overworld::GetRelations::Response& response);
  void initOrigin(const overworld::GetRelations::Request& request);
  std::vector<ToCompute_t> shouldRecompute(const overworld::GetRelations::Request& request);
  bool shouldRecompute(const std::string& subject, ComputedRelation_t& computed_relation);

  void computeRelationOnAll(const overworld::Triplet& pattern, overworld::GetRelations::Response& response,const std::string origin_id,bool first_it, bool deictic, bool intrinsic);
  void computeRelationOnOne(const overworld::Triplet& pattern, overworld::GetRelations::Response& response,const std::string origin_id,bool first_it, bool deictic, bool intrinsic);
  void computeOneRelation(Object* object_a, overworld::GetRelations::Response& response,const std::string origin_id,bool first_it, bool deictic, bool intrinsic);
  void computeDeicticRelation(Object* object_a, Object* object_b, overworld::GetRelations::Response& response,const std::string origin);

  void filterTriplets(const overworld::Triplet& triplet,overworld::GetRelations::Response& response,const std::string origin);

  bool shouldBeTested(Object* object);
  bool shouldBeTestedForDeictic(Object* object_a,Object* object_b, bool first_it);
  bool isNextTo(Object* object_a, Object* object_b);
  bool InOverlappingOnZ(Object* object_a, Object* object_b);
};

} // namespace owds

#endif // OVERWORLD_RELATIONSSENDER_H