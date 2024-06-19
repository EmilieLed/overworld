#include "overworld/Senders/RelationsSender.h"

#define DELTA_FRAME 5
#define MIN_DISTANCE 10

namespace owds {

RelationsSender::RelationsSender(ros::NodeHandle* nh,
                                const std::string& agent_name,
                                 ObjectsPerceptionManager* objects_manager,
                                 HumansPerceptionManager* humans_manager,
                                 RobotsPerceptionManager* robots_manager) : frames_(0),
                                                                            agent_name_(agent_name),
                                                                            objects_manager_(objects_manager),
                                                                            humans_manager_(humans_manager),
                                                                            robots_manager_(robots_manager),
                                                                            agent_(nullptr)
{
  get_relations_service_ = nh->advertiseService("/overworld/get_relations/" + agent_name, &RelationsSender::onGetRelationService, this);
}

void RelationsSender::getAgent()
{
  auto agent = robots_manager_->getAgent(agent_name_, false);
  if(agent == nullptr)
    agent = humans_manager_->getAgent(agent_name_, false);
  if(agent == nullptr)
    ShellDisplay::error("[RelationsSender] Agent " + agent_name_ + " does not exist. The sender will have no effect.");
  else
  {
    agent_ = agent->getTorso();
    if(agent_ == nullptr)
    {
      agent_ = agent->getBase();
      if(agent_ == nullptr)
      {
        agent_ = agent->getHead();
        if(agent_ == nullptr)
          ShellDisplay::error("[RelationsSender] Agent " + agent_name_ + " has no torso, base nor head. The sender will have no effect.");
      }
    }
  }
}

bool RelationsSender::onGetRelationService(overworld::GetRelations::Request& request,
                                           overworld::GetRelations::Response& response)
{
  std::cout << request.origin_id << std::endl;
  getAgent();
  if(agent_ == nullptr)
    return true;

  std::cout << "Passe dans onGetRelationService" << std::endl;

  bool first_it=true;
  initOrigin(request);
  if (last_facts_.at(request.origin_id).empty())
    bool first_it=true;
  std::vector<ToCompute_t> should_compute = shouldRecompute(request);
  std::cout <<"should_compute_size:"<<should_compute.size() << std::endl;
  for(size_t i = 0; i < should_compute.size(); i++)
  {
    if(should_compute[i].subject == "")
    {
      std::cout << "computeRelationOnAll" << std::endl;
      computeRelationOnAll(request.patterns[i], response,request.origin_id,first_it,
                           (should_compute[i].deitic_relation || should_compute[i].egocentric_relation),
                           (should_compute[i].intrinsic_relation || should_compute[i].egocentric_relation));
      std::cout << "go past computeRelationOnAll" << std::endl;
      if(should_compute[i].deitic_relation)
        last_use_[request.origin_id].computed_deictic_relation.last_complete_use = frames_;
      else if(should_compute[i].intrinsic_relation)
        last_use_[request.origin_id].computed_intrinsic_relation.last_complete_use = frames_;
      else if(should_compute[i].egocentric_relation)
        last_use_[request.origin_id].computed_egocentric_relation.last_complete_use = frames_;
    }
    else
    {
      std::cout << "computeRelationOnOne" << std::endl;
      computeRelationOnOne(request.patterns[i], response,request.origin_id,first_it,
                           (should_compute[i].deitic_relation || should_compute[i].egocentric_relation),
                           (should_compute[i].intrinsic_relation || should_compute[i].egocentric_relation));
      if(should_compute[i].deitic_relation)
        last_use_[request.origin_id].computed_deictic_relation.last_indiv_use[should_compute[i].subject] = frames_;
      else if(should_compute[i].intrinsic_relation)
        last_use_[request.origin_id].computed_intrinsic_relation.last_indiv_use[should_compute[i].subject] = frames_;
      else if(should_compute[i].egocentric_relation)
        last_use_[request.origin_id].computed_egocentric_relation.last_indiv_use[should_compute[i].subject] = frames_;
    } 
  }
  std::cout<<"Liste last_facts à la fin de OnGetRelationService:"<<std::endl;
  for(const auto& fact : last_facts_[request.origin_id])
    {
      std::cout << fact.getSubject()<<fact.getPredicate()<<fact.getObject()<< std::endl;
    }
  std::cout << "-----"<< std::endl;
  
  return true;
}

void RelationsSender::initOrigin(const overworld::GetRelations::Request& request)
{
  auto it = last_use_.find(request.origin_id);
  if(it == last_use_.end())
  {
    last_use_[request.origin_id] = ComputedRelations_t();
    last_facts_[request.origin_id] = {};
    
  }
}

std::vector<ToCompute_t> RelationsSender::shouldRecompute(const overworld::GetRelations::Request& request)
{
  auto it = last_use_.find(request.origin_id);
  if(it == last_use_.end()) return std::vector<ToCompute_t>(request.patterns.size(), true); //voir si necessaire
  std::vector<ToCompute_t> res;
  for(auto& pattern : request.patterns)
  {
    ToCompute_t to_compute;
    to_compute.subject = pattern.subject;
    if(pattern.predicate == "egocentricGeometricalProperty")
      to_compute.egocentric_relation = shouldRecompute(pattern.subject, it->second.computed_egocentric_relation);
    else if((pattern.predicate == "isAtRightOf") || (pattern.predicate == "isAtLeftOf") ||
            (pattern.predicate == "isInFrontOf") || (pattern.predicate == "isBehind"))
      to_compute.deitic_relation = shouldRecompute(pattern.subject, it->second.computed_deictic_relation);
    else if(pattern.predicate == "deicticGeometricalProperty")
      to_compute.deitic_relation = shouldRecompute(pattern.object, it->second.computed_deictic_relation);
    else if((pattern.predicate == "isToTheRightOf") || (pattern.predicate == "isToTheLeftOf") ||
            (pattern.predicate == "isAtTheFrontOf") || (pattern.predicate == "isAtTheBack"))
      to_compute.intrinsic_relation = shouldRecompute(pattern.subject, it->second.computed_intrinsic_relation);
    else if(pattern.predicate == "intrinsicGeometricalProperty")
      to_compute.intrinsic_relation = shouldRecompute(pattern.object, it->second.computed_intrinsic_relation);
      
    res.push_back(to_compute);
  }

  return res;
}

bool RelationsSender::shouldRecompute(const std::string& subject, ComputedRelation_t& computed_relation)
{
  if(subject != "")
  {
    auto indiv_it = computed_relation.last_indiv_use.find(subject);
    if(indiv_it == computed_relation.last_indiv_use.end())
    {
      computed_relation.last_indiv_use[subject] = frames_;
      return true;
    }
    else if(indiv_it->second + DELTA_FRAME < frames_)
    {
      indiv_it->second = frames_;
      return true;
    }
    else
      return false;
  }
  else if(computed_relation.last_complete_use + DELTA_FRAME < frames_)
  {
    computed_relation.last_complete_use = frames_;
    return true;
  }
  else
    return false;
}

void RelationsSender::computeRelationOnAll(const overworld::Triplet& pattern, overworld::GetRelations::Response& response,const std::string origin_id,bool first_it, bool deictic, bool intrinsic)
{
  if(agent_->isLocated() == false)
    { 
    std::cout << "agent not located" << std::endl;
    return;
    }
  if (!last_facts_.at(origin_id).empty()) first_it=false;

  auto objects = objects_manager_->getEntities(); 
  for(auto object_a_it = objects.begin(); object_a_it != objects.end(); ++object_a_it)
  {

    std::cout << "test all " << object_a_it->first << std::endl;
    computeOneRelation(object_a_it->second,response,origin_id,first_it,deictic,intrinsic);
  }
}

void RelationsSender::computeRelationOnOne(const overworld::Triplet& pattern, overworld::GetRelations::Response& response,const std::string origin_id,bool first_it, bool deictic, bool intrinsic)
{
  if(agent_->isLocated() == false)
    {
    std::cout << "agent not located" << std::endl;
    return;
    }

  auto objects = objects_manager_->getEntities();
  auto ref_object_it = objects.find(pattern.subject);
  if(ref_object_it == objects.end())
    return;

  if(shouldBeTested(ref_object_it->second) == false)  //shouldBeTested(a)
    return;

  std::cout << "test one" << ref_object_it->first << std::endl;
  computeOneRelation(ref_object_it->second,response,origin_id,first_it,deictic,intrinsic);
}


void RelationsSender::computeOneRelation(Object* object_a, overworld::GetRelations::Response& response,const std::string origin_id,bool first_it, bool deictic, bool intrinsic)
{
  shouldBeTested(object_a);
  std::cout << "test one" << object_a<< std::endl;
  auto objects = objects_manager_->getEntities();
  //updatePresentObjects(response,origin_id);
    for(auto object_b = objects.begin(); object_b != objects.end(); ++object_b) //for all b
  {
    if(object_a != object_b->second) 
    {
      if(deictic)
      { 
        computeDeicticRelation(object_a, object_b->second, response,origin_id);
        // if (shouldBeTestedForDeictic(object_a, object_b->second,first_it))
        //   computeDeicticRelation(object_a, object_b->second, response,origin_id);

        //TO DO: Utiliser shouldBeTestedForDeictic pour vérifier qu'un objet a bouger suffisament pour recalculer sa position. 
        //Probleme: il faut pouvoir autoriser un premier passage, sinon, on ne calcule jamais
      }
      else if(intrinsic)
        continue;

      else continue;
    }
  }
}

void RelationsSender::computeDeicticRelation(Object* object_a, Object* object_b, overworld::GetRelations::Response& response,const std::string origin)
{
  if(shouldBeTested(object_b) == false)
      return;

  if(isOverlappingOnZ(object_a, object_b))
    if(overlapXY(object_a->getAabb(), object_b->getAabb()) == false)
      if(isNextTo(object_a, object_b))
      {
        // agent to object_a
        std::pair<double, double> v = {object_a->pose().getX() - agent_->pose().getX(),
                                        object_a->pose().getY() - agent_->pose().getY()};
        // object_a to object_b
        std::pair<double, double> u = {object_b->pose().getX() - object_a->pose().getX(),
                                        object_b->pose().getY() - object_a->pose().getY()};

        double angle = atan2(u.second, u.first) - atan2(v.second, v.first); // [-2pi, +2pi]

        overworld::Triplet triplet;
        triplet.subject = object_b->id();
        triplet.object = object_a->id();
        if (angle<0)
          angle=2*M_PI+angle; // [0, +2pi]   
        if((angle < M_PI_4) || (angle > 7*M_PI_4))  
          triplet.predicate = "isBehind";
        else if((angle <= 3*M_PI_4) && (angle >= M_PI_4))
          triplet.predicate = "isAtLeftOf";
        else if((angle >= 5*M_PI_4) && (angle <= 7*M_PI_4))
          triplet.predicate = "isAtRightOf";
        else 
          triplet.predicate = "isInFrontOf";
        std::cout<<"angle calculé avec objet a "<<triplet.subject<<" et objet b "<<triplet.object<<":"<<angle<<std::endl;
        std::cout << "computeDeicticRelation:"<<triplet.subject<<" "<<triplet.predicate<<" "<<triplet.object<< std::endl;
        filterTriplets(triplet,response,origin);
        return;
 
      }

  //if a fact that contain both objects while they aren't nextTo each other exist, it need to be erased
  clearFactBetweenTwoObjects(object_a,object_b,response,origin);
}

void RelationsSender::computeIntrinsicRelation(Object* object_a, Object* object_b, overworld::GetRelations::Response& response,const std::string origin)
{
    if(shouldBeTested(object_b) == false)
      return;

  if(isOverlappingOnZ(object_a, object_b))
  {
    if(isNextTo(object_a, object_b))
    {

    }
  }

}

void RelationsSender::filterTriplets(const overworld::Triplet& triplet, overworld::GetRelations::Response& response,const std::string origin)
{
  Fact new_fact(triplet);

  std::vector<overworld::Triplet> to_delete_temp;
  bool already_exist = false;

  for(const auto& fact : last_facts_[origin])
  {
    if (new_fact.useSameEntities(fact) && (new_fact.getPredicate() != fact.getPredicate()))
      {
        to_delete_temp.push_back(fact.toTriplet()); 
        std::vector<overworld::Triplet> equivalent_delete=returnEquivalent(fact.toTriplet(),origin);
        for(auto& triplet_i:equivalent_delete)
          {to_delete_temp.push_back(triplet_i);
          std::cout<<"suppresion du triplet faux equivalent:" <<triplet_i.subject<<" "<<triplet_i.predicate<<" "<<triplet_i.object<<std::endl; }
      }

    if (new_fact.useSameEntitiesCrossed(fact) && (new_fact.getPredicate() == fact.getPredicate())) //suppression des triplets avec le même prédicat (double inFrontOf)
    {
      to_delete_temp.push_back(fact.toTriplet()); 
        std::vector<overworld::Triplet> equivalent_delete=returnEquivalent(fact.toTriplet(),origin);
        for(auto& triplet_i:equivalent_delete)
          {to_delete_temp.push_back(triplet_i);
          std::cout<<"suppresion du triplet en double:" <<triplet_i.subject<<" "<<triplet_i.predicate<<" "<<triplet_i.object<<std::endl;
          }
    }

    
    if (new_fact == fact)
      {
        already_exist = true; 
      std::cout << "fact existe deja"<< std::endl;
      }
    
    if (isSameFactCrossed(new_fact,fact)) //(ex: a LeftOf b = b RightOf a)
      {
        std::cout<<"fact equivalent existe déjà"<<std::endl;
        std::cout<<"test isSameFactCrossed"<<isSameFactCrossed(new_fact,fact)<<std::endl;
        
        already_exist=true;
      }
      
  }

  if(already_exist == false)
  {
    response.to_add.push_back(triplet);
    last_facts_[origin].insert(new_fact);
    std::cout << "ajout du triplet dans to_add: "<<triplet.subject<<" "<<triplet.predicate<<" "<<triplet.object<< std::endl;
  }
  for (auto triplet_i : to_delete_temp)
  {
    response.to_delete.push_back(triplet_i);
    Fact fact_i(triplet_i);
    last_facts_[origin].erase(fact_i);
    std::cout << "suppresion du triplet:" <<triplet_i.subject<<" "<<triplet_i.predicate<<" "<<triplet_i.object<< std::endl;
  }
}


void RelationsSender::clearFactBetweenTwoObjects(Object* object_a, Object* object_b, overworld::GetRelations::Response& response,const std::string origin)
{
  std::vector<overworld::Triplet> to_delete_temp;
  
  for(const auto& fact : last_facts_[origin])
  {
    if((fact.getSubject()==object_a->id() && fact.getObject()==object_b->id())||(fact.getSubject()==object_b->id() && fact.getObject()==object_a->id()))
    {
      to_delete_temp.push_back(fact.toTriplet()); 
    }
  }

  for (auto triplet_i : to_delete_temp)
  {
    response.to_delete.push_back(triplet_i);
    Fact fact_i(triplet_i);
    last_facts_[origin].erase(fact_i);
    std::cout << "suppresion du triplet parce que les objets ne sont plus à coté:" <<triplet_i.subject<<" "<<triplet_i.predicate<<" "<<triplet_i.object<< std::endl;
  }
 
}

bool RelationsSender::isSameFactCrossed(const Fact& fact_a,const Fact& fact_b)
{
  return (fact_a.useSameEntitiesCrossed(fact_b) && getInversePredicate(fact_b)==fact_a.getPredicate());
}

std::string RelationsSender::getInversePredicate(const Fact& checked_fact) const
{
    if (checked_fact.getPredicate()=="isAtRightOf")
      return "isAtLeftOf";
    else if (checked_fact.getPredicate()=="isAtLeftOf")
      return "isAtRightOf";
    else if (checked_fact.getPredicate()=="isBehind")
      return "isInFrontOf";
    else if (checked_fact.getPredicate()=="isInFrontOf")
      return "isBehind";
    else 
      return checked_fact.getPredicate(); 
}

std::vector<overworld::Triplet> RelationsSender::returnEquivalent(const overworld::Triplet& triplet,const std::string origin) const
{
  //pour un triplet, renvoie la liste de tout les triplets equivalents 
  // On l'utilisera pour determiner si un triplet est necessaire (si un equivalant est déjà là, on peut ne pas l'ajouter)
  //ou pour supprimer des triplets equivalent dun triplet devenu faux
  
  Fact checked_fact(triplet);
  std::cout<<"passe dans returnEquivalent"<<std::endl;
  std::vector<overworld::Triplet> res;
  for(const Fact &fact : last_facts_.at(origin))
  {
    if (checked_fact.useSameEntitiesCrossed(fact) && (getInversePredicate(fact)==checked_fact.getPredicate()))
    {
      res.push_back(fact.toTriplet());
      std::cout<<"fact equivalent:"<<fact.getSubject()<<" "<<fact.getPredicate()<<" "<<fact.getObject()<<" "<<std::endl;
      
    }
    return res; 
  }

  return res;
}

void RelationsSender::updatePresentObjects(overworld::GetRelations::Response& response,const std::string origin)
{
  std::vector<overworld::Triplet> to_delete_temp;
  //mettre à jour la liste d'entités
  auto objects_present=objects_manager_->getEntities();

  for(const Fact &fact : last_facts_.at(origin))
  {
    
    //si ni le sujet ni l'objet ne sont présents dans la liste des entités présentes, mettre le fact dans la liste des objets à supprimer
    auto it1=objects_present.find(fact.getSubject());
    auto it2=objects_present.find(fact.getObject());

    if (it1==objects_present.end()&&it2==objects_present.end()) //On d'utiliser or mais on pourrait utiliser and ?
    {
      //dans ce cas, le fact est relatif à des objets qui n'existe pas
      to_delete_temp.push_back(fact.toTriplet());

    }
  }

  for (auto triplet_i : to_delete_temp)//supprimer les éléments de last_facts
  {
    response.to_delete.push_back(triplet_i);
    Fact fact_i(triplet_i);
    last_facts_[origin].erase(fact_i);
    std::cout << "suppresion du triplet car un des objets n'est plus vu :" <<triplet_i.subject<<" "<<triplet_i.predicate<<" "<<triplet_i.object<< std::endl;
  }
  
}

bool RelationsSender::shouldBeTested(Object* object)
{
  if(object->isLocated() == false)
    return false;
  //else if(object->isStatic())
    //return false;
  else if(object->isTrueId() == false)
    return false;
  else if(object->isInHand())
    return false;
  else if(agent_->pose().distanceSqTo(object->pose()) > MIN_DISTANCE)
    return false;
  else
    return true;
}

bool RelationsSender::shouldBeTestedForDeictic(Object* object_a, Object* object_b, bool first_it) //TODO: determiner les differents elements decidant si quelque chose doit etre tester ou pas
{
  if(first_it==false)
    {
    double min_dist;
    if(shouldBeTested(object_b))
      {
        min_dist= object_a->getMinDistanceTo(*object_b); //TODO: decide on a relevant minimal distance
        std::cout<<min_dist<<std::endl;
        if (object_a->hasMoved(min_dist*0.25))
        {
          std::cout<<"go into shouldBeTestedForDeictic: object a hasMoved" <<std::endl;
          return true;
        }
        else if(agent_->hasMoved())
        {
          std::cout<<"go into shouldBeTestedForDeictic: the agent hasMoved" <<std::endl;
          return true;
        }
        else
        {
          std::cout<<"go into shouldBeTestedForDeictic (false): not enough movement" <<std::endl;
          return false;
        }
      }
    else return false;
    }
  
  else return true;
}

bool RelationsSender::shouldBeTestedForIntrinsic(Object* object_a, Object* object_b) //Definir les tests préléminaires aux test intrinseques 
{
  //est ce qu'objet_a a un sens canonique ? 

  //est t'il disposé de maniere a respecter son sens canonique? Laisser une marge

return false;

}


bool RelationsSender::isNextTo(Object* object_a, Object* object_b)
{
  double max_a = *std::max_element(object_a->getBoundingBox().begin(), object_a->getBoundingBox().end());
  double max_b = *std::max_element(object_b->getBoundingBox().begin(), object_b->getBoundingBox().end());
  double max_size = std::max(max_a, max_b);
  return (object_a->getMinDistanceTo(*object_b) <= max_size * 2.0);
  // we should concider to use the distance between the surfaces of the BBs but it can be time consuming
}

bool RelationsSender::isOverlappingOnZ(Object* object_a, Object* object_b)
{
  double min_height = std::min(object_a->getAabb().max[2] - object_a->getAabb().min[2], object_b->getAabb().max[2] - object_b->getAabb().min[2]);
  double overlapping_dist = std::max(0., std::min(object_a->getAabb().max[2], object_b->getAabb().max[2]) - std::max(object_a->getAabb().min[2], object_b->getAabb().min[2]));
  return (overlapping_dist >= min_height / 3.);
}

bool RelationsSender::overlapXY(const struct aabb_t& aabb_1, const struct aabb_t& aabb_2)
{
  if((aabb_1.min[0] == aabb_1.max[0]) || (aabb_1.min[1] == aabb_1.max[1]) ||
     (aabb_2.min[0] == aabb_2.max[0]) || (aabb_2.min[1] == aabb_2.max[1]))
    return false;
  else if((aabb_1.min[0] >= aabb_2.max[0]) || (aabb_2.min[0] >= aabb_1.max[0]))
    return false;
  else if((aabb_1.min[1] >= aabb_2.max[1]) || (aabb_2.min[1] >= aabb_1.max[1]))
    return false;
  else
    return true;
}

} // namespace owds