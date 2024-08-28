#include "overworld/Senders/ApproachSender.h"

#include "ontologenius/OntologiesManipulator.h"

#include <cctype>
#include <iostream>

namespace owds {

bool LogicalAlgebraNode::evaluate(const Pose& pose, AreasPerceptionManager* manager)
{
  switch (operator_)
  {
  case logical_and:
    for(auto& value : values_)
      if(!evaluate(pose, value, manager))
        return false;
    for(auto& node : nodes_)
      if(!node.evaluate(pose, manager))
        return false;
    return true;
  case logical_or:
    for(auto& value : values_)
      if(evaluate(pose, value, manager))
        return true;
    for(auto& node : nodes_)
      if(node.evaluate(pose, manager))
        return true;
    return false;
  case logical_not:
    if(values_.size())
      return !evaluate(pose, values_.front(), manager);
    else if(nodes_.size())
      return !nodes_.front().evaluate(pose, manager);
    else
      return false;
  default:
    return true;
  }
}

bool LogicalAlgebraNode::evaluate(const Pose& pose, const std::string& value, AreasPerceptionManager* manager)
{
  Area* area = manager->getArea(value);
  if(area == nullptr)
    return false;

  return area->isInside(pose);
}

void LogicalAlgebraNode::print(size_t level)
{
  std::string tabs;
  for(size_t i = 0; i < level; i++)
    tabs += "  ";
  if(operator_ == logical_and)
    std::cout << tabs << "-AND" << std::endl;
  else if(operator_ == logical_or)
    std::cout << tabs << "-OR" << std::endl;
  else if(operator_ == logical_not)
    std::cout << tabs << "-NOT" << std::endl;
  for(auto& value : values_)
    std::cout << tabs << "-->" << value << std::endl;
  for(auto& node : nodes_)
    node.print(level+1);
}

ApproachSender::ApproachSender(ros::NodeHandle* n,
                               PerceptionManagers* managers) : n_(n),
                                                                                   robot_(nullptr),
                                                                                   managers_(managers),
                                                                                   onto_(nullptr)
{
  if(n_ != nullptr)
    get_pose_service_ = n_->advertiseService("/overworld/getApproachPose", &ApproachSender::onGetApproachPoint, this);
  std::cout<<"Accès au service"<<std::endl;
}

void ApproachSender::setRobotName(const std::string& robot_name)
{
  robot_name_ = robot_name;
  onto::OntologiesManipulator ontos;
  ontos.waitInit();
  ontos.add(robot_name_);
  onto_ = ontos.get(robot_name_);
  onto_->close();

  robot_ = managers_->robots_manager_.getAgent(robot_name_);
}

bool ApproachSender::onGetApproachPoint(overworld::GetApproachPoint::Request& req, overworld::GetApproachPoint::Response& res)
{
  std::cout<<"onGetApproachPoint"<<std::endl;
  //setRobotName(robot_->getId());
  //part to debug
  
  std::string target = req.entity;
  auto furnitures = onto_->individuals.getOn(req.entity, "isAbove", "Furniture");
  std::cout<<"ici"<<std::endl;
  if(furnitures.empty() == false)
    target = furnitures.front();
  
  auto furniture_it = managers_->objects_manager_.getEntities().find(target);
  if(furniture_it == managers_->objects_manager_.getEntities().end())
    return false;

  if(furniture_it->second->isLocated() == false)
    return false;

  auto target_it = managers_->objects_manager_.getEntities().find(req.entity);
  if(target_it == managers_->objects_manager_.getEntities().end())
    return false;

  if(target_it->second->isLocated() == false)
    return false;

  Pose furniture_pose = furniture_it->second->pose();
  Pose target_pose = target_it->second->pose();

  auto around_poses = getApproachPoints(furniture_it->second);
  if(req.area_constraints.empty() == false)
  {
    auto constraint = constraintToTree(req.area_constraints);
    constraint.print();

    auto area = managers_->areas_manager_.getArea("an_area");
    if(area != nullptr)
    {
      area->isInside(around_poses.front(), false);
    }

    std::cout << "TODO" << std::endl;
  }

  auto nearest_pose = getNearestPoint(target_pose, around_poses);

  res.pose.pose = nearest_pose.toPoseMsg();
  res.pose.header.frame_id = "map";
  
  return true;
}

LogicalAlgebraNode ApproachSender::constraintToTree(std::string constraint)
{
  constraint.erase(std::remove_if(constraint.begin(), constraint.end(), ::isspace),constraint.end());

  std::unordered_map<std::string, LogicalAlgebraNode> braquet_nodes;
  size_t braquet_cpt = 0;
  size_t braquet_pose = constraint.find("(");
  while(braquet_pose != std::string::npos)
  {
    std::string in_braquet;
    size_t out_pose = getIn(braquet_pose, in_braquet, constraint, '(', ')');
    constraint.replace(braquet_pose, out_pose - braquet_pose + 1, "$" + std::to_string(braquet_cpt));
    braquet_nodes.insert(std::make_pair<std::string,LogicalAlgebraNode>("$" + std::to_string(braquet_cpt),
                                                                        constraintToTree(in_braquet)));
    braquet_cpt++;
    braquet_pose = constraint.find("(");
  }

  auto or_statments = split(constraint, "|");
  if(or_statments.size() > 1)
  {
    auto or_node = LogicalAlgebraNode(logical_or);
    for(auto& statment : or_statments)
    {
      if(statment.find("&") == std::string::npos)
      {
        if(statment[0] == '!')
        {
          auto not_node = LogicalAlgebraNode(logical_not);
          fillNode(not_node, statment.substr(1), braquet_nodes);
          or_node.insert(not_node);
        }
        else
          fillNode(or_node, statment, braquet_nodes);
      }
      else
        or_node.insert(createAndNode(statment, braquet_nodes));
    }

    return or_node;
  }
  else
    return createAndNode(constraint, braquet_nodes);
}

LogicalAlgebraNode ApproachSender::createAndNode(std::string constraint, const std::unordered_map<std::string, LogicalAlgebraNode>& braquet_nodes)
{
  auto and_node = LogicalAlgebraNode(logical_and);

  auto and_statments = split(constraint, "&");
  for(auto& statment : and_statments)
  {
    if(statment[0] == '!')
    {
      auto not_node = LogicalAlgebraNode(logical_not);
      fillNode(not_node, statment.substr(1), braquet_nodes);
      and_node.insert(not_node);
    }
    else
      fillNode(and_node, statment, braquet_nodes);
  }

  return and_node;
}

void ApproachSender::fillNode(LogicalAlgebraNode& node, std::string constraint, const std::unordered_map<std::string, LogicalAlgebraNode>& braquet_nodes)
{
  if(constraint[0] == '$')
    node.insert(braquet_nodes.at(constraint));
  else
    node.insert(constraint);
}

size_t ApproachSender::getIn(size_t begin, std::string& in_text, const std::string& text, char symbol_in, char symbol_out)
{
  size_t pose = begin;

  if(text.at(pose) == symbol_in)
  {
    size_t first_pose = pose;
    int cpt = 1;
    while((cpt != 0) && (pose+1 < text.length()))
    {
      ++pose;
      if(text.at(pose) == symbol_in)
        cpt++;
      else if(text.at(pose) == symbol_out)
        cpt--;

    }

    in_text = text.substr(first_pose+1, pose-first_pose-1);

    if(cpt == 0)
      return pose;
    else
      return std::string::npos;
  }
  else
    return begin;
}

std::vector<std::string> ApproachSender::split(const std::string& text, const std::string& delim)
{
  std::vector<std::string> res;
  std::string tmp_text = text;
  while(tmp_text.find(delim) != std::string::npos)
  {
    size_t pos = tmp_text.find(delim);
    std::string part = tmp_text.substr(0, pos);
    tmp_text = tmp_text.substr(pos + delim.size(), tmp_text.size() - pos - delim.size());
    if(part != "")
      res.emplace_back(part);
  }
  res.emplace_back(tmp_text);
  return res;
}

Pose ApproachSender::findBestApproachPoint(Object* target_object, Area valid_area)
{
  auto support=findSupport(target_object);
  auto approach_points=getApproachPoints(support);
  auto valid_poses=applyConstraint(valid_area,approach_points);
  auto bestApproachPoint=getNearestPoint(target_object->pose(),valid_poses);
  return bestApproachPoint;
  
}

std::vector<Pose> ApproachSender::getApproachPoints(Object* object)
{
  auto corners = object->getCorners();
  //We calculate the middle point of the lower face of the bounding box
  
  auto rectd = drawRectangleAtDistance(corners[0],corners[1],corners[2],corners[3],1.0);

  //Calcultate the approach point
  auto face1=calculateMidPointBetweenTwoPoints(rectd[0],rectd[1]);
  auto face2=calculateMidPointBetweenTwoPoints(rectd[0],rectd[2]);
  auto face3=calculateMidPointBetweenTwoPoints(rectd[2],rectd[3]);
  auto face4=calculateMidPointBetweenTwoPoints(rectd[1],rectd[3]);

  std::vector<Pose> approach_points_={face1,face2,face3,face4};

  return approach_points_;
}

std::vector<Pose> ApproachSender::drawRectangleAtDistance(Pose p1, Pose p2,Pose p3,Pose p4, double d)
{
  Pose newP1={{(p1.getX() - d, p1.getY() - d, p1.getZ())},{0,0,0,1.0}};
  Pose newP2={{(p2.getX() + d, p2.getY()- d, p2.getZ())},{0,0,0,1.0}};
  Pose newP3={{(p3.getX() - d, p3.getY() + d, p3.getZ())},{0,0,0,1.0}};
  Pose newP4={{(p4.getX() + d, p4.getY()+ d, p4.getZ())},{0,0,0,1.0}};
  std::vector<Pose> newRectangle={newP1,newP2,newP3,newP4};
  
  return newRectangle;
}

Pose ApproachSender::calculateMidPointBetweenTwoPoints(Pose p1, Pose p2)
{
   std::array<double,3> translation ={(p1.getX() + p2.getX()) / 2,
                   (p1.getY() + p2.getY())/2,
                   (p1.getZ() + p2.getZ())/2}; //Pourrait être remplacé par 0 
  std::array<double,4> rotation ={0,0,0,1.0}; 
  return Pose(translation,rotation);

}

std::vector<Pose> ApproachSender::applyConstraint(Area valid_area, const std::vector<Pose>& approach_points)
{
  std::vector<Pose> valid_poses;

  for (auto point : approach_points)
  {
    if (valid_area.isInside(point))
    {
      valid_poses.push_back(point);
    }
  }
  return valid_poses;

}

Pose ApproachSender::getNearestPoint(const Pose& pose_object, const std::vector<Pose>& valid_poses)
{
  double mindist=pose_object.distanceTo(valid_poses.front());
  Pose res;
  for(auto& pose : valid_poses)
  {
    if (pose_object.distanceTo(pose)<=mindist )
    mindist=pose_object.distanceTo(pose);
    res=pose;
  }
return res;

}

bool ApproachSender::isOverlappingOnZ(Object* object_a, Object* object_b)
{
  double min_height = std::min(object_a->getAabb().max[2] - object_a->getAabb().min[2], object_b->getAabb().max[2] - object_b->getAabb().min[2]);
  double overlapping_dist = std::max(0., std::min(object_a->getAabb().max[2], object_b->getAabb().max[2]) - std::max(object_a->getAabb().min[2], object_b->getAabb().min[2]));
  return (overlapping_dist >= min_height / 3.);
}

bool ApproachSender::overlapXY(Object* object_a, Object* object_b)
{
  auto aabb_1=object_a->getAabb();
  auto aabb_2=object_b->getAabb();
  
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

Object* ApproachSender::findSupport (Object* target_object)
{
  //We must find the object on which the target object is on
  auto objects = managers_->objects_manager_.getEntities(); 
  std::vector<Object*> valid_support;
  for(auto object = objects.begin(); object != objects.end(); ++object) //we go through all the objects and check if they count as a support
  {
    if(target_object != object->second) 
    {
      //we check if the BB of the target object overlap with the BB of the suspected support object 
      if(overlapXY(target_object,object->second))
        if (isOverlappingOnZ(target_object,object->second)) // on ajoute à une liste l'objet qui valide ces deux conditions
          valid_support.push_back(object->second);
    }
  }
  //After that, we check which support is the best suited in the list of valid support (Biggest one ?)

  auto max=0.0;
  Object* support;
    for(auto& object : valid_support)
  {
    if (getLowerFaceSurface(object)>max)
    {
      max=getLowerFaceSurface(object);
      support=object;
    }
  }
  return support;
}

double ApproachSender::getLowerFaceSurface(Object* object)
{
  auto corners = object->getCorners();

  return (corners[0].distanceTo(corners[1]))*(corners[0].distanceTo(corners[3]));
}


} // namespace owds