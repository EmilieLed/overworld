#ifndef OWDS_FACT_H
#define OWDS_FACT_H

#include <string>
#include <functional>
#include "overworld/Triplet.h"

namespace owds {

class Fact
{
public:
  Fact(const std::string& subject,
       const std::string& predicate,
       const std::string& object) : subject_(subject),
                                    predicate_(predicate),
                                    object_(object)
  {
    hash_ = std::hash<std::string>{}(toString());
  }

  Fact(const overworld::Triplet& triplet): subject_(triplet.subject),
                                           predicate_(triplet.predicate),
                                           object_(triplet.object)
  {
    hash_ = std::hash<std::string>{}(toString());
  }

  const std::string& getSubject() const { return subject_; }
  const std::string& getPredicate() const { return predicate_; }
  const std::string& getObject() const { return object_; }

  size_t getHash() const { return hash_; }
  bool operator==(const Fact& other) const
  {
    return hash_ == other.hash_;
  }

  bool operator<(const Fact& other) const
  {
    return hash_ < other.hash_;
  }
  bool useSameEntities(const Fact& other) const
  {
    return ((subject_ == other.subject_) &&
            (object_ == other.object_));
  }

  bool useSameEntitiesCrossed(const Fact& other) const 
  {
    return ((subject_ == other.object_) &&
            (object_ == other.subject_));
  }

  std::string toString(const std::string& delim = " ") const
  {
    return subject_ + delim + predicate_ + delim + object_;
  }

  overworld::Triplet toTriplet()const 
{
  overworld::Triplet triplet;
  triplet.subject = subject_;
  triplet.predicate = predicate_;
  triplet.object = object_;
  return triplet;
  }

private:
  std::string subject_;
  std::string predicate_;
  std::string object_;
  size_t hash_;
};

} // namespace owds

namespace std {

template <>
struct hash<owds::Fact>
{
  std::size_t operator()(const owds::Fact& fact) const
  {
    return fact.getHash();
  }
};
} // namespace std 

#endif // OWDS_FACT_H