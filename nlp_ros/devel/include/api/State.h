// Generated by gencpp from file api/State.msg
// DO NOT EDIT!


#ifndef API_MESSAGE_STATE_H
#define API_MESSAGE_STATE_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace api
{
template <class ContainerAllocator>
struct State_
{
  typedef State_<ContainerAllocator> Type;

  State_()
    : Start()
    , AnyQuestions()
    , NoiseLevel()
    , Attentiveness()
    , NoQuestionsLoop()  {
    }
  State_(const ContainerAllocator& _alloc)
    : Start(_alloc)
    , AnyQuestions(_alloc)
    , NoiseLevel(_alloc)
    , Attentiveness(_alloc)
    , NoQuestionsLoop(_alloc)  {
  (void)_alloc;
    }



   typedef std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> _Start_type;
  _Start_type Start;

   typedef std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> _AnyQuestions_type;
  _AnyQuestions_type AnyQuestions;

   typedef std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> _NoiseLevel_type;
  _NoiseLevel_type NoiseLevel;

   typedef std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> _Attentiveness_type;
  _Attentiveness_type Attentiveness;

   typedef std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> _NoQuestionsLoop_type;
  _NoQuestionsLoop_type NoQuestionsLoop;





  typedef boost::shared_ptr< ::api::State_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::api::State_<ContainerAllocator> const> ConstPtr;

}; // struct State_

typedef ::api::State_<std::allocator<void> > State;

typedef boost::shared_ptr< ::api::State > StatePtr;
typedef boost::shared_ptr< ::api::State const> StateConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::api::State_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::api::State_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::api::State_<ContainerAllocator1> & lhs, const ::api::State_<ContainerAllocator2> & rhs)
{
  return lhs.Start == rhs.Start &&
    lhs.AnyQuestions == rhs.AnyQuestions &&
    lhs.NoiseLevel == rhs.NoiseLevel &&
    lhs.Attentiveness == rhs.Attentiveness &&
    lhs.NoQuestionsLoop == rhs.NoQuestionsLoop;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::api::State_<ContainerAllocator1> & lhs, const ::api::State_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace api

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::api::State_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::api::State_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::api::State_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::api::State_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::api::State_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::api::State_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::api::State_<ContainerAllocator> >
{
  static const char* value()
  {
    return "814c22ab7e9ed8b959e5c73c87910fce";
  }

  static const char* value(const ::api::State_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x814c22ab7e9ed8b9ULL;
  static const uint64_t static_value2 = 0x59e5c73c87910fceULL;
};

template<class ContainerAllocator>
struct DataType< ::api::State_<ContainerAllocator> >
{
  static const char* value()
  {
    return "api/State";
  }

  static const char* value(const ::api::State_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::api::State_<ContainerAllocator> >
{
  static const char* value()
  {
    return "string Start\n"
"string AnyQuestions \n"
"string NoiseLevel\n"
"string Attentiveness\n"
"string NoQuestionsLoop\n"
;
  }

  static const char* value(const ::api::State_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::api::State_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.Start);
      stream.next(m.AnyQuestions);
      stream.next(m.NoiseLevel);
      stream.next(m.Attentiveness);
      stream.next(m.NoQuestionsLoop);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct State_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::api::State_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::api::State_<ContainerAllocator>& v)
  {
    s << indent << "Start: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>::stream(s, indent + "  ", v.Start);
    s << indent << "AnyQuestions: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>::stream(s, indent + "  ", v.AnyQuestions);
    s << indent << "NoiseLevel: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>::stream(s, indent + "  ", v.NoiseLevel);
    s << indent << "Attentiveness: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>::stream(s, indent + "  ", v.Attentiveness);
    s << indent << "NoQuestionsLoop: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>::stream(s, indent + "  ", v.NoQuestionsLoop);
  }
};

} // namespace message_operations
} // namespace ros

#endif // API_MESSAGE_STATE_H