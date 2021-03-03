// Generated by gencpp from file ed_gui_server/QueryMeshesRequest.msg
// DO NOT EDIT!


#ifndef ED_GUI_SERVER_MESSAGE_QUERYMESHESREQUEST_H
#define ED_GUI_SERVER_MESSAGE_QUERYMESHESREQUEST_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace ed_gui_server
{
template <class ContainerAllocator>
struct QueryMeshesRequest_
{
  typedef QueryMeshesRequest_<ContainerAllocator> Type;

  QueryMeshesRequest_()
    : entity_ids()  {
    }
  QueryMeshesRequest_(const ContainerAllocator& _alloc)
    : entity_ids(_alloc)  {
  (void)_alloc;
    }



   typedef std::vector<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > , typename ContainerAllocator::template rebind<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::other >  _entity_ids_type;
  _entity_ids_type entity_ids;





  typedef boost::shared_ptr< ::ed_gui_server::QueryMeshesRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::ed_gui_server::QueryMeshesRequest_<ContainerAllocator> const> ConstPtr;

}; // struct QueryMeshesRequest_

typedef ::ed_gui_server::QueryMeshesRequest_<std::allocator<void> > QueryMeshesRequest;

typedef boost::shared_ptr< ::ed_gui_server::QueryMeshesRequest > QueryMeshesRequestPtr;
typedef boost::shared_ptr< ::ed_gui_server::QueryMeshesRequest const> QueryMeshesRequestConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::ed_gui_server::QueryMeshesRequest_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::ed_gui_server::QueryMeshesRequest_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace ed_gui_server

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': False}
// {'std_msgs': ['/opt/ros/kinetic/share/std_msgs/cmake/../msg'], 'geometry_msgs': ['/opt/ros/kinetic/share/geometry_msgs/cmake/../msg'], 'ed_gui_server': ['/home/gosse/ropod-project-software/catkin_workspace/src/functionalities/ED/ed_gui_server/msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::ed_gui_server::QueryMeshesRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::ed_gui_server::QueryMeshesRequest_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::ed_gui_server::QueryMeshesRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::ed_gui_server::QueryMeshesRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::ed_gui_server::QueryMeshesRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::ed_gui_server::QueryMeshesRequest_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::ed_gui_server::QueryMeshesRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "c3342e15d8974ec05cf94758c3900440";
  }

  static const char* value(const ::ed_gui_server::QueryMeshesRequest_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xc3342e15d8974ec0ULL;
  static const uint64_t static_value2 = 0x5cf94758c3900440ULL;
};

template<class ContainerAllocator>
struct DataType< ::ed_gui_server::QueryMeshesRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "ed_gui_server/QueryMeshesRequest";
  }

  static const char* value(const ::ed_gui_server::QueryMeshesRequest_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::ed_gui_server::QueryMeshesRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "\n\
string[] entity_ids\n\
";
  }

  static const char* value(const ::ed_gui_server::QueryMeshesRequest_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::ed_gui_server::QueryMeshesRequest_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.entity_ids);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct QueryMeshesRequest_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::ed_gui_server::QueryMeshesRequest_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::ed_gui_server::QueryMeshesRequest_<ContainerAllocator>& v)
  {
    s << indent << "entity_ids[]" << std::endl;
    for (size_t i = 0; i < v.entity_ids.size(); ++i)
    {
      s << indent << "  entity_ids[" << i << "]: ";
      Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.entity_ids[i]);
    }
  }
};

} // namespace message_operations
} // namespace ros

#endif // ED_GUI_SERVER_MESSAGE_QUERYMESHESREQUEST_H