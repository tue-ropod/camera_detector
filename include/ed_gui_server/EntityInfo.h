// Generated by gencpp from file ed_gui_server/EntityInfo.msg
// DO NOT EDIT!


#ifndef ED_GUI_SERVER_MESSAGE_ENTITYINFO_H
#define ED_GUI_SERVER_MESSAGE_ENTITYINFO_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <geometry_msgs/Pose.h>
#include <ed_gui_server/Color.h>
#include <ed_gui_server/Polygon.h>

namespace ed_gui_server
{
template <class ContainerAllocator>
struct EntityInfo_
{
  typedef EntityInfo_<ContainerAllocator> Type;

  EntityInfo_()
    : id()
    , type()
    , existence_probability(0.0)
    , has_pose(false)
    , pose()
    , mesh_revision(0)
    , color()
    , polygon()  {
    }
  EntityInfo_(const ContainerAllocator& _alloc)
    : id(_alloc)
    , type(_alloc)
    , existence_probability(0.0)
    , has_pose(false)
    , pose(_alloc)
    , mesh_revision(0)
    , color(_alloc)
    , polygon(_alloc)  {
  (void)_alloc;
    }



   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _id_type;
  _id_type id;

   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _type_type;
  _type_type type;

   typedef float _existence_probability_type;
  _existence_probability_type existence_probability;

   typedef uint8_t _has_pose_type;
  _has_pose_type has_pose;

   typedef  ::geometry_msgs::Pose_<ContainerAllocator>  _pose_type;
  _pose_type pose;

   typedef uint32_t _mesh_revision_type;
  _mesh_revision_type mesh_revision;

   typedef  ::ed_gui_server::Color_<ContainerAllocator>  _color_type;
  _color_type color;

   typedef  ::ed_gui_server::Polygon_<ContainerAllocator>  _polygon_type;
  _polygon_type polygon;





  typedef boost::shared_ptr< ::ed_gui_server::EntityInfo_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::ed_gui_server::EntityInfo_<ContainerAllocator> const> ConstPtr;

}; // struct EntityInfo_

typedef ::ed_gui_server::EntityInfo_<std::allocator<void> > EntityInfo;

typedef boost::shared_ptr< ::ed_gui_server::EntityInfo > EntityInfoPtr;
typedef boost::shared_ptr< ::ed_gui_server::EntityInfo const> EntityInfoConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::ed_gui_server::EntityInfo_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::ed_gui_server::EntityInfo_<ContainerAllocator> >::stream(s, "", v);
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
struct IsFixedSize< ::ed_gui_server::EntityInfo_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::ed_gui_server::EntityInfo_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::ed_gui_server::EntityInfo_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::ed_gui_server::EntityInfo_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::ed_gui_server::EntityInfo_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::ed_gui_server::EntityInfo_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::ed_gui_server::EntityInfo_<ContainerAllocator> >
{
  static const char* value()
  {
    return "88e09645e5e6e869399f324ac0c7ad87";
  }

  static const char* value(const ::ed_gui_server::EntityInfo_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x88e09645e5e6e869ULL;
  static const uint64_t static_value2 = 0x399f324ac0c7ad87ULL;
};

template<class ContainerAllocator>
struct DataType< ::ed_gui_server::EntityInfo_<ContainerAllocator> >
{
  static const char* value()
  {
    return "ed_gui_server/EntityInfo";
  }

  static const char* value(const ::ed_gui_server::EntityInfo_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::ed_gui_server::EntityInfo_<ContainerAllocator> >
{
  static const char* value()
  {
    return "string id\n\
string type\n\
float32 existence_probability\n\
bool has_pose\n\
geometry_msgs/Pose pose\n\
\n\
# Number indicating the revision of the mesh. If this number\n\
# has changed, the mesh has changed (and should be queried\n\
# again).\n\
# Special case: mesh_revision == 0 means no mesh available.\n\
uint32 mesh_revision\n\
\n\
Color color\n\
\n\
Polygon polygon\n\
\n\
================================================================================\n\
MSG: geometry_msgs/Pose\n\
# A representation of pose in free space, composed of position and orientation. \n\
Point position\n\
Quaternion orientation\n\
\n\
================================================================================\n\
MSG: geometry_msgs/Point\n\
# This contains the position of a point in free space\n\
float64 x\n\
float64 y\n\
float64 z\n\
\n\
================================================================================\n\
MSG: geometry_msgs/Quaternion\n\
# This represents an orientation in free space in quaternion form.\n\
\n\
float64 x\n\
float64 y\n\
float64 z\n\
float64 w\n\
\n\
================================================================================\n\
MSG: ed_gui_server/Color\n\
uint8 r\n\
uint8 g\n\
uint8 b\n\
uint8 a\n\
\n\
================================================================================\n\
MSG: ed_gui_server/Polygon\n\
float32 z_min\n\
float32 z_max\n\
float32[] xs\n\
float32[] ys\n\
";
  }

  static const char* value(const ::ed_gui_server::EntityInfo_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::ed_gui_server::EntityInfo_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.id);
      stream.next(m.type);
      stream.next(m.existence_probability);
      stream.next(m.has_pose);
      stream.next(m.pose);
      stream.next(m.mesh_revision);
      stream.next(m.color);
      stream.next(m.polygon);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct EntityInfo_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::ed_gui_server::EntityInfo_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::ed_gui_server::EntityInfo_<ContainerAllocator>& v)
  {
    s << indent << "id: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.id);
    s << indent << "type: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.type);
    s << indent << "existence_probability: ";
    Printer<float>::stream(s, indent + "  ", v.existence_probability);
    s << indent << "has_pose: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.has_pose);
    s << indent << "pose: ";
    s << std::endl;
    Printer< ::geometry_msgs::Pose_<ContainerAllocator> >::stream(s, indent + "  ", v.pose);
    s << indent << "mesh_revision: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.mesh_revision);
    s << indent << "color: ";
    s << std::endl;
    Printer< ::ed_gui_server::Color_<ContainerAllocator> >::stream(s, indent + "  ", v.color);
    s << indent << "polygon: ";
    s << std::endl;
    Printer< ::ed_gui_server::Polygon_<ContainerAllocator> >::stream(s, indent + "  ", v.polygon);
  }
};

} // namespace message_operations
} // namespace ros

#endif // ED_GUI_SERVER_MESSAGE_ENTITYINFO_H