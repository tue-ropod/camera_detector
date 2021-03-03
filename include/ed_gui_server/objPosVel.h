// Generated by gencpp from file ed_gui_server/objPosVel.msg
// DO NOT EDIT!


#ifndef ED_GUI_SERVER_MESSAGE_OBJPOSVEL_H
#define ED_GUI_SERVER_MESSAGE_OBJPOSVEL_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <ed_gui_server/rectangleProperties.h>
#include <ed_gui_server/circleProperties.h>

namespace ed_gui_server
{
template <class ContainerAllocator>
struct objPosVel_
{
  typedef objPosVel_<ContainerAllocator> Type;

  objPosVel_()
    : ID()
    , rectangle()
    , circle()  {
    }
  objPosVel_(const ContainerAllocator& _alloc)
    : ID(_alloc)
    , rectangle(_alloc)
    , circle(_alloc)  {
  (void)_alloc;
    }



   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _ID_type;
  _ID_type ID;

   typedef  ::ed_gui_server::rectangleProperties_<ContainerAllocator>  _rectangle_type;
  _rectangle_type rectangle;

   typedef  ::ed_gui_server::circleProperties_<ContainerAllocator>  _circle_type;
  _circle_type circle;





  typedef boost::shared_ptr< ::ed_gui_server::objPosVel_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::ed_gui_server::objPosVel_<ContainerAllocator> const> ConstPtr;

}; // struct objPosVel_

typedef ::ed_gui_server::objPosVel_<std::allocator<void> > objPosVel;

typedef boost::shared_ptr< ::ed_gui_server::objPosVel > objPosVelPtr;
typedef boost::shared_ptr< ::ed_gui_server::objPosVel const> objPosVelConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::ed_gui_server::objPosVel_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::ed_gui_server::objPosVel_<ContainerAllocator> >::stream(s, "", v);
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
struct IsFixedSize< ::ed_gui_server::objPosVel_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::ed_gui_server::objPosVel_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::ed_gui_server::objPosVel_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::ed_gui_server::objPosVel_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::ed_gui_server::objPosVel_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::ed_gui_server::objPosVel_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::ed_gui_server::objPosVel_<ContainerAllocator> >
{
  static const char* value()
  {
    return "e87e0ef6cdfe4c5fe0a4aeaaeb0a3f72";
  }

  static const char* value(const ::ed_gui_server::objPosVel_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xe87e0ef6cdfe4c5fULL;
  static const uint64_t static_value2 = 0xe0a4aeaaeb0a3f72ULL;
};

template<class ContainerAllocator>
struct DataType< ::ed_gui_server::objPosVel_<ContainerAllocator> >
{
  static const char* value()
  {
    return "ed_gui_server/objPosVel";
  }

  static const char* value(const ::ed_gui_server::objPosVel_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::ed_gui_server::objPosVel_<ContainerAllocator> >
{
  static const char* value()
  {
    return "string ID\n\
rectangleProperties rectangle\n\
circleProperties circle\n\
\n\
================================================================================\n\
MSG: ed_gui_server/rectangleProperties\n\
float32 probability\n\
\n\
geometry_msgs/Pose pose\n\
float32 xPosStdDev\n\
float32 yPosStdDev\n\
float32 yawStdDev\n\
\n\
float32 width\n\
float32 depth\n\
float32 yaw\n\
\n\
float32 widthStdDev\n\
float32 depthStdDev\n\
\n\
geometry_msgs/Vector3 vel\n\
float32 yawVel\n\
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
MSG: geometry_msgs/Vector3\n\
# This represents a vector in free space. \n\
# It is only meant to represent a direction. Therefore, it does not\n\
# make sense to apply a translation to it (e.g., when applying a \n\
# generic rigid transformation to a Vector3, tf2 will only apply the\n\
# rotation). If you want your data to be translatable too, use the\n\
# geometry_msgs/Point message instead.\n\
\n\
float64 x\n\
float64 y\n\
float64 z\n\
================================================================================\n\
MSG: ed_gui_server/circleProperties\n\
float32 probability\n\
\n\
geometry_msgs/Pose pose\n\
float32 xPosStdDev\n\
float32 yPosStdDev\n\
\n\
float32 radius\n\
float32 radiusStdDev\n\
\n\
geometry_msgs/Vector3 vel\n\
";
  }

  static const char* value(const ::ed_gui_server::objPosVel_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::ed_gui_server::objPosVel_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.ID);
      stream.next(m.rectangle);
      stream.next(m.circle);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct objPosVel_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::ed_gui_server::objPosVel_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::ed_gui_server::objPosVel_<ContainerAllocator>& v)
  {
    s << indent << "ID: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.ID);
    s << indent << "rectangle: ";
    s << std::endl;
    Printer< ::ed_gui_server::rectangleProperties_<ContainerAllocator> >::stream(s, indent + "  ", v.rectangle);
    s << indent << "circle: ";
    s << std::endl;
    Printer< ::ed_gui_server::circleProperties_<ContainerAllocator> >::stream(s, indent + "  ", v.circle);
  }
};

} // namespace message_operations
} // namespace ros

#endif // ED_GUI_SERVER_MESSAGE_OBJPOSVEL_H
