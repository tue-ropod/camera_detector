// Generated by gencpp from file ed_gui_server/Mesh.msg
// DO NOT EDIT!


#ifndef ED_GUI_SERVER_MESSAGE_MESH_H
#define ED_GUI_SERVER_MESSAGE_MESH_H


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
struct Mesh_
{
  typedef Mesh_<ContainerAllocator> Type;

  Mesh_()
    : revision(0)
    , vertices()
    , triangles()  {
    }
  Mesh_(const ContainerAllocator& _alloc)
    : revision(0)
    , vertices(_alloc)
    , triangles(_alloc)  {
  (void)_alloc;
    }



   typedef uint32_t _revision_type;
  _revision_type revision;

   typedef std::vector<float, typename ContainerAllocator::template rebind<float>::other >  _vertices_type;
  _vertices_type vertices;

   typedef std::vector<uint32_t, typename ContainerAllocator::template rebind<uint32_t>::other >  _triangles_type;
  _triangles_type triangles;





  typedef boost::shared_ptr< ::ed_gui_server::Mesh_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::ed_gui_server::Mesh_<ContainerAllocator> const> ConstPtr;

}; // struct Mesh_

typedef ::ed_gui_server::Mesh_<std::allocator<void> > Mesh;

typedef boost::shared_ptr< ::ed_gui_server::Mesh > MeshPtr;
typedef boost::shared_ptr< ::ed_gui_server::Mesh const> MeshConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::ed_gui_server::Mesh_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::ed_gui_server::Mesh_<ContainerAllocator> >::stream(s, "", v);
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
struct IsFixedSize< ::ed_gui_server::Mesh_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::ed_gui_server::Mesh_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::ed_gui_server::Mesh_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::ed_gui_server::Mesh_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::ed_gui_server::Mesh_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::ed_gui_server::Mesh_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::ed_gui_server::Mesh_<ContainerAllocator> >
{
  static const char* value()
  {
    return "47b3b8e1033fe3cd3e2f1aad5f228523";
  }

  static const char* value(const ::ed_gui_server::Mesh_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x47b3b8e1033fe3cdULL;
  static const uint64_t static_value2 = 0x3e2f1aad5f228523ULL;
};

template<class ContainerAllocator>
struct DataType< ::ed_gui_server::Mesh_<ContainerAllocator> >
{
  static const char* value()
  {
    return "ed_gui_server/Mesh";
  }

  static const char* value(const ::ed_gui_server::Mesh_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::ed_gui_server::Mesh_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# Mesh identifier\n\
uint32 revision\n\
\n\
# At the moment, only vertices are used. The triangles are described by 9 elements in the\n\
# vertices array (x1, y1, z1, x2, y2, z2, x3, y3, z3). The triangles array is NOT used.\n\
\n\
# (original method (NOT USED): List of vertices: each 3 elements describe (x, y, z) )\n\
float32[] vertices\n\
\n\
# (origin method (NOT USED) List of indices: each 3 indices describe one triangle, indexing the vertices given above)\n\
uint32[] triangles\n\
";
  }

  static const char* value(const ::ed_gui_server::Mesh_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::ed_gui_server::Mesh_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.revision);
      stream.next(m.vertices);
      stream.next(m.triangles);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct Mesh_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::ed_gui_server::Mesh_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::ed_gui_server::Mesh_<ContainerAllocator>& v)
  {
    s << indent << "revision: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.revision);
    s << indent << "vertices[]" << std::endl;
    for (size_t i = 0; i < v.vertices.size(); ++i)
    {
      s << indent << "  vertices[" << i << "]: ";
      Printer<float>::stream(s, indent + "  ", v.vertices[i]);
    }
    s << indent << "triangles[]" << std::endl;
    for (size_t i = 0; i < v.triangles.size(); ++i)
    {
      s << indent << "  triangles[" << i << "]: ";
      Printer<uint32_t>::stream(s, indent + "  ", v.triangles[i]);
    }
  }
};

} // namespace message_operations
} // namespace ros

#endif // ED_GUI_SERVER_MESSAGE_MESH_H
