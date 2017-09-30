#include <boost/python.hpp>
#include <string>

#include <ros/serialization.h>
#include <smba_core/smba_object_list.h>
#include <smba_core/smba_polygon_list.h>

#include <smba_converter.h>

 /* Read a ROS message from a serialized string.
   */
template <typename M>
M from_python(const std::string str_msg)
{
  size_t serial_size = str_msg.size();
  boost::shared_array<uint8_t> buffer(new uint8_t[serial_size]);
  for (size_t i = 0; i < serial_size; ++i)
  {
    buffer[i] = str_msg[i];
  }
  ros::serialization::IStream stream(buffer.get(), serial_size);
  M msg;
  ros::serialization::Serializer<M>::read(stream, msg);
  return msg;
}

/* Write a ROS message into a serialized string.
*/
template <typename M>
std::string to_python(const M& msg)
{
  size_t serial_size = ros::serialization::serializationLength(msg);
  boost::shared_array<uint8_t> buffer(new uint8_t[serial_size]);
  ros::serialization::OStream stream(buffer.get(), serial_size);
  ros::serialization::serialize(stream, msg);
  std::string str_msg;
  str_msg.reserve(serial_size);
  for (size_t i = 0; i < serial_size; ++i)
  {
    str_msg.push_back(buffer[i]);
  }
  return str_msg;
}

class SMBAConverterWrapper : public smba_converter::SMBAConverter
{
  public:
    SMBAConverterWrapper() : SMBAConverter() {}
    std::string convert_to_polygon(const std::string& str_input)
    {
      smba_core::smba_object_list input = from_python<smba_core::smba_object_list>(str_input);
      smba_core::smba_polygon_list output = SMBAConverter::convert_to_polygon(input);

      return to_python(output);
    }
    std::string transform_object_list(const std::string& str_input, float x, float y, float theta)
    {
      smba_core::smba_object_list input = from_python<smba_core::smba_object_list>(str_input);
      smba_core::smba_object_list output = SMBAConverter::transform_object_list(input,x,y,theta);

      return to_python(output);
    }
    std::string transform_object_list_forward(const std::string& str_input)
    {
        smba_core::smba_object_list input = from_python<smba_core::smba_object_list>(str_input);
        smba_core::smba_object_list output = SMBAConverter::transform_object_list_forward(input);

        return to_python(output);
    }
    std::string transform_object_list_backward(const std::string& str_input)
    {
        smba_core::smba_object_list input = from_python<smba_core::smba_object_list>(str_input);
        smba_core::smba_object_list output = SMBAConverter::transform_object_list_backward(input);

        return to_python(output);
    }

};
BOOST_PYTHON_MODULE(_smba_converter_wrapper_cpp)
{
  boost::python::class_<SMBAConverterWrapper>("SMBAConverterWrapper", boost::python::init<>())
          .def("convert_to_polygon", &SMBAConverterWrapper::convert_to_polygon)
          .def("transform_object_list", &SMBAConverterWrapper::transform_object_list)
          .def("transform_object_list_forward", &SMBAConverterWrapper::transform_object_list_forward)
          .def("transform_object_list_backward", &SMBAConverterWrapper::transform_object_list_backward)
          .def("set_transformation_matrix", &SMBAConverterWrapper::set_transformation_matrix);
}
