#ifndef SMBA_CONVERTER_H
#define SMBA_CONVERTER_H
#include <smba_core/smba_object_list.h>
#include <smba_core/smba_polygon_list.h>
#include <Eigen/Dense>

namespace smba_converter {

class SMBAConverter
{
    public:
	smba_core::smba_polygon_list convert_to_polygon(const smba_core::smba_object_list& a);
	smba_core::smba_object_list transform_object_list(const smba_core::smba_object_list& input, float x, float y, float theta);
    smba_core::smba_object_list transform_object_list_forward(const smba_core::smba_object_list& input);
    smba_core::smba_object_list transform_object_list_backward(const smba_core::smba_object_list& input);
    bool set_transformation_matrix(float x, float y, float rotation);

private:
	Eigen::Matrix3f TRMat;
    Eigen::Matrix3f TRMatI;
    float Rotation;
};

} // namespace smba_converter

#endif // SMBA_CONVERTER_H
