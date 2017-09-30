#include <smba_converter.h>
#include <smba_core/smba_object_list.h>
#include <smba_core/smba_polygon_list.h>
#include <smba_core/smba_enums.h>
//#include <cmath>

using namespace smba_converter;

smba_core::smba_polygon_list SMBAConverter::convert_to_polygon(const smba_core::smba_object_list& input)
{
	smba_core::smba_polygon_list output;
	// header is the same
	output.head = input.head;

	for(int i = 0; i<(input.data).size(); i++) {

		smba_core::smba_object current_in = input.data[i];
		smba_core::smba_polygon_object current_out; // = output.data[i];

		// same object id and debug object id
		current_out.object_id = current_in.obj_id;
		current_out.object_id_debug = current_in.obj_id;
		// position from rectangle map
		float x = current_in.pos.x;
		float y = current_in.pos.y;

        // in y-direction
		float width = current_in.width;
		// in x-direction
		float length =current_in.length;

        // heading direction of vehicle
        float dir = current_in.heading;

		// left bottom
		smba_core::smba_float2d l_bottom;
		l_bottom.x = x - ((length/2)*cosf(dir)) - ((width/2)*cosf(dir + 90));
		l_bottom.y = y - ((length/2)*sinf(dir)) - ((width/2)*sinf(dir + 90));
		// right bottom
		smba_core::smba_float2d r_top;
		r_top.x = x - ((length/2)*cosf(dir)) + ((width/2)*cosf(dir + 90));
		r_top.y = y - ((length/2)*sinf(dir)) + ((width/2)*sinf(dir + 90));
		// right top
		smba_core::smba_float2d r_bottom;
		r_bottom.x = x + ((length/2)*cosf(dir)) - ((width/2)*cosf(dir + 90));
		r_bottom.y = y + (((length/2)*sinf(dir)) - ((width/2)*sinf(dir + 90)));
		// left top
		smba_core::smba_float2d l_top;
		l_top.x = x + (((length/2)*cosf(dir)) + ((width/2)*cosf(dir + 90)));
		l_top.y = y + (((length/2)*sinf(dir)) + ((width/2)*sinf(dir + 90)));

        // add vertices to the points vector
		current_out.points.push_back(l_bottom);
		current_out.points.push_back(r_bottom);
		current_out.points.push_back(r_top);
		current_out.points.push_back(l_top);
		current_out.points.push_back(l_bottom); // edge for the last vertex to the first vertex

        // x-coordinate on unit circle with current heading direction
		float x_unit = cosf(current_in.heading);
        // y-coordinate on unit circle with current heading direction
		float y_unit = sinf(current_in.heading);
		smba_core::smba_float2d speed;
		// speed = magnitude of the vector
		speed.x = current_in.speed * x_unit;
		speed.y = current_in.speed * y_unit;
		current_out.speed = speed; // magnitude of vectors is speed in m/s
		current_out.object_type = current_in.object_type;
		output.data.push_back(current_out);
	}

	return output;
}


smba_core::smba_object_list SMBAConverter::transform_object_list(const smba_core::smba_object_list& input, float x, float y, float theta)
{
  	smba_core::smba_object_list output;
  	output = input;
  	float x_temp, y_temp;
  	for(std::vector<smba_core::smba_object>::iterator it = output.data.begin(); it != output.data.end(); ++it) {
    	x_temp = it->pos.x + x;
    	y_temp = it->pos.y + y;

    	it->heading = fmodf(180.f*(theta/smba_core::smba_enums::pi),360.f);
    	it->pos.x = cosf(theta)*x_temp + sinf(theta)*y_temp;
    	it->pos.y = cosf(theta)*y_temp + sinf(theta)*x_temp;
	}
    return output;
}

bool SMBAConverter::set_transformation_matrix(float x, float y, float rotation){
    SMBAConverter::TRMat  <<    cosf(rotation), -sinf(rotation),    x,
                                sinf(rotation), cosf(rotation),     y,
                                0,              0,                  1;
    TRMatI = TRMat.inverse();
    Rotation = rotation;
};

smba_core::smba_object_list SMBAConverter::transform_object_list_forward(const smba_core::smba_object_list& input){

    smba_core::smba_object_list output;
    output = input;
    Eigen::Vector3f temp;
    for(std::vector<smba_core::smba_object>::iterator it = output.data.begin(); it != output.data.end(); ++it) {
        temp << it->pos.x, it->pos.y, 1;
        temp = TRMat*temp;
        it->heading = fmodf(it->heading-Rotation,2*smba_core::smba_enums::pi);
        it->pos.x = temp(0);
        it->pos.y = temp(1);
    }
    return output;
};

smba_core::smba_object_list SMBAConverter::transform_object_list_backward(const smba_core::smba_object_list& input){
    smba_core::smba_object_list output;
    output = input;
    Eigen::Vector3f temp;
    for(std::vector<smba_core::smba_object>::iterator it = output.data.begin(); it != output.data.end(); ++it) {
        temp << it->pos.x, it->pos.y, 1;
        temp = TRMatI*temp;
        it->heading = fmodf(it->heading+Rotation,2*smba_core::smba_enums::pi);
        it->pos.x = temp(0);
        it->pos.y = temp(1);
    }
    return output;
};