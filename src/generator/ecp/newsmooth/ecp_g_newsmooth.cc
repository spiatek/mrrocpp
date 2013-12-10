/**
 * @file
 * @brief Contains definitions of the methods of newsmooth class.
 * @author rtulwin
 * @ingroup generators
 */

#include <fstream>

#include "base/ecp/ecp_exceptions.h"
#include "generator/ecp/newsmooth/ecp_g_newsmooth.h"

namespace mrrocpp {
namespace ecp {
namespace common {
namespace generator {

using namespace std;

newsmooth::newsmooth(common::task::task& _ecp_task, lib::ECP_POSE_SPECIFICATION pose_spec, int axes_num) :
		multiple_position <ecp_mp::common::trajectory_pose::bang_bang_trajectory_pose,
				ecp::common::generator::trajectory_interpolator::bang_bang_interpolator,
				ecp::common::generator::velocity_profile_calculator::bang_bang_profile>(_ecp_task)
{
	this->pose_spec = pose_spec;
	this->axes_num = axes_num;
	this->vpc = velocity_profile_calculator::bang_bang_profile();
	this->inter = trajectory_interpolator::bang_bang_interpolator();

	create_velocity_vectors(axes_num);
}

newsmooth::~newsmooth()
{

}

bool newsmooth::calculate()
{
	//printf("\n################################## Calculate #################################\n");
	sr_ecp_msg.message("Calculating...");
	int i, j; //loop counters

	pose_vector_iterator = pose_vector.begin();

	for (i = 0; i < pose_vector.size(); i++) {
		vpc.clean_up_pose(pose_vector_iterator);
		pose_vector_iterator++;
	}

        //printf("\n------------ first print pose --------------\n");
        //print_pose_vector();

	pose_vector_iterator = pose_vector.begin();

	for (i = 0; i < pose_vector.size(); i++) { //this has to be done here (not in the load_trajectory_pose method) because of the potential recursive call of calculate method
		if (!vpc.calculate_v_r_a_r_pose(pose_vector_iterator)) {
			if (debug) {
				printf("calculate_v_r_a_r_pose returned false\n");
			}
			return false;
		}
		pose_vector_iterator++;
	}

//        printf("\n------------ second print pose --------------\n");
//        print_pose_vector();

	pose_vector_iterator = pose_vector.begin();
	if (pose_spec == lib::ECP_XYZ_ANGLE_AXIS && motion_type == lib::ABSOLUTE) {

		set_relative();
		angle_axis_absolute_transformed_into_relative = true;

		for (i = 0; i < pose_vector.size(); i++) {
			if (!vpc.calculate_relative_angle_axis_vector(pose_vector_iterator)) {
				if (debug) {
					printf("calculate_relative_angle_axis_vector returned false\n");
				}
				return false;
			}
			pose_vector_iterator++;
		}
	}

	pose_vector_iterator = pose_vector.begin();

	for (i = 0; i < pose_vector.size(); i++) { //calculate distances, directions

		if (motion_type == lib::ABSOLUTE) { //absolute type of motion
			if (!vpc.calculate_absolute_distance_direction_pose(pose_vector_iterator)) {
				if (debug) {
					printf("calculate_absolute_distance_direction_pose returned false\n");

				}
				return false;
			}
		} else if (motion_type == lib::RELATIVE) { //relative type of motion
			if (!vpc.calculate_relative_distance_direction_pose(pose_vector_iterator)) {
				if (debug) {
					printf("calculate_relative_distance_direction_pose returned false\n");
				}
				return false;
			}
		} else {
			sr_ecp_msg.message("Wrong motion type");
			BOOST_THROW_EXCEPTION(exception::nfe_g() << lib::exception::mrrocpp_error0(ECP_ERRORS));
			//TODO change the second argument
		}

		pose_vector_iterator++;
	}

	pose_vector_iterator = pose_vector.begin();
	vector <ecp_mp::common::trajectory_pose::bang_bang_trajectory_pose>::iterator tempIter = pose_vector.end();
	vector <ecp_mp::common::trajectory_pose::bang_bang_trajectory_pose>::iterator tempIter2 = pose_vector.begin();


	for (i = 0; i < pose_vector.size(); i++) { //for each pose

                //printf("\n------------ first print pose %d --------------\n", pose_vector_iterator->pos_num);
                //print_pose(pose_vector_iterator);

		if (!vpc.set_v_k_pose(pose_vector_iterator, tempIter) || //set up v_k for the pose
				!vpc.set_v_p_pose(pose_vector_iterator, tempIter2) || //set up v_p for the pose
				!vpc.set_model_pose(pose_vector_iterator) || //choose motion model for the pose
				!vpc.calculate_s_acc_s_dec_pose(pose_vector_iterator)) { //calculate s_acc and s_dec for the pose
			return false;
		}

		//printf("\n------------ second print pose %d --------------\n", pose_vector_iterator->pos_num);
		//print_pose(pose_vector_iterator);
		for (j = 0; j < axes_num; j++) { //for each axis
			if (vpc.check_if_no_movement(pose_vector_iterator, j)) {
				continue;
			}
			if (vpc.check_s_acc_s_decc(pose_vector_iterator, j)) { //check if s_acc && s_dec < s
				vpc.calculate_s_uni(pose_vector_iterator, j); //calculate s_uni
				vpc.calculate_time(pose_vector_iterator, j); //calculate and set time
			} else { //if not

				if (!vpc.optimize_time_axis(pose_vector_iterator, j)) {
					return calculate();
				}

				//printf("\n------------ after optimize time pose %d axis: %d --------------\n", pose_vector_iterator->pos_num, j);
				//print_pose(pose_vector_iterator);

				if (!vpc.reduction_axis(pose_vector_iterator, j)) {
					return calculate();
				}

				//printf("\n------------ after reduction axis pose %d axis: %d --------------\n", pose_vector_iterator->pos_num, j);
				//print_pose(pose_vector_iterator);
			}
		}

		if (!vpc.set_model_pose(pose_vector_iterator)) {
			return false;
		}

		if (!vpc.calculate_pose_time(pose_vector_iterator, mc) || //calculate pose time
				!vpc.set_times_to_t(pose_vector_iterator)) { //set times to t
			return false;
		}

		//printf("\n------------ after calculate_pose_time pose %d --------------\n", pose_vector_iterator->pos_num);
		//print_pose(pose_vector_iterator);

		pose_vector_iterator->interpolation_node_no = ceil(pose_vector_iterator->t / mc); //calculate the number of the macrosteps for the pose

		for (j = 0; j < axes_num; j++) { //for each axis call reduction methods
			if (!vpc.reduction_axis(pose_vector_iterator, j)) {
				return calculate();
			}
		}

		//printf("\n------------ after second reduction_axis pose %d --------------\n", pose_vector_iterator->pos_num);
		//print_pose(pose_vector_iterator);

		if (!vpc.calculate_acc_uni_pose(pose_vector_iterator, mc)) { //set uni and acc
			return false;
		}

		//printf("\n------------ third print pose %d --------------\n", pose_vector_iterator->pos_num);
		//print_pose(pose_vector_iterator);

		pose_vector_iterator++;
	}

	return true;
}

void newsmooth::print_pose_vector()
{
	printf("\n------------------ Pose List ------------------\n");
	pose_vector_iterator = pose_vector.begin();
	for (int k = 0; k < pose_vector.size(); k++) {
		print_pose(pose_vector_iterator);
		pose_vector_iterator++;
	}
}

void newsmooth::print_pose(vector <ecp_mp::common::trajectory_pose::bang_bang_trajectory_pose>::iterator & it)
{

	if (it == pose_vector.end() || pose_vector.empty()) {
		return;
	}

	int z;
	printf("coords:\t");
	for (z = 0; z < pose_vector_iterator->coordinates.size(); z++) {
		printf("%f\t", pose_vector_iterator->coordinates[z]);
	}
	printf("\n");
	printf("start:\t");
	for (z = 0; z < pose_vector_iterator->start_position.size(); z++) {
		printf("%f\t", pose_vector_iterator->start_position[z]);
	}
	printf("\n");
	printf("s:\t");
	for (z = 0; z < pose_vector_iterator->s.size(); z++) {
		printf("%f\t", pose_vector_iterator->s[z]);
	}
	printf("\n");
	printf("k:\t");
	for (z = 0; z < pose_vector_iterator->k.size(); z++) {
		printf("%f\t", pose_vector_iterator->k[z]);
	}
	printf("\n");
	printf("times:\t");
	for (z = 0; z < pose_vector_iterator->s.size(); z++) {
		printf("%f\t", pose_vector_iterator->times[z]);
	}
	printf("\n");
	printf("v_r:\t");
	for (z = 0; z < pose_vector_iterator->v_r.size(); z++) {
		printf("%f\t", pose_vector_iterator->v_r[z]);
	}
	printf("\n");
        printf("v:\t");
        for (z = 0; z < pose_vector_iterator->v.size(); z++) {
                printf("%f\t", pose_vector_iterator->v[z]);
        }
        printf("\n");
        printf("a_r:\t");
        for (z = 0; z < pose_vector_iterator->a_r.size(); z++) {
                printf("%f\t", pose_vector_iterator->a_r[z]);
        }
        printf("\n");
        printf("a:\t");
        for (z = 0; z < pose_vector_iterator->a.size(); z++) {
                printf("%f\t", pose_vector_iterator->a[z]);
        }
        printf("\n");
	printf("v_p:\t");
	for (z = 0; z < pose_vector_iterator->v_p.size(); z++) {
		printf("%f\t", pose_vector_iterator->v_p[z]);
	}
	printf("\n");
	printf("v_k:\t");
	for (z = 0; z < pose_vector_iterator->v_k.size(); z++) {
		printf("%f\t", pose_vector_iterator->v_k[z]);
	}
	printf("\n");
	printf("s_acc:\t");
	for (z = 0; z < pose_vector_iterator->s_acc.size(); z++) {
		printf("%f\t", pose_vector_iterator->s_acc[z]);
	}
	printf("\n");
	printf("s_uni:\t");
	for (z = 0; z < pose_vector_iterator->s_uni.size(); z++) {
		printf("%f\t", pose_vector_iterator->s_uni[z]);
	}
	printf("\n");
	printf("s_dec:\t");
	for (z = 0; z < pose_vector_iterator->s_dec.size(); z++) {
		printf("%f\t", pose_vector_iterator->s_dec[z]);
	}
	printf("\n");
	printf("model:\t");
	for (z = 0; z < pose_vector_iterator->model.size(); z++) {
		printf("%d\t\t", pose_vector_iterator->model[z]);
	}
	printf("\n");
	printf("acc:\t");
	for (z = 0; z < pose_vector_iterator->acc.size(); z++) {
		printf("%f\t", pose_vector_iterator->acc[z]);
	}
	printf("\n");
	printf("uni:\t");
	for (z = 0; z < pose_vector_iterator->uni.size(); z++) {
		printf("%f\t", pose_vector_iterator->uni[z]);
	}
	printf("\n");
	printf("t: %f\t pos_num: %d\t number of macrosteps: %d\n", pose_vector_iterator->t, pose_vector_iterator->pos_num, pose_vector_iterator->interpolation_node_no);
	printf("--------------------------------\n\n");
	flushall();
}

void newsmooth::create_velocity_vectors(int axes_num)
{
	joint_velocity = vector <double>(axes_num, 0.15);
	joint_max_velocity = vector <double>(axes_num, 1.5);
	joint_acceleration = vector <double>(axes_num, 0.02);
	joint_max_acceleration = vector <double>(axes_num, 7.0);
	motor_velocity = vector <double>(axes_num, 0.15);
	motor_max_velocity = vector <double>(axes_num, 200.0);
	motor_acceleration = vector <double>(axes_num, 0.02);
	motor_max_acceleration = vector <double>(axes_num, 150.0);
	euler_zyz_velocity = vector <double>(axes_num, 0.15); //TODO check if this is a reasonable value
	euler_zyz_max_velocity = vector <double>(axes_num, 5.0);
	euler_zyz_acceleration = vector <double>(axes_num, 0.02); //TODO check if this is a reasonable value
	euler_zyz_max_acceleration = vector <double>(axes_num, 5.0);
	angle_axis_velocity = vector <double>(axes_num, 0.15); //TODO check if this is a reasonable value
	angle_axis_max_velocity = vector <double>(axes_num, 5.0);
	angle_axis_acceleration = vector <double>(axes_num, 0.02); //TODO check if this is a reasonable value
	angle_axis_max_acceleration = vector <double>(axes_num, 5.0);
}

//--------------- METHODS USED TO LOAD POSES ----------------

bool newsmooth::load_absolute_joint_trajectory_pose(const vector <double> & coordinates)
{

	ecp_mp::common::trajectory_pose::bang_bang_trajectory_pose pose;

	return load_trajectory_pose(coordinates, lib::ABSOLUTE, lib::ECP_JOINT, joint_velocity, joint_acceleration, joint_max_velocity, joint_max_acceleration);
}

bool newsmooth::load_relative_joint_trajectory_pose(const vector <double> & coordinates)
{

	ecp_mp::common::trajectory_pose::bang_bang_trajectory_pose pose;

	return load_trajectory_pose(coordinates, lib::RELATIVE, lib::ECP_JOINT, joint_velocity, joint_acceleration, joint_max_velocity, joint_max_acceleration);
}

bool newsmooth::load_absolute_motor_trajectory_pose(const vector <double> & coordinates)
{

	ecp_mp::common::trajectory_pose::bang_bang_trajectory_pose pose;

	return load_trajectory_pose(coordinates, lib::ABSOLUTE, lib::ECP_MOTOR, motor_velocity, motor_acceleration, motor_max_velocity, motor_max_acceleration);
}

bool newsmooth::load_relative_motor_trajectory_pose(const vector <double> & coordinates)
{

	ecp_mp::common::trajectory_pose::bang_bang_trajectory_pose pose;

	return load_trajectory_pose(coordinates, lib::RELATIVE, lib::ECP_MOTOR, motor_velocity, motor_acceleration, motor_max_velocity, motor_max_acceleration);
}

bool newsmooth::load_absolute_euler_zyz_trajectory_pose(const vector <double> & coordinates)
{

	ecp_mp::common::trajectory_pose::bang_bang_trajectory_pose pose;

	return load_trajectory_pose(coordinates, lib::ABSOLUTE, lib::ECP_XYZ_EULER_ZYZ, euler_zyz_velocity, euler_zyz_acceleration, euler_zyz_max_velocity, euler_zyz_max_acceleration);
}

bool newsmooth::load_relative_euler_zyz_trajectory_pose(const vector <double> & coordinates)
{

	ecp_mp::common::trajectory_pose::bang_bang_trajectory_pose pose;

	return load_trajectory_pose(coordinates, lib::RELATIVE, lib::ECP_XYZ_EULER_ZYZ, euler_zyz_velocity, euler_zyz_acceleration, euler_zyz_max_velocity, euler_zyz_max_acceleration);
}

bool newsmooth::load_absolute_angle_axis_trajectory_pose(const vector <double> & coordinates)
{

	ecp_mp::common::trajectory_pose::bang_bang_trajectory_pose pose;

	return load_trajectory_pose(coordinates, lib::ABSOLUTE, lib::ECP_XYZ_ANGLE_AXIS, angle_axis_velocity, angle_axis_acceleration, angle_axis_max_velocity, angle_axis_max_acceleration);
}

bool newsmooth::load_relative_angle_axis_trajectory_pose(const vector <double> & coordinates)
{

	ecp_mp::common::trajectory_pose::bang_bang_trajectory_pose pose;

	return load_trajectory_pose(coordinates, lib::RELATIVE, lib::ECP_XYZ_ANGLE_AXIS, angle_axis_velocity, angle_axis_acceleration, angle_axis_max_velocity, angle_axis_max_acceleration);
}

bool newsmooth::load_absolute_pose(ecp_mp::common::trajectory_pose::bang_bang_trajectory_pose & trajectory_pose)
{
	if (trajectory_pose.arm_type == lib::ECP_JOINT) {
		//std::cout<<"JOINT"<<std::endl;
		load_trajectory_pose(trajectory_pose.coordinates, lib::ABSOLUTE, trajectory_pose.arm_type, trajectory_pose.v, trajectory_pose.a, joint_max_velocity, joint_max_acceleration);
	} else if (trajectory_pose.arm_type == lib::ECP_MOTOR) {
		//std::cout<<"MOTOR"<<std::endl;
		load_trajectory_pose(trajectory_pose.coordinates, lib::ABSOLUTE, trajectory_pose.arm_type, trajectory_pose.v, trajectory_pose.a, motor_max_velocity, motor_max_acceleration);
	} else if (trajectory_pose.arm_type == lib::ECP_XYZ_ANGLE_AXIS) {
		//std::cout<<"ANGLE_AXIS"<<std::endl;
		load_trajectory_pose(trajectory_pose.coordinates, lib::ABSOLUTE, trajectory_pose.arm_type, trajectory_pose.v, trajectory_pose.a, angle_axis_max_velocity, angle_axis_max_acceleration);
	} else if (trajectory_pose.arm_type == lib::ECP_XYZ_EULER_ZYZ) {
		//std::cout<<"EULER7"<<std::endl;
		load_trajectory_pose(trajectory_pose.coordinates, lib::ABSOLUTE, trajectory_pose.arm_type, trajectory_pose.v, trajectory_pose.a, euler_zyz_max_velocity, euler_zyz_max_acceleration);
	} else {
		BOOST_THROW_EXCEPTION(exception::nfe_g() << lib::exception::mrrocpp_error0(INVALID_POSE_SPECIFICATION));
	}
	return true;
}

bool newsmooth::load_relative_pose(ecp_mp::common::trajectory_pose::bang_bang_trajectory_pose & trajectory_pose)
{
	if (trajectory_pose.arm_type == lib::ECP_JOINT) {
		load_trajectory_pose(trajectory_pose.coordinates, lib::RELATIVE, trajectory_pose.arm_type, trajectory_pose.v, trajectory_pose.a, joint_max_velocity, joint_max_acceleration);
	} else if (trajectory_pose.arm_type == lib::ECP_MOTOR) {
		load_trajectory_pose(trajectory_pose.coordinates, lib::RELATIVE, trajectory_pose.arm_type, trajectory_pose.v, trajectory_pose.a, motor_max_velocity, motor_max_acceleration);
	} else if (trajectory_pose.arm_type == lib::ECP_XYZ_ANGLE_AXIS) {
		load_trajectory_pose(trajectory_pose.coordinates, lib::RELATIVE, trajectory_pose.arm_type, trajectory_pose.v, trajectory_pose.a, angle_axis_max_velocity, angle_axis_max_acceleration);
	} else if (trajectory_pose.arm_type == lib::ECP_XYZ_EULER_ZYZ) {
		load_trajectory_pose(trajectory_pose.coordinates, lib::RELATIVE, trajectory_pose.arm_type, trajectory_pose.v, trajectory_pose.a, euler_zyz_max_velocity, euler_zyz_max_acceleration);
	} else {
		BOOST_THROW_EXCEPTION(exception::nfe_g() << lib::exception::mrrocpp_error0(INVALID_POSE_SPECIFICATION));
	}
	return true;
}

bool newsmooth::load_trajectory_pose(const vector <double> & coordinates, lib::MOTION_TYPE motion_type, lib::ECP_POSE_SPECIFICATION pose_spec, const vector <
		double> & v, const vector <double> & a, const vector <double> & v_max, const vector <double> & a_max)
{

	if (!pose_vector.empty() && this->pose_spec != pose_spec) { //check if previous positions were provided in the same representation

		sr_ecp_msg.message("Representation different than the previous one");
		return false;
	}

	if (!pose_vector.empty() && this->motion_type != motion_type) {

		sr_ecp_msg.message("Wrong motion type");
		return false;
	}

	this->motion_type = motion_type;
	this->pose_spec = pose_spec;

	ecp_mp::common::trajectory_pose::bang_bang_trajectory_pose pose; //new trajectory pose
	pose = ecp_mp::common::trajectory_pose::bang_bang_trajectory_pose(pose_spec, coordinates, v, a); //create new trajectory pose
	pose.v_max = v_max; //set the v_max vector
	pose.a_max = a_max; //set the a_max vector

	if (pose_vector.empty()) {
		pose.pos_num = 1;
	} else {
		pose.pos_num = pose_vector.back().pos_num + 1;
	}

	if (motion_type == lib::ABSOLUTE) {
		if (!pose_vector.empty()) { //set the start position of the added pose as the desired position of the previous pose
			pose.start_position = pose_vector.back().coordinates;
		}
	}

	pose_vector.push_back(pose); //put new trajectory pose into a pose vector

        if (optimization == true)
        {
            optimal_pose_vector.push_back(pose);
        }

	sr_ecp_msg.message("Pose loaded");

	return true;
}
//TODO change exceptions into "return false"s
bool newsmooth::load_trajectory_from_file(const char* file_name)
{

	sr_ecp_msg.message(file_name);

        last_loaded_file_path = file_name;

	char coordinate_type_desc[80]; //description of pose specification read from the file
	char motion_type_desc[80]; //description of motion type read from the file
	lib::ECP_POSE_SPECIFICATION ps; //pose specification read from the file
	lib::MOTION_TYPE mt; //type of the commanded motion (relative or absolute)
	int number_of_poses = 0; //number of poses to be read
	int i, j; //loop counters

	std::ifstream from_file(file_name); // open the file
	if (!from_file.good()) {
		//perror(file_name);
		BOOST_THROW_EXCEPTION(exception::nfe_g() << lib::exception::mrrocpp_error0(NON_EXISTENT_FILE));
		return false;
	}

	if (!(from_file >> coordinate_type_desc)) {
		BOOST_THROW_EXCEPTION(exception::nfe_g() << lib::exception::mrrocpp_error0(READ_FILE_ERROR));
		return false;
	}

	//removing spaces and tabs
	i = 0;
	j = 0;
	while (coordinate_type_desc[i] == ' ' || coordinate_type_desc[i] == '\t')
		i++;
	while (coordinate_type_desc[i] != ' ' && coordinate_type_desc[i] != '\t' && coordinate_type_desc[i] != '\n'
			&& coordinate_type_desc[i] != '\r' && coordinate_type_desc[j] != '\0') {
		coordinate_type_desc[j] = toupper(coordinate_type_desc[i]);
		i++;
		j++;
	}
	coordinate_type_desc[j] = '\0';

	if (!strcmp(coordinate_type_desc, "MOTOR")) {
		ps = lib::ECP_MOTOR;
	} else if (!strcmp(coordinate_type_desc, "JOINT")) {
		ps = lib::ECP_JOINT;
	} else if (!strcmp(coordinate_type_desc, "XYZ_EULER_ZYZ")) {
		ps = lib::ECP_XYZ_EULER_ZYZ;
	} else if (!strcmp(coordinate_type_desc, "XYZ_ANGLE_AXIS")) {
		ps = lib::ECP_XYZ_ANGLE_AXIS;
	} else {
		BOOST_THROW_EXCEPTION(exception::nfe_g() << lib::exception::mrrocpp_error0(NON_TRAJECTORY_FILE));
		return false;
	}

	if (!(from_file >> number_of_poses)) {
		BOOST_THROW_EXCEPTION(exception::nfe_g() << lib::exception::mrrocpp_error0(READ_FILE_ERROR));
		return false;
	}

        flushall();
	if (!(from_file >> motion_type_desc)) {
		BOOST_THROW_EXCEPTION(exception::nfe_g() << lib::exception::mrrocpp_error0(READ_FILE_ERROR));
		return false;
	}

	if (!strcmp(motion_type_desc, "ABSOLUTE")) {
		mt = lib::ABSOLUTE;
	} else if (!strcmp(motion_type_desc, "RELATIVE")) {
		mt = lib::RELATIVE;
	} else {
		BOOST_THROW_EXCEPTION(exception::nfe_g() << lib::exception::mrrocpp_error0(NON_TRAJECTORY_FILE));
		return false;
	}

	double tab[10];
	int pos = from_file.tellg();
	char line[80];
	int dlugosc;
	do {
		from_file.getline(line, 80);
		dlugosc = strlen(line);
	} while (dlugosc < 5);
	int num = lib::setValuesInArray(tab, line);
	this->set_axes_num(num);
	from_file.seekg(pos);

	std::vector <double> v(axes_num); //vector of read velocities
	std::vector <double> a(axes_num); //vector of read accelerations
	std::vector <double> coordinates(axes_num); //vector of read coordinates

	for (i = 0; i < number_of_poses; i++) {
		for (j = 0; j < axes_num; j++) {
			if (!(from_file >> v[j])) { //protection before the non-numerical data
				BOOST_THROW_EXCEPTION(exception::nfe_g() << lib::exception::mrrocpp_error0(READ_FILE_ERROR));
				return false;
			}
		}
		from_file.ignore(std::numeric_limits <std::streamsize>::max(), '\n');
		for (j = 0; j < axes_num; j++) {
			if (!(from_file >> a[j])) { //protection before the non-numerical data
				BOOST_THROW_EXCEPTION(exception::nfe_g() << lib::exception::mrrocpp_error0(READ_FILE_ERROR));
				return false;
			}
		}
		from_file.ignore(std::numeric_limits <std::streamsize>::max(), '\n');
		for (j = 0; j < axes_num; j++) {
			if (!(from_file >> coordinates[j])) { //protection before the non-numerical data
				BOOST_THROW_EXCEPTION(exception::nfe_g() << lib::exception::mrrocpp_error0(READ_FILE_ERROR));
				return false;
			}
		}
		from_file.ignore(std::numeric_limits <std::streamsize>::max(), '\n');

		if (ps == lib::ECP_MOTOR) {
			load_trajectory_pose(coordinates, mt, ps, v, a, motor_max_velocity, motor_max_acceleration);
		} else if (ps == lib::ECP_JOINT) {
			load_trajectory_pose(coordinates, mt, ps, v, a, joint_max_velocity, joint_max_acceleration);
		} else if (ps == lib::ECP_XYZ_EULER_ZYZ) {
			load_trajectory_pose(coordinates, mt, ps, v, a, euler_zyz_max_velocity, euler_zyz_max_acceleration);
		} else if (ps == lib::ECP_XYZ_ANGLE_AXIS) {
			load_trajectory_pose(coordinates, mt, ps, v, a, angle_axis_max_velocity, angle_axis_max_acceleration);
		}
	}

	return true;
}

bool newsmooth::optimize_objective(std::vector<double> max_current, std::vector<double> max_current_change, std::vector<double> max_velocity, std::vector<double> max_acceleration, double stop_condition, double timeCoef)
{
    bool finish = false;

    int i, j;

    std::vector <double> temp1;
    std::vector <double> temp2;

    double control[axes_num];
    double control2[axes_num];
    double highest_current_change[axes_num];
    double highest_current[axes_num];
    bool max_current_change_exceeded = false;
    bool max_current_exceeded = false;

    for (i = 0; i < axes_num; i++)
    {
        control[i] = 0.0;
        control2[i] = 0.0;
        highest_current_change[i] = 0.0;
        highest_current[i] = 0.0;
    }

    if (debug) {
            printf("##################################### optimize #####################################\n");
    }

    if (!optimization || current_vector.size() <= 1)
    {
        sr_ecp_msg.message("Optimization not performed. Lack of data.");
        if (debug) {
                printf("Be sure that optimization is set to true and the motion was performed with more than 1 macrosteps.");
        }
        return true;
    }

    //calculation of the energy used during the last motion and its time of execution
    double energySum = 0;
    double timeSum = 0;


    //energy
    printf("energy vector size: %d\n", energy_vector.size());

    energy_vector_iterator = energy_vector.begin();

    for (i = 0; i < energy_vector.size() - 1; i++)
    {
        for (j = 0; j < axes_num; j++)
        {
            energySum += (*energy_vector_iterator)[j];
        }
        energy_vector_iterator++;
    }

    //time
    pose_vector_iterator = pose_vector.begin();

    for (i = 0; i < pose_vector.size(); i++)
    {
        timeSum += pose_vector_iterator->t;
        pose_vector_iterator++;
    }

    if (debug)
    {
        if (objective_vector.size() == 0)
        {
            std::ofstream rmDataFile(last_loaded_file_path + "-optimizedData", std::ios::out);
            rmDataFile << "";
        }
    }
    double objective = calculate_objective(energySum, timeSum, timeCoef);
    objective_vector.push_back(objective);
    time_cost.push_back(timeSum);
    energy_cost.push_back(energySum);

    printf("optimal vector size: %d\n", optimal_pose_vector.size());

    //if (check_if_lowest_energy_cost(energySum) == true)
    if (check_if_highest_objective(objective) == true)
    {
        printf("highest objective\n");
        //printf("lowerst obj function");
        if (optimal_pose_vector.size() == pose_vector.size())
        {
            optimal_pose_vector_iterator = optimal_pose_vector.begin();
            pose_vector_iterator = pose_vector.begin();

            for (i = 0; i < pose_vector.size(); i++)
            {
                for (j = 0; j < axes_num; j++)
                {
                    optimal_pose_vector_iterator->v[j] = pose_vector_iterator->v[j];
                    optimal_pose_vector_iterator->a[j] = pose_vector_iterator->a[j];
                }

                pose_vector_iterator++;
                optimal_pose_vector_iterator++;
            }
        }
    }

    if (debug)
    {
        pose_vector_iterator = pose_vector.begin();

        printf ("########### Energy consumption: %f \n", energySum);

        if (last_loaded_file_path == "")
        {
            last_loaded_file_path = "../../src/application/generator_tester/trajectory.trj";
        }

        std::ofstream outDataFile(last_loaded_file_path + "-optimizedData", std::ios::app);
        outDataFile << energySum << ";";
        outDataFile << timeSum << ";";
        outDataFile << objective << ";";

        for (j = 0; j < pose_vector.size(); j++)
        {
            //printf("\n%d:\n", j);
            //printf("v:\t");
            for (i = 0; i < axes_num; i++)
            {
               //printf("%f\t", pose_vector_iterator->v[i]);
               outDataFile << pose_vector_iterator->v[i] << ";";
            }
            //printf("\n");
            //printf("a:\t");
            for (i = 0; i < axes_num; i++)
            {
                //printf("%f\t", pose_vector_iterator->a[i]);
                outDataFile << pose_vector_iterator->a[i] << ";";
            }
            //printf("\n");
            flushall();
            pose_vector_iterator++;
        }
        outDataFile << "\n";
    }

    double current_macrostep_in_pose = 1;

    pose_vector_iterator = pose_vector.begin();

    current_vector_iterator = current_vector.begin();

    temp1 = (*current_vector_iterator);

    current_vector_iterator++;

    temp2 = (*current_vector_iterator);

    if (debug)
    {
        printf("---- pose %d\n", pose_vector_iterator->pos_num);
    }

    for (i = 0; i < current_vector.size() - 1; i++)
    {
        if (current_macrostep_in_pose == pose_vector_iterator->interpolation_node_no-1)
        {
            for (j = 0; j < axes_num; j++)
            {

                if (control[j] < 0 && pose_vector_iterator->v[j] != 0.01 && pose_vector_iterator->a[j] != 0.01)
                {
                    max_current_change_exceeded = true;
                }

                if (control2[j] < 0 && pose_vector_iterator->v[j] != 0.01 && pose_vector_iterator->a[j] != 0.01)
                {
                    max_current_exceeded = true;
                }

                pose_vector_iterator->v[j] += (max_velocity[j] * control[j] / max_current_change[j]) * 0.4;
                pose_vector_iterator->a[j] += (max_acceleration[j] * control[j] / max_current_change[j]) * 0.4;
                printf("control[%d]: %f\n", j, (max_velocity[j] * control[j] / max_current_change[j]) * 0.4);

                if (control2[j] < 0)
                {
                    printf("to high current\n");
                    pose_vector_iterator->v[j] += (max_velocity[j] * control2[j] / max_current_change[j]) * 0.4;
                    pose_vector_iterator->a[j] += (max_acceleration[j] * control2[j] / max_current_change[j]) * 0.4;
                }

                //finish = false;
                if (pose_vector_iterator->v[j] >= max_velocity[j])
                {
                    pose_vector_iterator->v[j] = max_velocity[j];
                    //finish = true;
                }
                else if (pose_vector_iterator->v[j] <= 0.01)
                {
                    pose_vector_iterator->v[j] = 0.01;
                }
                if (pose_vector_iterator->a[j] >= max_acceleration[j])
                {
                    pose_vector_iterator->a[j] = max_acceleration[j];
                    //finish = true;
                }
                else if (pose_vector_iterator->a[j] <= 0.01)
                {
                    pose_vector_iterator->a[j] = 0.01;
                }
                //}
            }

            if (debug)
            {
                for (j = 0; j < axes_num; j++)
                {
                    printf("max current peak: %d: %f\t", j, highest_current_change[j]);
                    printf("max current: %d: %f\t", j, highest_current[j]);
                    printf("v[%d]: %f\t", j, pose_vector_iterator->v[j]);
                    printf("a[%d]: %f\n", j, pose_vector_iterator->a[j]);
                }
            }

            if (pose_vector_iterator->pos_num == pose_vector.size())
            {
                break;
            }

            for (i = 0; i < axes_num; i++)
            {
                control[i] = 0.0;
                control2[i] = 0.0;
                highest_current_change[i] = 0.0;
                highest_current[i] = 0.0;
            }

            pose_vector_iterator++;

            if (debug)
            {
                printf("---- pose %d\n", pose_vector_iterator->pos_num);
            }

            current_macrostep_in_pose = 1;
        }

        for (j = 0; j < axes_num; j++)
        {
            if (fabs(temp2[j] - temp1[j]) > highest_current_change[j])
            {
                highest_current_change[j] = fabs(temp2[j] - temp1[j]);
                control[j] = max_current_change[j] - fabs(temp2[j] - temp1[j]);

            }

            if (fabs(temp2[j]) > highest_current[j])
            {
                highest_current[j] = fabs(temp2[j]);
                control2[j] = max_current[j] - fabs(temp2[j]);
                //printf("max current %f\n", max_current[j]);
                //printf("current %f\n", fabs(temp2[j]));

            }
        }

        current_vector_iterator++;

        temp1 = temp2;

        temp2 = *current_vector_iterator;

        current_macrostep_in_pose++;
    }

    double cost_change1;
    double cost_change2;
    double cost_change3;

    if (objective_vector.size() > 3)
    {
        cost_change1 = fabs(objective_vector[objective_vector.size()-1])/fabs(objective_vector[objective_vector.size()-2]);
        cost_change2 = fabs(objective_vector[objective_vector.size()-2])/fabs(objective_vector[objective_vector.size()-3]);
        cost_change3 = fabs(objective_vector[objective_vector.size()-3])/fabs(objective_vector[objective_vector.size()-4]);
        printf("cost change 1: %f\n", cost_change1);
        printf("cost change 2: %f\n", cost_change2);
        printf("cost change 3: %f\n", cost_change3);
    }


    if (objective_vector.size() > 3 &&
        cost_change1 < (1.0 + stop_condition) && cost_change1 > (1.0 - stop_condition) &&
        cost_change2 < (1.0 + stop_condition) && cost_change2 > (1.0 - stop_condition) &&
        cost_change3 < (1.0 + stop_condition) && cost_change3 > (1.0 - stop_condition) &&
        max_current_change_exceeded == false && max_current_exceeded == false)
        //check if optimized
    {
        finish = true;
    }

    optimal_pose_vector_iterator = optimal_pose_vector.begin();

    if (finish == true)
    {
        if (last_loaded_file_path == "")
        {
            last_loaded_file_path = "../../src/application/generator_tester/trajectory.trj";
        }

        std::ofstream outfile(last_loaded_file_path + "-optimized", std::ios::out);

        if (pose_spec == lib::ECP_JOINT)
        {
            outfile << "JOINT\n";
        }
        else if (pose_spec == lib::ECP_MOTOR)
        {
            outfile << "MOTOR\n";
        }
        else
        {
            outfile << "\n";
        }

        outfile << ((int) pose_vector.size()) << "\n";

        if (motion_type == lib::ABSOLUTE)
        {
            outfile << "ABSOLUTE\n\n";
        }
        else if (motion_type == lib::RELATIVE)
        {
            outfile << "RELATIVE\n\n";
        }

        optimal_pose_vector_iterator = optimal_pose_vector.begin();

        for (i = 0; i < optimal_pose_vector.size(); i++)
        {
            for (j = 0; j < axes_num; j++)
            {
                outfile << optimal_pose_vector_iterator->v[j] << "\t";
            }

            outfile << "\n";

            for (j = 0; j < axes_num; j++)
            {
                outfile << optimal_pose_vector_iterator->a[j] << "\t";
            }

            outfile << "\n";

            for (j = 0; j < axes_num; j++)
            {
                outfile << optimal_pose_vector_iterator->coordinates[j] << "\t";
            }

            outfile << "\n";
            outfile << "\n";

            optimal_pose_vector_iterator++;
        }

        sr_ecp_msg.message("Optimized!");
    }

    print_energy_cost();
    print_time_cost();
    print_objective();

    return finish;
}

void newsmooth::optimize_objective(std::vector<double> startPos, std::vector<double> max_current, std::vector<double>max_current_change, std::vector<double>max_velocity, std::vector<double>max_acceleration, double stop_condition, boost::shared_ptr <newsmooth> sgenstart, const char *file_name, double timeCoef)
{
    reset();
    set_optimization(true);
    load_trajectory_from_file(file_name);

    sgenstart->load_absolute_joint_trajectory_pose(startPos);
    sgenstart->calculate_interpolate();
    sgenstart->Move();

    if (calculate_interpolate()/* && sgenjoint->detect_jerks(1) == 0*/) {
            Move();

            //sgenstart->set_debug(true);

            while (!optimize_objective(max_current, max_current_change, max_velocity, max_acceleration, stop_condition, timeCoef)) {
                    sr_ecp_msg.message("Optimizing...");

                    sgenstart->load_absolute_joint_trajectory_pose(startPos);
                    sgenstart->calculate_interpolate();
                    sgenstart->Move();

                    if (calculate_interpolate())
                    {
                        Move();
                    }
            }

            //sgenstart->load_absolute_joint_trajectory_pose(startPos);
            //sgenstart->calculate_interpolate();
            //sgenstart->Move();
    }

    set_optimization(false);
    reset();
}

void newsmooth::optimize_objectivet_postument(boost::shared_ptr <newsmooth> sgenstart, const char *file_name, std::vector<double> start_pos, double stop_condition, double timeCoef)
{
    std::vector <double> max_current = std::vector <double>(6);
    max_current[0] = 13000;
    max_current[1] = 16000;
    max_current[2] = 8000;
    max_current[3] = 8000;
    max_current[4] = 8000;
    max_current[5] = 8000;

    std::vector <double> max_current_change = std::vector <double>(6);
    max_current_change[0] = 4000;
    max_current_change[1] = 3000;
    max_current_change[2] = 2500;
    max_current_change[3] = 2500;
    max_current_change[4] = 2500;
    max_current_change[5] = 600;

    std::vector <double> max_velocity = std::vector <double>(6);
    max_velocity[0] = 0.5;
    max_velocity[1] = 0.5;
    max_velocity[2] = 0.5;
    max_velocity[3] = 0.5;
    max_velocity[4] = 0.5;
    max_velocity[5] = 0.5;

    std::vector <double> max_acceleration = std::vector <double>(6);
    max_acceleration[0] = 0.2;
    max_acceleration[1] = 0.2;
    max_acceleration[2] = 0.2;
    max_acceleration[3] = 0.2;
    max_acceleration[4] = 0.2;
    max_acceleration[5] = 0.2;

    optimize_objective(start_pos, max_current, max_current_change, max_velocity, max_acceleration, stop_condition, sgenstart, file_name, timeCoef);
}

void newsmooth::optimize_objective_track(boost::shared_ptr <newsmooth> sgenstart, const char *file_name, std::vector<double> start_pos, double stop_condition, double timeCoef)
{
    std::vector <double> max_current = std::vector <double>(7);
    max_current[0] = 22000;
    max_current[1] = 13000;
    max_current[2] = 16000;
    max_current[3] = 8000;
    max_current[4] = 8000;
    max_current[5] = 8000;
    max_current[6] = 8000;

    std::vector <double> max_current_change = std::vector <double>(7);
    max_current_change[0] = 7000;
    max_current_change[1] = 4000;
    max_current_change[2] = 3000;
    max_current_change[3] = 2500;
    max_current_change[4] = 2500;
    max_current_change[5] = 2500;
    max_current_change[6] = 600;

    std::vector <double> max_velocity = std::vector <double>(7);
    max_velocity[0] = 0.05;
    max_velocity[1] = 0.5;
    max_velocity[2] = 0.5;
    max_velocity[3] = 0.5;
    max_velocity[4] = 0.5;
    max_velocity[5] = 0.5;
    max_velocity[6] = 0.5;

    std::vector <double> max_acceleration = std::vector <double>(7);
    max_acceleration[0] = 0.01;
    max_acceleration[1] = 0.2;
    max_acceleration[2] = 0.2;
    max_acceleration[3] = 0.2;
    max_acceleration[4] = 0.2;
    max_acceleration[5] = 0.2;
    max_acceleration[6] = 0.2;

    optimize_objective(start_pos, max_current, max_current_change, max_velocity, max_acceleration, stop_condition, sgenstart, file_name, timeCoef);
}

//--------------- METHODS USED TO LOAD POSES END ----------------

}// namespace generator
} // namespace common
} // namespace ecp
} // namespace mrrocpp
