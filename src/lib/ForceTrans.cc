#include "lib/ForceTrans.h"


int debugi = 1;

/*
ForceTrans::ForceTrans(const Homog_matrix & init_frame, const Homog_matrix & s_frame)
{
	initialized = false;
	sensor_frame = s_frame;
	synchro(init_frame);
	double arm[3] = { X_AXIS_ARM , Y_AXIS_ARM , Z_AXIS_ARM };
	K_vector point_of_gravity(arm);
	double weight = Z_AXIS_GRAVITY_FORCE;
	defineTool(weight, point_of_gravity);
	initialized = true;
}
*/

ForceTrans::ForceTrans(const short l_force_sensor_name, const Homog_matrix & init_frame, const Homog_matrix & s_frame, 
	const double weight, const K_vector & point_of_gravity) : force_sensor_name(l_force_sensor_name), initialized (false)
{

	sensor_frame = s_frame;
//	sensor_frame_translation = Homog_matrix (sensor_frame.return_with_with_removed_rotation());
	// sensor_frame_translation.remove_rotation();
//	sensor_frame_rotation  = Homog_matrix (sensor_frame.return_with_with_removed_translation());
	// sensor_frame_rotation.remove_translation();
	// cout << sensor_frame;

	ft_tr_sensor_in_wrist = Ft_v_tr (sensor_frame, Ft_v_tr::FT);

	// ft_tr_inv_sensor_translation_matrix = !ft_tr_sensor_translation_matrix;
//	ft_tr_sensor_rotation_matrix = Ft_v_tr (sensor_frame_rotation, Ft_v_tr::FT);;
	// ft_tr_inv_sensor_rotation_matrix = !ft_tr_sensor_rotation_matrix;

	tool_weight = weight;
	gravity_arm_in_wrist = point_of_gravity;
	
	synchro(init_frame);
	defineTool(init_frame, weight, point_of_gravity);
	initialized = true;
}


void ForceTrans::defineTool(const Homog_matrix & init_frame, const double weight, const K_vector & point_of_gravity)
{
	tool_weight = weight;
	gravity_arm_in_wrist = point_of_gravity;
//	gravity_force_in_base = K_vector (0.0, 0.0, tool_weight);
	gravity_force_torque_in_base = Ft_v_vector (0.0, 0.0, -tool_weight, 0.0, 0.0, 0.0);

	//	frame_tab sens_rot = {{0,-1,0},{1,0,0},{0,0,1},{0,0,0}};
	//	Homog_matrix sensor_rotation = Homog_matrix(sens_rot);
	// orientacja koncowki manipulatora bez narzedzia	
	Homog_matrix current_orientation (init_frame.return_with_with_removed_translation());
	// cout << current_orientation << endl;
	// current_orientation.remove_translation(); // elminacja skladowej polozenia
	//	cout <<"aaaa"<<	endl << sensor_frame <<endl << sensor_rotation<<endl ;
	//	K_vector gravity_force_in_sensor = (!(orientation*sensor_rotation))*gravity_force_in_base;
	// wyznaczenie sily grawitacji i z jej pomoca sil i momentow
//	 K_vector gravity_force_in_sensor = (!current_orientation)*gravity_force_in_base;
	Ft_v_vector gravity_force_torque_in_sensor (Ft_v_tr(!current_orientation, Ft_v_tr::FT)*gravity_force_torque_in_base);
	//	cout << orientation << endl;
	// wzynaczenie macierzy transformacji sil dla danego polozenia srodka ciezkosci narzedzia wzgledem czujnika
	Homog_matrix tool_mass_center_translation (point_of_gravity[0], point_of_gravity[1], point_of_gravity[2]);
	ft_tool_mass_center_translation = Ft_v_tr(tool_mass_center_translation, Ft_v_tr::FT);
//	cout << tool_mass_center_translation << endl;

//	reaction_torque_in_sensor = K_vector((gravity_force_in_sensor*gravity_arm_in_sensor)*(-1));
//	reaction_force_in_sensor = K_vector(gravity_force_in_sensor*(-1));  


	// reaction_torque_in_sensor = K_vector((gravity_force_torque_in_sensor.get_force_K_vector()*gravity_arm_in_sensor)*(-1));
	// reaction_force_in_sensor = K_vector(gravity_force_torque_in_sensor.get_force_K_vector()*(-1));

//	cout << "aa:" << reaction_force_in_sensor << reaction_torque_in_sensor << endl;
	// wyznaczenie sil reakcji
	reaction_force_torque_in_sensor = - (ft_tool_mass_center_translation*gravity_force_torque_in_sensor);
	
	
//	reaction_force_in_sensor = reaction_force_torque_in_sensor.get_force_K_vector();
//	reaction_torque_in_sensor = reaction_force_torque_in_sensor.get_torque_K_vector();
	
//	cout << "bb:" << reaction_force_torque_in_sensor << endl;
}



// zeraca sily i momenty sil w w ukladzie z orientacja koncowki manipulatory bez narzedzia
double* ForceTrans::getForce(const double inputForceTorque[6], const Homog_matrix & orientation)
{
	static long deblicz=0;

	static int last_debugi = 0;

	if (initialized)
	{
		deblicz++;
/*
		K_vector input_force((double*) inputForceTorque);
		K_vector input_torque((double*) inputForceTorque+3);
*/
		// sprowadzenie sil i momentow sil do ukladu umieszczonego w srodku czujnika ale z orientacja koncowki
		Ft_v_vector input_force_torque (ft_tr_sensor_in_wrist * Ft_v_vector ((double*) inputForceTorque));
		/*
		if ((debugi%10==0)&&(force_sensor_name==FORCE_SENSOR_ATI3084)&&(last_debugi!=debugi))
		{
			printf("ft: ");
			input_force_torque.wypisz_wartosc_na_konsole();
		}
*/
//		if ((deblicz%100) == 0)cout << "if" << input_force << endl;
		// by Y na podstawie kodu Slawka
		/*
		input_force = sensor_frame_rotation*input_force;
		input_torque = sensor_frame_rotation*input_torque;
		*/
		
//		K_vector input_force = input_force_torque.get_force_K_vector();
//		K_vector input_torque = input_force_torque.get_torque_K_vector();
		
//		if ((deblicz%100) == 0)cout << "af" << input_force << endl;
		//end by Y
//		frame_tab rot = {{0,-1,0},{1,0,0},{0,0,1},{0,0,0}};
//		Homog_matrix sensor_rotation(rot);
		// orientacja koncowki manipulatora bez narzedzia
		Homog_matrix current_orientation (orientation.return_with_with_removed_translation());
//			cout << current_orientation << endl;
		// current_orientation.remove_translation (); // elminacja skladowej polozenia
//		cout <<"bbbbb"<<	endl << sensor_frame <<endl << sensor_rotation;
//		K_vector gravity_force_in_sensor = (!(current_orientation*sensor_rotation))*gravity_force_in_base;
		// wyznaczenie sily grawitacji i z jej pomoca sil i momentow w kisci
		
//		K_vector gravity_force_in_sensor = (!current_orientation)*gravity_force_in_base;
		Ft_v_vector gravity_force_torque_in_sensor (Ft_v_tr(!current_orientation, Ft_v_tr::FT)*gravity_force_torque_in_base);
		
		// cout << gravity_force_in_sensor << endl;
		// uwzglednienie w odczytach sily grawitacji i sily reakcji
//		K_vector output_force = input_force - gravity_force_in_sensor - reaction_force_in_sensor;
//		K_vector output_torque = input_torque - (gravity_force_in_sensor*gravity_arm_in_sensor) - reaction_torque_in_sensor;
	//	if ((deblicz%100) == 0) cout << "aa:" << output_force << output_torque << endl;
		Ft_v_vector output_force_torque (input_force_torque - (ft_tool_mass_center_translation*gravity_force_torque_in_sensor)
			 - reaction_force_torque_in_sensor);
			 
			 
//		if ((deblicz%100) == 0) cout << "bb:" << output_force_torque << endl;
		// sprowadzenie sil i momentow sil do ukladu umieszczonego w koncowce kinematyki i z jej orientacja
				// sprowadzenie sil i momentow sil do ukladu umieszczonego w koncowce kinematyki ale z orientacja ukladu bazowego
		/*
		output_force_torque = ft_tr_sensor_translation_matrix * 
				output_force_torque;
		*/
			

		output_force_torque = Ft_v_tr (current_orientation, Ft_v_tr::FT) * (-output_force_torque);	

//		Ft_v_vector tmp_force_torque = Ft_v_tr (current_orientation*sensor_frame_translation, FT_VARIANT) * output_force_torque;	
		
/*
		if ((debugi%10==0)&&(force_sensor_name==FORCE_SENSOR_ATI3084)&&(last_debugi!=debugi))
		{
			printf("ft2: ");
			output_force_torque.wypisz_wartosc_na_konsole();
			printf("ft3: ");
			tmp_force_torque.wypisz_wartosc_na_konsole();
		}
		*/

//		output_force_torque = Ft_v_tr (sensor_frame_translation) * Ft_v_vector(output_force, output_torque);
		// sprowadzenie sil i momentow sil do ukladu umieszczonego w koncowce kinematyki ale z orientacja ukladu bazowego
		// output_force_torque = Ft_v_tr (current_orientation) * output_force_torque;
		// 
				
		/*
		output_force = (current_orientation)*output_force;
		output_torque = (current_orientation)*output_torque;
		*/
		 // if ((deblicz%100) == 0) cout << output_force_torque << endl;

		// przygotowania tablicy wynikowej
		double *outputForceTorque = new double[6];
		output_force_torque.to_table(outputForceTorque);
		/*
		output_force.to_table(outputForceTorque);
//			if ((deblicz%100) == 0)cout << "of" << output_force << endl;
		output_torque.to_table(outputForceTorque+3);
*/
//		Ft_v_vector output_force_torque = force_from_sensor_to_tool_transform * Ft_v_vector(ft_table);
//		output_force_torque.to_table(outputForceTorque);
/*		output_force.to_table(outputForceTorque);
		output_torque.to_table(outputForceTorque+3);
		input_force.to_table(outputForceTorque+6);
		input_torque.to_table(outputForceTorque+9);
		gravity_force_in_sensor.to_table(outputForceTorque+12);
		(gravity_force_in_sensor*gravity_arm_in_sensor).to_table(outputForceTorque+15);
		reaction_force_in_sensor.to_table(outputForceTorque+18);
		reaction_torque_in_sensor.to_table(outputForceTorque+21);
		for(int j=0;j<4;j++)
			for(int i=j*6;i<j*6+3;i++) {
				outputForceTorque[i]*=20;
				outputForceTorque[i+3]*=333;
			}	
						
*/
		last_debugi = debugi;
		return outputForceTorque;
	}
	return 0;
}


void ForceTrans::synchro(const Homog_matrix & init_frame)
{

	//initialisation_frame = init_frame;
	if (initialized) defineTool (init_frame, tool_weight, gravity_arm_in_wrist);
 }
