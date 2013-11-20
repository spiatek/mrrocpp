// -------------------------------------------------------------------------
//                            ui_class.h
// Definicje klasy Ui
//
// Ostatnia modyfikacja: 2010
// -------------------------------------------------------------------------

#ifndef __UI_R_IRP6_M_H
#define __UI_R_IRP6_M_H

#include "../base/ui.h"
#include "../base/ui_r_common_012.h"

class wgt_irp6_m_joints;
class wgt_irp6_m_motors;
class wgt_irp6_m_angle_axis;
class wgt_irp6_m_euler;
class wgt_irp6_m_relative_angle_axis;
class wgt_irp6_m_tool_angle_axis;
class wgt_irp6_m_tool_euler;

namespace Ui {
class MenuBar;
class MenuBarAction;
}

namespace mrrocpp {
namespace ui {
namespace common {
class Interface;
}

namespace irp6_m {

//
//
// KLASA UiRobotIrp6_m
//
//

class UiRobot : public common_012::UiRobot
{
public:

	UiRobot(common::Interface& _interface, lib::robot_name_t _robot_name, int _number_of_servos);

	void setup_menubar();

	void manage_interface();

	int synchronise_int();
	virtual void move_to_preset_position(int variant);

	virtual void synchronise();

	int execute_motor_motion();
	int execute_joint_motion();

	const static std::string WGT_JOINTS;
	const static std::string WGT_MOTORS;
	const static std::string WGT_ANGLE_AXIS;
	const static std::string WGT_EULER;
	const static std::string WGT_RELATIVE_ANGLE_AXIS;
	const static std::string WGT_TOOL_ANGLE_AXIS;
	const static std::string WGT_TOOL_EULER;

protected:

private:

	QAction *action_Pre_Synchro_Moves_Motors;
	QAction *action_Joints;
	QAction *action_Absolute_Moves_Xyz_Euler_Zyz;
	QAction *action_Absolute_Moves_Xyz_Angle_Axis;
	QAction *action_Xyz_Relative_Moves_Angle_Axis;
	QAction *action_Tool_Xyz_Euler_Zyz;
	QAction *action_Tool_Xyz_Angle_Axis;
	QAction *action_Absolute_Moves_Motors;
	QAction *action_Front_Position;

	QMenu *menu_Absolute_Moves;
	QMenu *menu_Relative_Moves;
	QMenu *menu_Tool;

};

}
} //namespace ui
} //namespace mrrocpp

#endif

