/* Y o u r   D e s c r i p t i o n                       */
/*                            AppBuilder Photon Code Lib */
/*                                         Version 2.01  */

#include "ui_r_common_012.h"
#include "ui_ecp_robot/ui_ecp_r_common_012.h"
#include "interface.h"

#include "wgt_kinematic.h"
#include "wgt_servo_algorithm.h"
#include "mainwindow.h"
#include "menu_bar.h"
#include "menu_bar_action.h"
#include "mp.h"

namespace mrrocpp {
namespace ui {
namespace common_012 {

// extern ui_state_def ui_state;

//
//
// KLASA UiRobotIrp6ot_m
//
//

UiRobot::UiRobot(common::Interface& _interface, lib::robot_name_t _robot_name, int _number_of_servos) :
		common::UiRobot(_interface, _robot_name, _number_of_servos)
{
	add_wgt <wgt_kinematic>(WGT_KINEMATIC, "Kinematic");
	add_wgt <wgt_servo_algorithm>(WGT_SERVO_ALGORITHM, "Servo Algorithm");
}

void UiRobot::unsynchronise()
{
	msg->message(lib::NON_FATAL_ERROR, "unsynchronise begin");
	if ((is_edp_loaded()) && (state.edp.is_synchronised == true)) {

		msg->message(lib::NON_FATAL_ERROR, "unsynchronise inside");
		std::cout << "ui unsynchronise()" << std::endl;
		close_all_windows();
		ui_ecp_robot->ecp->unsynchronise();
		get_edp_state();
		interface.manage_interface();
		std::cout << "ui unsynchronise() end" << std::endl;
		msg->message(lib::NON_FATAL_ERROR, "unsynchronise end");
	}
}

void UiRobot::setup_menubar()
{
	common::UiRobot::setup_menubar();
	Ui::MenuBar *menuBar = interface.get_main_window()->getMenuBar();
	Ui::SignalDispatcher *signalDispatcher = interface.get_main_window()->getSignalDispatcher();

	action_UnSynchronisation = new Ui::MenuBarAction(QString("&UnSynchronisation"), this, menuBar);
	action_Synchronisation = new Ui::MenuBarAction(QString("&Synchronisation"), this, menuBar);
	action_Kinematics = new Ui::MenuBarAction(QString("&Kinematic"), wgts[WGT_KINEMATIC], signalDispatcher, menuBar);
	action_Servo_Algorithm =
			new Ui::MenuBarAction(QString("Servo Algorit&hm"), wgts[WGT_SERVO_ALGORITHM], signalDispatcher, menuBar);
	action_Synchro_Position = new Ui::MenuBarAction(QString("&Synchro Position"), this, menuBar);
	action_Position_0 = new Ui::MenuBarAction(QString("Position &0"), this, menuBar);
	action_Position_1 = new Ui::MenuBarAction(QString("Position &1"), this, menuBar);
	action_Position_2 = new Ui::MenuBarAction(QString("Position &2"), this, menuBar);
	menu_Pre_Synchro_Moves = new QMenu(robot_menu);
	menu_Preset_Positions = new QMenu(robot_menu);
	menu_Special = new QMenu(robot_menu);
	robot_menu->addSeparator();

	robot_menu->addAction(menu_Pre_Synchro_Moves->menuAction());
	menu_Pre_Synchro_Moves->addAction(action_Synchronisation);

	robot_menu->addAction(menu_Preset_Positions->menuAction());
	menu_Preset_Positions->addAction(action_Synchro_Position);
	menu_Preset_Positions->addAction(action_Position_0);
	menu_Preset_Positions->addAction(action_Position_1);
	menu_Preset_Positions->addAction(action_Position_2);

	menu_Pre_Synchro_Moves->setTitle(QApplication::translate("MainWindow", "P&re Synchro Moves", 0, QApplication::UnicodeUTF8));
	menu_Preset_Positions->setTitle(QApplication::translate("MainWindow", "&Preset positions", 0, QApplication::UnicodeUTF8));
	robot_menu->addSeparator();
	robot_menu->addAction(menu_Special->menuAction());
	menu_Special->addAction(action_UnSynchronisation);
	menu_Special->setTitle(QApplication::translate("MainWindow", "&Special", 0, QApplication::UnicodeUTF8));
	robot_menu->addAction(action_Kinematics);
	robot_menu->addAction(action_Servo_Algorithm);
	robot_menu->addSeparator();
	// connections
	connect(action_UnSynchronisation, SIGNAL(triggered(mrrocpp::ui::common::UiRobot*)), signalDispatcher, SLOT(on_UnSynchronisation_triggered(mrrocpp::ui::common::UiRobot*)), Qt::AutoCompatConnection);

	// connections
	connect(action_Synchronisation, SIGNAL(triggered(mrrocpp::ui::common::UiRobot*)), signalDispatcher, SLOT(on_Synchronisation_triggered(mrrocpp::ui::common::UiRobot*)), Qt::AutoCompatConnection);
	connect(action_Synchro_Position, SIGNAL(triggered(mrrocpp::ui::common::UiRobot*)), signalDispatcher, SLOT(on_Synchro_Position_triggered(mrrocpp::ui::common::UiRobot*)), Qt::AutoCompatConnection);
	connect(action_Position_0, SIGNAL(triggered(mrrocpp::ui::common::UiRobot*)), signalDispatcher, SLOT(on_Position_0_triggered(mrrocpp::ui::common::UiRobot*)), Qt::AutoCompatConnection);
	connect(action_Position_1, SIGNAL(triggered(mrrocpp::ui::common::UiRobot*)), signalDispatcher, SLOT(on_Position_1_triggered(mrrocpp::ui::common::UiRobot*)), Qt::AutoCompatConnection);
	connect(action_Position_2, SIGNAL(triggered(mrrocpp::ui::common::UiRobot*)), signalDispatcher, SLOT(on_Position_2_triggered(mrrocpp::ui::common::UiRobot*)), Qt::AutoCompatConnection);

}

void UiRobot::manage_interface()
{

	common::UiRobot::manage_interface();

	switch (state.edp.state)
	{
		case common::UI_EDP_INACTIVE:

			break;
		case common::UI_EDP_OFF:
			menu_Preset_Positions->setEnabled(false);
			menu_Pre_Synchro_Moves->setEnabled(false);
			menu_Special->setEnabled(false);
			action_Kinematics->setEnabled(false);
			action_Servo_Algorithm->setEnabled(false);
			break;
		case common::UI_EDP_WAITING_TO_START_READER:
		case common::UI_EDP_WAITING_TO_STOP_READER:

			// jesli robot jest zsynchronizowany
			if (state.edp.is_synchronised) {
				menu_Pre_Synchro_Moves->setEnabled(false);

				switch (interface.mp->mp_state.state)
				{
					case common::UI_MP_NOT_PERMITED_TO_RUN:
					case common::UI_MP_PERMITED_TO_RUN:
						menu_Special->setEnabled(true);
						menu_Preset_Positions->setEnabled(true);
						action_Kinematics->setEnabled(true);
						action_Servo_Algorithm->setEnabled(true);
						break;
					case common::UI_MP_WAITING_FOR_START_PULSE:
						menu_Special->setEnabled(false);
						menu_Preset_Positions->setEnabled(true);
						action_Kinematics->setEnabled(true);
						action_Servo_Algorithm->setEnabled(true);
						break;
					case common::UI_MP_TASK_RUNNING:
					case common::UI_MP_TASK_PAUSED:
						menu_Special->setEnabled(false);
						menu_Preset_Positions->setEnabled(false);
						action_Kinematics->setEnabled(false);
						action_Servo_Algorithm->setEnabled(false);
						break;
					default:
						break;
				}

			} else // jesli robot jest niezsynchronizowany
			{
				menu_Special->setEnabled(false);
				menu_Pre_Synchro_Moves->setEnabled(true);
				menu_Preset_Positions->setEnabled(false);
				action_Kinematics->setEnabled(true);
				action_Servo_Algorithm->setEnabled(true);
			}
			break;
		default:
			break;

	}

}

}
}
}
