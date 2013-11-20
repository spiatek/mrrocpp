// -------------------------------------------------------------------------
//                            ui_class.h
// Definicje klasy Ui
//
// Ostatnia modyfikacja: 2010
// -------------------------------------------------------------------------

#ifndef __UI_R_CONVEYOR_H
#define __UI_R_CONVEYOR_H

#include <QObject>
#include <QMenu>
#include "../base/ui.h"
#include "../base/ui_r_common_012.h"
#include "robot/conveyor/const_conveyor.h"

namespace Ui {
class MenuBar;
class MenuBarAction;
}

class wgt_single_motor_move;

namespace mrrocpp {
namespace ui {
namespace common {
class Interface;

}
namespace common_012 {
class EcpRobot;
}

namespace conveyor {

//
// KLASA UiRobotConveyor
//
//

class UiRobot : public common_012::UiRobot
{
Q_OBJECT
private:

public:

	UiRobot(common::Interface& _interface);

	void manage_interface();

	void synchronise();

	int synchronise_int();

	void create_ui_ecp_robot();
	void edp_create_int_extra_operations();

	void setup_menubar();

private:

	QAction *actionconveyor_Move;

};

}
} //namespace ui
} //namespace mrrocpp

#endif

