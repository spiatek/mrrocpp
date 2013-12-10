// -------------------------------------------------------------------------
//                            ui_class.h
// Definicje klasy Ui
//
// Ostatnia modyfikacja: 2010
// -------------------------------------------------------------------------

#ifndef __UI_R_SARKOFAG_H
#define __UI_R_SARKOFAG_H

#include <QObject>
#include <QMenu>
#include "../base/ui.h"
#include "../base/ui_r_common_012.h"
#include "robot/sarkofag/const_sarkofag.h"

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
namespace sarkofag {

//
//
// KLASA UiRobotSarkofag
//
//

// super klasa agregujaca porozrzucane struktury

class UiRobot : public common_012::UiRobot
{
Q_OBJECT

public:

	UiRobot(common::Interface& _interface);

	void manage_interface();

	void synchronise();

	int synchronise_int();

	int execute_motor_motion();
	int execute_joint_motion();

	void create_ui_ecp_robot();
	void edp_create_int_extra_operations();

	void setup_menubar();

private:
	QAction *actionsarkofag_Move;

};

}
} //namespace ui
} //namespace mrrocpp

#endif

