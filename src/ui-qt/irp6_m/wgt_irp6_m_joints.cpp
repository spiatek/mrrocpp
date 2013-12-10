//#include "ui_ecp_r_irp6_m.h"
#include "ui_r_irp6_m.h"

//#include "ui/src/ui_ecp_r_single_motor.h"
#include "../base/ui_ecp_robot/ui_ecp_r_common_012.h"
#include "wgt_irp6_m_joints.h"
#include "../base/interface.h"
#include "../base/mainwindow.h"
#include "../base/ui_robot.h"

wgt_irp6_m_joints::wgt_irp6_m_joints(QString _widget_label, mrrocpp::ui::common::Interface& _interface, mrrocpp::ui::common::UiRobot *_robot, QWidget *parent) :
		WgtAbsoluteBase(_widget_label, _interface, _robot, parent)
{
	ui.setupUi(this);
	specyficrobot = dynamic_cast <mrrocpp::ui::irp6_m::UiRobot *>(_robot);

	setup_ui(ui.grid_up, robot->number_of_servos);

	for (int i = 0; i < robot->number_of_servos; ++i) {
		axis_labels[i]->setText(QString("q%1").arg(i));
	}
}

void wgt_irp6_m_joints::setup_ui(QGridLayout *layout, int _rows_number)
{
	WgtAbsoluteBase::setup_ui(layout, _rows_number);

}

wgt_irp6_m_joints::~wgt_irp6_m_joints()
{

}

void wgt_irp6_m_joints::init()
{

	try {

		if (robot->state.edp.pid != -1) {
			if (robot->state.edp.is_synchronised) // Czy robot jest zsynchronizowany?
			{
				ui.pushButton_execute->setDisabled(false);
				specyficrobot->ui_ecp_robot->read_joints(robot->current_pos);
				std::cout << "init ok. wartosci current pos:" << std::endl;
				for (int i = 0; i < robot->number_of_servos; i++) {
					current_pos_spin_boxes[i]->setValue(robot->current_pos[i]);
					robot->desired_pos[i] = robot->current_pos[i];
					std::cout << current_pos_spin_boxes[i]->value() << " ";
				}
				std::cout << std::endl;
			} else {
				ui.pushButton_execute->setDisabled(true); // Wygaszanie elementow przy niezsynchronizowanym robocie
			}
		}

	} // end try
	CATCH_SECTION_UI_PTR

}

void wgt_irp6_m_joints::move_it()
{
	// wychwytania ew. bledow ECP::robot
	try {

		if (robot->state.edp.pid != -1) {

			specyficrobot->ui_ecp_robot->move_joints(robot->desired_pos);

			if ((robot->state.edp.is_synchronised) /* TR && (is_open)*/) { // by Y o dziwo nie dziala poprawnie 	 if (robot->state.edp.is_synchronised)
				std::cout << "move it ok" << std::endl;
				for (int i = 0; i < robot->number_of_servos; i++) {
					desired_pos_spin_boxes[i]->setValue(robot->desired_pos[i]);
				}

				init();
			}
		}
	}

	CATCH_SECTION_UI_PTR
}

