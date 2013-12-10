/*
 * wgt_servo_algorithm.cpp
 *
 *  Created on: 14-07-2011
 *      Author: matt
 */

#include "wgt_servo_algorithm.h"
#include "mainwindow.h"
#include "ui_robot.h"
#include "interface.h"
#include "ui_r_common_012.h"
#include "allrobots.h"
#include "mp.h"
#include "ui_ecp_robot/ui_ecp_r_common_012.h"

wgt_servo_algorithm::wgt_servo_algorithm(QString _widget_label, mrrocpp::ui::common::Interface& _interface, mrrocpp::ui::common::UiRobot *robo, QWidget *parent) :
		wgt_base(_widget_label, _interface, robo, parent), ui(new Ui::wgt_servo_algorithm_templateClass)
{
	robot = dynamic_cast <mrrocpp::ui::common_012::UiRobot *>(robo);
	ui->setupUi(this);
	dwgt->setWindowTitle(QString::fromStdString((robo->getName())) + " sa");

	setup_ui(ui->grid_up, robot->number_of_servos);

	for (int i = 0; i < robot->number_of_servos; ++i) {
		axis_labels[i]->setText(QString("q%1").arg(i));
	}

}

wgt_servo_algorithm::~wgt_servo_algorithm()
{
	// TODO Auto-generated destructor stub
}

void wgt_servo_algorithm::setup_ui(QGridLayout *layout, int _rows_number)
{

	wgt_base::setup_ui(layout, _rows_number);

	for (int i = 0; i < rows_number; i++) {
		gridLayout->addWidget(create_label_to_vector(axis_labels), i + 1, 0, 1, 1);
		gridLayout->addWidget(create_spin_box_to_vector(current_servo_algorithm_boxes), i + 1, 1, 1, 1);
		gridLayout->addWidget(create_spin_box_to_vector(current_servo_parameters_boxes), i + 1, 2, 1, 1);

		gridLayout->addWidget(create_spin_box_to_vector(desired_servo_algorithm_boxes), i + 1, 6, 1, 1);
		gridLayout->addWidget(create_spin_box_to_vector(desired_servo_parameters_boxes), i + 1, 7, 1, 1);
	}
	copy_button = add_button(">", 1, 4, rows_number, 1);
	connect(copy_button, SIGNAL(clicked()), this, SLOT(copy_button_clicked()), Qt::QueuedConnection);

}

void wgt_servo_algorithm::on_pushButton_read_clicked()
{
	uint8_t servo_alg_no[lib::MAX_SERVOS_NR], servo_par_no[lib::MAX_SERVOS_NR];
	try {
		robot->ui_ecp_robot->get_servo_algorithm(servo_alg_no, servo_par_no);

		for (int i = 0; i < rows_number; i++) {
			current_servo_algorithm_boxes[i]->setValue(servo_alg_no[i]);
			current_servo_parameters_boxes[i]->setValue(servo_par_no[i]);
		}
	}
	CATCH_SECTION_UI_PTR
	//cos 	tam
}void wgt_servo_algorithm::on_pushButton_set_clicked()
{
	uint8_t servo_alg_no[lib::MAX_SERVOS_NR], servo_par_no[lib::MAX_SERVOS_NR];
	try {
		for (int i = 0; i < rows_number; i++) {
			servo_alg_no[i] = desired_servo_algorithm_boxes[i]->value();
			servo_par_no[i] = desired_servo_parameters_boxes[i]->value();
		}
		robot->ui_ecp_robot->set_servo_algorithm(servo_alg_no, servo_par_no);
		init();
		copy();
	}
	CATCH_SECTION_UI_PTR
	//cos 	tam
}void wgt_servo_algorithm::init()
{
	uint8_t servo_alg_no[lib::MAX_SERVOS_NR], servo_par_no[lib::MAX_SERVOS_NR];

	try {
		robot->ui_ecp_robot->get_servo_algorithm(servo_alg_no, servo_par_no);

		for (int i = 0; i < rows_number; i++) {
			current_servo_algorithm_boxes[i]->setValue(servo_alg_no[i]);
			current_servo_parameters_boxes[i]->setValue(servo_par_no[i]);
		}

	}
	CATCH_SECTION_UI_PTR
	//cos 	tam
}void wgt_servo_algorithm::init_and_copy_slot()
{
	init();
	copy();
}

void wgt_servo_algorithm::copy_button_clicked()
{
	copy();
}

int wgt_servo_algorithm::copy()
{

	if (robot->state.edp.pid != -1) {

		for (int i = 0; i < rows_number; i++) {
			desired_servo_algorithm_boxes[i]->setValue(current_servo_algorithm_boxes[i]->value());
			desired_servo_parameters_boxes[i]->setValue(current_servo_parameters_boxes[i]->value());
		}
	}

	return 1;
}

