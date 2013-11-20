#include "wgt_kinematic.h"
#include "ui_robot.h"
#include "interface.h"
#include "ui_r_common_012.h"
#include "allrobots.h"
#include "mp.h"
#include "ui_ecp_robot/ui_ecp_r_common_012.h"

wgt_kinematic::wgt_kinematic(QString _widget_label, mrrocpp::ui::common::Interface& _interface, mrrocpp::ui::common::UiRobot *robo, QWidget *parent) :
		wgt_base(QString::fromStdString(robo->getName()), _interface, parent), ui(new Ui::wgt_kinematicClass)
{
	robot = dynamic_cast <mrrocpp::ui::common_012::UiRobot *>(robo);
	ui->setupUi(this);
	dwgt->setWindowTitle(QString::fromStdString((robo->getName())) + " kinematic");
	connect(this, SIGNAL(window_init_signal()), this, SLOT(window_init_slot()), Qt::QueuedConnection);
}

wgt_kinematic::~wgt_kinematic()
{
	delete ui;
}

Ui::wgt_kinematicClass * wgt_kinematic::get_ui()
{
	return ui;
}

void wgt_kinematic::window_init()
{
	emit window_init_signal();
}

void wgt_kinematic::window_init_slot()
{
	init();
}

void wgt_kinematic::my_open(bool set_on_top)
{
	wgt_base::my_open(set_on_top);
	window_init();
}

void wgt_kinematic::on_pushButton_read_clicked()
{
	try {
		uint8_t kinematic_model_no;
		robot->ui_ecp_robot->get_kinematic(&kinematic_model_no);
		ui->spinBox_current_kinematics->setValue(kinematic_model_no);
	}
	CATCH_SECTION_UI_PTR

} void wgt_kinematic::on_pushButton_set_clicked()
{
	try {
		uint8_t kinematic_model_no;

		kinematic_model_no = ui->spinBox_desired_kinematics->value();
		robot->ui_ecp_robot->set_kinematic(kinematic_model_no);
		init();
	}
	CATCH_SECTION_UI_PTR

} void wgt_kinematic::init()
{
	try {
		uint8_t kinematic_model_no;
		robot->ui_ecp_robot->get_kinematic(&kinematic_model_no);
		ui->spinBox_current_kinematics->setValue(kinematic_model_no);
		ui->spinBox_desired_kinematics->setValue(kinematic_model_no);
	}
	CATCH_SECTION_UI_PTR

}
