#ifndef WGT_SERVO_ALGORITHM_H_
#define WGT_SERVO_ALGORITHM_H_

#include "wgt_base.h"
#include <QGridLayout>
#include <QPushButton>
#include <iostream>
#include "ui_wgt_servo_algorithm_template.h"

namespace mrrocpp {
namespace ui {
namespace common {
class Interface;
class AllRobots;

}
namespace common_012 {
class UiRobot;

}
}
}

class wgt_servo_algorithm : public wgt_base
{
Q_OBJECT

public:
	wgt_servo_algorithm(QString _widget_label, mrrocpp::ui::common::Interface& _interface, mrrocpp::ui::common::UiRobot *robo, QWidget *parent =
			0);
	~wgt_servo_algorithm();

	//void synchro_depended_init();
	//void init_and_copy();

	const static int desired_pos_column = 5;
	const static int inc_move_column = 7;

protected:

	QVector <QSpinBox*> current_servo_algorithm_boxes;
	QVector <QSpinBox*> current_servo_parameters_boxes;
	QVector <QSpinBox*> desired_servo_algorithm_boxes;
	QVector <QSpinBox*> desired_servo_parameters_boxes;
	QVector <QLabel*> axis_labels;
	QPushButton *copy_button;

	void setup_ui(QGridLayout *layout, int _rows_number);

private:
	Ui::wgt_servo_algorithm_templateClass *ui;
	mrrocpp::ui::common_012::UiRobot* robot;

	int copy();
	void init();

public slots:
	void copy_button_clicked();

	void init_and_copy_slot();

	void on_pushButton_read_clicked();
	void on_pushButton_set_clicked();

};

#endif /* WGT_SERVO_ALGORITHM_H_ */
