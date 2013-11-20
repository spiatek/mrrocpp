#ifndef WGT_kinematic_H
#define WGT_kinematic_H

#include <QtGui/QWidget>
#include <QVBoxLayout>
#include <QDockWidget>
#include "ui_wgt_kinematic.h"
#include "wgt_base.h"

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

class wgt_kinematic : public wgt_base
{
Q_OBJECT

public:
	wgt_kinematic(QString _widget_label, mrrocpp::ui::common::Interface& _interface, mrrocpp::ui::common::UiRobot *robo, QWidget *parent =
			0);
	~wgt_kinematic();

	void window_init();

	void my_open(bool set_on_top = false);

	Ui::wgt_kinematicClass * get_ui();

	virtual void add_button(QPushButton *button, int row, int space)
	{
	}
	virtual void setup_ui()
	{
	}

private:
	Ui::wgt_kinematicClass* ui;

	mrrocpp::ui::common_012::UiRobot* robot;

	// aktualizacja ustawien przyciskow
	void init();

signals:
	void window_init_signal();

public slots:
	void window_init_slot();

private slots:

	void on_pushButton_read_clicked();
	void on_pushButton_set_clicked();

};

#endif // WGT_PROCESS_CONTROL_H
