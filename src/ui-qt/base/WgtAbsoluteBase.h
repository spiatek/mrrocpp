#ifndef WGTABSOLUTEBASE_H_
#define WGTABSOLUTEBASE_H_

#include "wgt_base.h"
#include <QGridLayout>
#include <QPushButton>
#include <iostream>
#include "ui_wgt_absolute_template.h"

class WgtAbsoluteBase : public wgt_base
{
Q_OBJECT

public:
	WgtAbsoluteBase(QString _widget_label, mrrocpp::ui::common::Interface& _interface, mrrocpp::ui::common::UiRobot *robo, QWidget *parent =
			0);
	~WgtAbsoluteBase();

	//void synchro_depended_init();
	//void init_and_copy();

	const static int desired_pos_column = 5;
	const static int inc_move_column = 7;

protected:
	Ui::wgt_absolute_template ui;
	QVector <QDoubleSpinBox*> current_pos_spin_boxes;
	QVector <QLabel*> axis_labels;
	//QDoubleSpinBox *step_spinbox;
	QPushButton *copy_button;
	/*
	 QPushButton *read_button;
	 QPushButton *execute_button;
	 QPushButton *import_button;
	 QPushButton *export_button;

	 */
	void create_buttons_and_spin_boxes();
	void synchro_depended_widgets_disable(bool set_disabled);
	virtual void setup_ui(QGridLayout *layout, int _rows_number);
	virtual void add_current_position_spin_box(QDoubleSpinBox *spin_box, int row);

private:
	void create_buttons();

	int copy();
	virtual void move_it()
	{
	}
	virtual void init()
	{
	}

public slots:
	virtual void inc_move_left_button_clicked(int button);
	virtual void inc_move_right_button_clicked(int button);

	void on_pushButton_read_clicked();
	void on_pushButton_export_clicked();
	void on_pushButton_import_clicked();
	void copy_button_clicked();
	void on_pushButton_execute_clicked();
	void init_and_copy_slot();
};

#endif /* WGTABSOLUTEBASE_H_ */
