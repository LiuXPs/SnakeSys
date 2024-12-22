/*
========================================================================================================================

    ┌────────────────────────────────────────────────────────┐                                       .::!!!!!!!:.
    │  mmmm                #              mmmm               │      .!!!!!:.                        .:!!!!!!!!!!!!
    │ #"   " m mm    mmm   #   m   mmm   #"   " m   m   mmm  │      ~~~~!!!!!!.                 .:!!!!!!!!!UWWW$$$
    │ "#mmm  #"  #  "   #  # m"   #"  #  "#mmm  "m m"  #   " │          :$$NWX!!:           .:!!!!!!XUWW$$$$$$$$$P
    │     "# #   #  m"""#  #"#    #""""      "#  #m#    """m │          $$$$$##WX!:      .<!!!!UW$$$$"  $$$$$$$$#
    │ "mmm#" #   #  "mm"#  #  "m  "#mm"  "mmm#"  "#    "mmm" │          $$$$$  $$$UX   :!!UW$$$$$$$$$   4$$$$$*
    │                                            m"          │          ^$$$B  $$$$\     $$$$$$$$$$$$   d$$R"
    │                                           ""           │            "*$bd$$$$      '*$$$$$$$$$$$o+#"
    └────────────────────────────────────────────────────────┘                 """"          """""""

* SnakeSys: An Open-Source System for Snake Robots Research
* Copyright (C) 2020-2025 Xupeng Liu, Yong Zang and Zhiying Gao
*
* This program is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
* License as published by the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
* the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along with this program.
* If not, see <http://www.gnu.org/licenses/>.

========================================================================================================================
*/

#include "snake_gui/double_spinbox_and_slider.h"
#include "snake_gui/ui_double_spinbox_and_slider.h"

DoubleSpinBoxAndSlider::DoubleSpinBoxAndSlider(QWidget *parent) : QWidget(parent), ui(new Ui::DoubleSpinBoxAndSlider) {
    ui->setupUi(this);

    // Paper color.
//    setStyleSheet("background-color: rgb(255, 255, 255);color: rgb(0, 0, 0);");

    connect(ui->doubleSpinBox, &QDoubleSpinBox::valueChanged, ui->horizontalSlider, &QSlider::setValue);
    connect(ui->horizontalSlider, &QSlider::valueChanged, ui->doubleSpinBox, &QDoubleSpinBox::setValue);
}

DoubleSpinBoxAndSlider::~DoubleSpinBoxAndSlider() {
    delete ui;
}

void DoubleSpinBoxAndSlider::initParam(std::string name, double min, double max, double step) {
    ui->label->setText(name.c_str());
    ui->doubleSpinBox->setMinimum(min);
    ui->doubleSpinBox->setMaximum(max);
    ui->doubleSpinBox->setSingleStep(step);
}

double DoubleSpinBoxAndSlider::getVal() {
    return ui->doubleSpinBox->value();
}

void DoubleSpinBoxAndSlider::setVal(double val) {
    ui->doubleSpinBox->setValue(val);
}