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

#ifndef DIALOG_TYPICAL_MODE_H
#define DIALOG_TYPICAL_MODE_H

#include "snake_gui/snake_gui_node.h"

#include <QAbstractButton>
#include <QDialog>
#include <QDoubleSpinBox>
#include <QPushButton>

#include <iostream>
#include <utility>

#include <rclcpp/rclcpp.hpp>
#include <snake_params/snake_params_heart.h>
#include <snake_params/snake_params_cpg_hopf.h>

namespace Ui {
    class DialogTypicalMode;
}

class DialogTypicalMode : public QDialog {
Q_OBJECT

public:
    explicit DialogTypicalMode(QWidget *parent = nullptr);

    ~DialogTypicalMode() override;

private:
    std::shared_ptr<Ui::DialogTypicalMode> ui;

public:
    void setClient(std::shared_ptr<snake::SnakeGUINode> node,
                   rclcpp::AsyncParametersClient::SharedPtr client_heart,
                   rclcpp::AsyncParametersClient::SharedPtr client_cpg_hopf);

    void getParam();

    void setParam();

    void getBoxValue();

    void setBoxValue();

private:
    std::shared_ptr<snake::SnakeGUINode> node_;

    std::shared_ptr<snake::SnakeParamsHeart> params_heart_;
    std::shared_ptr<snake::SnakeParamsCPGHopf> params_cpg_hopf_;

    rclcpp::AsyncParametersClient::SharedPtr client_heart_;
    rclcpp::AsyncParametersClient::SharedPtr client_cpg_hopf_;
};

#endif //DIALOG_TYPICAL_MODE_H
