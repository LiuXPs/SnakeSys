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

#ifndef STRUCT_FRAME_H
#define STRUCT_FRAME_H

#include <iostream>
#include <vector>

#include <QThread>
#include <QMutex>

#include <vtkActor.h>
#include <vtkAxesActor.h>
#include <vtkSmartPointer.h>
#include <vtkTransform.h>
#include <vtkMatrix4x4.h>
#include <vtkCubeAxesActor.h>
#include <vtkOpenGLRenderer.h>
#include <vtkOpenGLRenderWindow.h>
#include <vtkGenericOpenGLRenderWindow.h>
#include <vtkGenericRenderWindowInteractor.h>
#include <vtkInteractorStyleTrackballCamera.h>

namespace snake {
    struct VTKFrame {
        bool vtk_flag;
        QMutex vtk_mutex;

        std::vector<vtkSmartPointer<vtkTransform>> vtk_tfs;
        std::vector<vtkSmartPointer<vtkMatrix4x4>> vtk_tfs_mat;
        std::vector<vtkSmartPointer<vtkAxesActor>> vtk_axes_actors;
        vtkSmartPointer<vtkAxesActor> vtk_axes_world;
        vtkSmartPointer<vtkCubeAxesActor> vtk_cube_actor;

        vtkSmartPointer<vtkOpenGLRenderer> vtk_render;
        vtkSmartPointer<vtkGenericOpenGLRenderWindow> vtk_render_win;
        vtkSmartPointer<vtkGenericRenderWindowInteractor> vtk_iren;
        vtkSmartPointer<vtkInteractorStyleTrackballCamera> vtk_style;
    };
}

#endif //STRUCT_FRAME_H
