/*=========================================================================
 Authors: David Doria at Rensselaer Polytechnic Institute and
   Arnaud Gelas at Harvard Medical School

 Copyright (c) 2010, David Doria at Rensselaer Polytechnic Institute and
   Arnaud Gelas at Harvard Medical School,
   All rights reserved.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions are met:

 Redistributions of source code must retain the above copyright notice,
 this list of conditions and the following disclaimer.
 Redistributions in binary form must reproduce the above copyright notice,
 this list of conditions and the following disclaimer in the documentation
 and/or other materials provided with the distribution.
 Neither the name of the Rensselaer Polytechnic Institute and of Harvard
 Medical School nor the names of its contributors may be used to endorse
 or promote products derived from this software without specific prior
 written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS
 BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY,
 OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT
 OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
 ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
=========================================================================*/
#include <vtkSmartPointer.h>
#include <vtkXMLPolyDataReader.h>
#include <vtkXMLPolyDataWriter.h>

#include "vtkPoissonReconstruction.h"
#include <filesystem>
#include <iostream>

#include <vtkSmartPointer.h>

#include <vtkBYUReader.h>
#include <vtkOBJReader.h>
#include <vtkPLYReader.h>
#include <vtkPolyDataReader.h>
#include <vtkSTLReader.h>
#include <vtkXMLPolyDataReader.h>

#include <vtkPointSource.h>
#include <vtkPoissonReconstruction.h>
#include <vtkPCANormalEstimation.h>

#include <vtkPointData.h>
#include <vtkPolyDataMapper.h>
#include <vtkProperty.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkRenderer.h>
#include <vtkCamera.h>

#include <vtkNamedColors.h>
#include <vtksys/SystemTools.hxx>

//int main(int argc, char *argv[])
//{
//  //if ( argc < 4 )
//  //  {
//  //  cout << "PoissonReconstruction takes 3 arguments: " << endl;
//  //  cout << "1-Input file (*.vtp)" << endl;
//  //  cout << "2-Depth" << endl;
//  //  cout << "3-Output file (*.vtp)" << endl;
//  //  return EXIT_FAILURE;
//  //  }
//  std::string inputFileName = "D:/programming/C++/VTK/New folder/PoissonReconstruction-master/build/bin/Debug/horsePoints.vtp";  //"horsePoints.vtp";
//  std::string outputFileName = "D:/programming/C++/VTK/New folder/PoissonReconstruction-master/build/bin/Debug/horse.vtp"; //"horse.vtp";
//  int            depth = 10;
//
//  vtkSmartPointer< vtkXMLPolyDataReader > reader =
//    vtkSmartPointer< vtkXMLPolyDataReader >::New();
//  reader->SetFileName(inputFileName.c_str());
//  reader->Update();
//  if (!reader && !reader->GetOutputPort())
//      return -1;
//  vtkSmartPointer< vtkPoissonReconstruction > poissonFilter =
//    vtkSmartPointer< vtkPoissonReconstruction >::New();
//  poissonFilter->SetDepth(depth);
//  poissonFilter->SetInputConnection( reader->GetOutputPort() );
//  poissonFilter->Update();
//
//  vtkSmartPointer< vtkXMLPolyDataWriter > writer =
//    vtkSmartPointer< vtkXMLPolyDataWriter >::New();
//  writer->SetInputConnection( poissonFilter->GetOutputPort() );
//  writer->SetFileName(outputFileName.c_str());
//  writer->Update();
//
//  return EXIT_SUCCESS;
//}

namespace
{
    vtkSmartPointer<vtkPolyData> ReadPolyData(const char *fileName);
}

int main(int argc, char *argv[])
{
    vtkSmartPointer<vtkPolyData> polyData = ReadPolyData("D:/programming/C++/VTK/New folder/PoissonReconstruction-master/build/bin/Debug/sphere_points.vtp");
    std::cout << "# of points: " << polyData->GetNumberOfPoints() << std::endl;

    vtkSmartPointer<vtkPoissonReconstruction> surface =
        vtkSmartPointer<vtkPoissonReconstruction>::New();
    surface->SetDepth(12);

    int sampleSize = polyData->GetNumberOfPoints() * .00005;
    if (sampleSize < 10)
    {
        sampleSize = 10;
    }
    if (polyData->GetPointData()->GetNormals())
    {
        std::cout << "Using normals from input file" << std::endl;
        surface->SetInputData(polyData);
    }
    else
    {
        std::cout << "Estimating normals using PCANormalEstimation" << std::endl;
        vtkSmartPointer<vtkPCANormalEstimation> normals =
            vtkSmartPointer<vtkPCANormalEstimation>::New();
        normals->SetInputData(polyData);
        normals->SetSampleSize(sampleSize);
        normals->SetNormalOrientationToGraphTraversal();
        normals->FlipNormalsOff();
        surface->SetInputConnection(normals->GetOutputPort());
    }

    vtkSmartPointer<vtkPolyDataMapper> surfaceMapper =
        vtkSmartPointer<vtkPolyDataMapper>::New();
    surfaceMapper->SetInputConnection(surface->GetOutputPort());

    vtkSmartPointer<vtkNamedColors> colors =
        vtkSmartPointer<vtkNamedColors>::New();

    vtkSmartPointer<vtkProperty> back =
        vtkSmartPointer<vtkProperty>::New();
    back->SetColor(colors->GetColor3d("banana").GetData());

    vtkSmartPointer<vtkActor> surfaceActor =
        vtkSmartPointer<vtkActor>::New();
    surfaceActor->SetMapper(surfaceMapper);
    surfaceActor->GetProperty()->SetColor(colors->GetColor3d("Tomato").GetData());
    surfaceActor->SetBackfaceProperty(back);

    // Create graphics stuff
    //
    vtkSmartPointer<vtkRenderer> ren1 =
        vtkSmartPointer<vtkRenderer>::New();
    ren1->SetBackground(colors->GetColor3d("SlateGray").GetData());

    vtkSmartPointer<vtkRenderWindow> renWin =
        vtkSmartPointer<vtkRenderWindow>::New();
    renWin->AddRenderer(ren1);
    renWin->SetSize(512, 512);

    vtkSmartPointer<vtkRenderWindowInteractor> iren =
        vtkSmartPointer<vtkRenderWindowInteractor>::New();
    iren->SetRenderWindow(renWin);

    // Add the actors to the renderer, set the background and size
    //
    ren1->AddActor(surfaceActor);

    // Generate an interesting view
    //
    ren1->ResetCamera();
    ren1->GetActiveCamera()->Azimuth(120);
    ren1->GetActiveCamera()->Elevation(30);
    ren1->GetActiveCamera()->Dolly(1.0);
    ren1->ResetCameraClippingRange();

    renWin->Render();
    iren->Initialize();
    iren->Start();

    return EXIT_SUCCESS;
}

namespace
{
    vtkSmartPointer<vtkPolyData> ReadPolyData(const char *fileName)
    {
        vtkSmartPointer<vtkPolyData> polyData;
        std::string extension = vtksys::SystemTools::GetFilenameExtension(std::string(fileName));
        if (extension == ".ply")
        {
            vtkSmartPointer<vtkPLYReader> reader =
                vtkSmartPointer<vtkPLYReader>::New();
            reader->SetFileName(fileName);
            reader->Update();
            polyData = reader->GetOutput();
        }
        else if (extension == ".vtp")
        {
            vtkSmartPointer<vtkXMLPolyDataReader> reader =
                vtkSmartPointer<vtkXMLPolyDataReader>::New();
            reader->SetFileName(fileName);
            reader->Update();
            polyData = reader->GetOutput();
        }
        else if (extension == ".vtk")
        {
            vtkSmartPointer<vtkPolyDataReader> reader =
                vtkSmartPointer<vtkPolyDataReader>::New();
            reader->SetFileName(fileName);
            reader->Update();
            polyData = reader->GetOutput();
        }
        else if (extension == ".obj")
        {
            vtkSmartPointer<vtkOBJReader> reader =
                vtkSmartPointer<vtkOBJReader>::New();
            reader->SetFileName(fileName);
            reader->Update();
            polyData = reader->GetOutput();
        }
        else if (extension == ".stl")
        {
            vtkSmartPointer<vtkSTLReader> reader =
                vtkSmartPointer<vtkSTLReader>::New();
            reader->SetFileName(fileName);
            reader->Update();
            polyData = reader->GetOutput();
        }
        else if (extension == ".g")
        {
            vtkSmartPointer<vtkBYUReader> reader =
                vtkSmartPointer<vtkBYUReader>::New();
            reader->SetGeometryFileName(fileName);
            reader->Update();
            polyData = reader->GetOutput();
        }
        else
        {
            vtkSmartPointer<vtkPointSource> points =
                vtkSmartPointer<vtkPointSource>::New();
            points->SetNumberOfPoints(1000);
            points->SetRadius(1.0);
            points->SetCenter(vtkMath::Random(-1, 1),
                vtkMath::Random(-1, 1),
                vtkMath::Random(-1, 1));
            points->SetDistributionToShell();
            points->Update();
            polyData = points->GetOutput();
        }
        return polyData;
    }
}
