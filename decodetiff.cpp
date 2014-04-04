// decodetiff.cpp : définit le point d'entrée pour l'application console.
//

#include "stdafx.h"

#include <iostream>
#include <fstream>
#include <string>

#include "GDALContext.h"
#include "PCLContext.h"

#include <boost/filesystem/path.hpp>
#include <boost/filesystem.hpp>

#include <pcl/point_types.h>

//#include <pcl/io/io.h>
//#include <pcl/io/pcd_io.h>
//#include <pcl/io/ply_io.h>
//#include <pcl/io/obj_io.h>
//#include <pcl/io/vtk_io.h>
//#include <pcl/io/vtk_lib_io.h>
//#include <pcl/kdtree/kdtree_flann.h>
//#include <pcl/features/normal_3d.h>
//#include <pcl/surface/gp3.h>
//
//#include "gdal_priv.h"
#include "gdal.h"
//#include "gdal_alg.h"
//#include "ogr_srs_api.h"
//#include "cpl_string.h"
//#include "cpl_conv.h"
//#include "cpl_multiproc.h"


static void 
printUsage(std::string message);


// GLobal variables	
std::string inputFile = "";
std::string outputFile = "";

int openTiff(std::string filename, int band);

int main(int argc, char* argv[])
{

	bool bVerbose = TRUE;
	GDALAllRegister();

    argc = GDALGeneralCmdLineProcessor( argc, &argv, 0 );
    if( argc < 1 )
        exit( -argc );

	/* -------------------------------------------------------------------- */
	/*      Parse arguments.                                                */
	/* -------------------------------------------------------------------- */
	int i = 0;
	for( i = 1; i < argc; i++ )
	{
        if( EQUAL(argv[i], "--version") )
        {
			std::cout << " -GDAL version: " << GDAL_RELEASE_NAME << " (" << 
				GDALVersionInfo("RELEASE_NAME") << ")." <<std::endl;
            std::cout << " -PCL version: " <<  PCL_MAJOR_VERSION << "." << PCL_MINOR_VERSION << std::endl;
            return 0;
        }
        else if( EQUAL(argv[i],"--help") )
		{
            printUsage("Help");
		}
		else if( EQUAL(argv[i], "-verbose") )
        {
            bVerbose = TRUE;
        }
        else if( argv[i][0] == '-' )
		{
			std::stringstream temp;
			std::string buffer;
			temp << "Unknown option name '" << argv[i] << "'";
			temp >> buffer;
            printUsage(buffer);
		}
        else if( inputFile == "" )
		{
            inputFile = argv[i];
		}
		else if( outputFile == "" )
		{
			outputFile = argv[i];
		}
		else
		{
            printUsage("Too many command options.");
		}
    }

	boost::filesystem::path filePath (inputFile);
	if ( ! boost::filesystem::exists(filePath) || !is_regular_file(filePath) ) 
	{
		std::cout << "Invalid file: " << filePath << '\n';
		return 0;
	}

	GDALContext ctx;
	ctx.OpenTiff(inputFile);
	pcl::PointCloud<pcl::PointXYZ>::Ptr pPCLCloud = ctx.ToPCL(1, 64);
	ctx.Release();

	if ( NULL!=pPCLCloud )
	{
		std::cout << " PCL size: " << pPCLCloud->width << " x " << pPCLCloud->height << std::endl;
		PCLContext pclContext;

	}


	/*
	GDALDatasetH _hDataset = lodaer.openTiff(inputFile);
	if ( NULL!=_hDataset ) 
	{

		pcl::PointCloud<pcl::PointXYZ>::Ptr pPCLCloud (new pcl::PointCloud<pcl::PointXYZ>);
		float			   *pGeoC, *pGeoD;

		pPCLCloud->width = GDALGetRasterXSize(_hDataset) /stride;
		pPCLCloud->height = GDALGetRasterYSize( _hDataset )/stride;
		pGeoC = new float[3];
		pGeoD = new float[3];
	}*/

	return 0;
}




static 
void printUsage(std::string message)
{
	std::cout <<
		"" << message << std::endl << 
		"Usage: decodetiff \n " <<
		"         inputfilename  \n" <<
		"	      outputFormat \n" << 
		"\n" << 
		"  filename: Name of the GeoTIFF file to report on.\n";

	exit(1);
}

