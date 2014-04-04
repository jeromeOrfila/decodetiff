#pragma once


#ifndef __PCLUtils__
#define __PCLUtils__


#include <iostream>
#include <fstream>
#include <string>

#include "gdal.h"

#include <pcl/point_types.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>

class PCLContext
{
public:
	PCLContext(void);
	~PCLContext(void);

	//int Fill(pcl::PointCloud<pcl::PointXYZ>::Ptr pcl, GDALDatasetH _hDataset, int stride);

private:

	//int GDALProcessLine(GDALRasterBandH & hBand, pcl::PointCloud<pcl::PointXYZ>::Ptr pcl, double * adfGeoTransform, int yind, int xsize);

};

#endif
