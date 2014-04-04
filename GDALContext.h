#pragma once



#include <iostream>
#include <fstream>
#include <string>

#include "gdal.h"
#include "ogr_srs_api.h"

#include "gdal_alg.h"
#include "cpl_string.h"
#include "cpl_conv.h"
#include "cpl_multiproc.h"

#include <pcl/point_types.h>


#define RAD_TO_DEG				57.295779513082320876798154814105f /* (180.0f / 3.1415926535897932384626433832795f) */
#define DEG_TO_RAD				0.01745329251994329576923690768489f /* (3.1415926535897932384626433832795f / 180.0f) */
//WGS84
#define WGS84_ECLIPSE_E			0.081819191025f /* 1/298,257223563 */
#define WGS84_EARTH_RADIUS		6378137.00f 
#define WGS84_C1				0.993305619980014559449375f /* 1.0f - (ECLIPSE_E*ECLIPSE_E) */


class GDALContext
{
private:

	GDALDatasetH					_hDataset;
	GDALDriverH						_hDriver;
	double *						_adfGeoTransform;
	bool							_bVerbose;
    OGRCoordinateTransformationH	_hTransform;

public:

	GDALContext(void);
	~GDALContext(void);

	int OpenTiff(std::string filename);

	pcl::PointCloud<pcl::PointXYZ>::Ptr ToPCL(int band, int stride);

	int Release();

private:
	
	int GDALInfoReportCorner( GDALDatasetH hDataset, 
                      OGRCoordinateTransformationH hTransform,
                      const char * corner_name,
                      double x, double y );

	int ProcessLine( //GDALRasterBandH & hBand, 
					pcl::PointCloud<pcl::PointXYZ>::Ptr pcl, 
					int yind, 
					int xsize, 
					int stride, 
					int band);

	bool GetGeocCoords(const float* iGeodPos, float* oCoords);

};

