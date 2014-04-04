#include "StdAfx.h"


#include "PCLContext.h"


PCLContext::PCLContext(void)
{
}


PCLContext::~PCLContext(void)
{
}

/*
void
PCLUtils::Fill(pcl::PointCloud<pcl::PointXYZ>::Ptr pcl, GDALDatasetH _hDataset, int stride)
{


}
*/

/************************************************************************/
/*                        Process Methods                               */
/************************************************************************/
/*
void
PCLUtils::GDALProcessLine(GDALRasterBandH & hBand, pcl::PointCloud<pcl::PointXYZ>::Ptr pcl, double * adfGeoTransform, int yind, int xsize) 
{
	float * pGeoD = new float[3];
	float * pGeoC = new float[3];

	GDALRasterIO(hBand, GF_Read, 0, yind, xsize , 1, linedata, xsize , 1, GDT_CInt16, 0, 0 );
            
	for( int xind=0; xind<xsize-1; xind+=stride) {

		pGeoD[0] = adfGeoTransform[0] + adfGeoTransform[1] * xind
			+ adfGeoTransform[2] * yind;
		pGeoD[1] = adfGeoTransform[3] + adfGeoTransform[4] * xind
			+ adfGeoTransform[5] * yind;
		pGeoD[2] =  (float) linedata[xind];

		GDALGetGeocCoords(pGeoD, pGeoC);

		amin = (amin < linedata[xind]) ? amin : linedata[xind];
		amax = (amax > linedata[xind]) ? amax : linedata[xind];

		std::cout << '\r';
		std::cout << "reading " << xind << ", " << yind << ": " << linedata[xind];

    }

	// Add last point
	pGeoD[0] = adfGeoTransform[0] + adfGeoTransform[1] * (xsize-1)
		+ adfGeoTransform[2] * yind;
	pGeoD[1] = adfGeoTransform[3] + adfGeoTransform[4] * (xsize-1)
		+ adfGeoTransform[5] * yind;
	pGeoD[2] =  (float) linedata[(xsize-1)];

	GDALGetGeocCoords(pGeoD, pGeoC);
	amin = (amin < linedata[(xsize-1)]) ? amin : linedata[(xsize-1)];
	amax = (amax > linedata[(xsize-1)]) ? amax : linedata[(xsize-1)];

	std::cout << '\r';
	std::cout << "reading " << (xsize-1) << ", " << yind << ": " << linedata[(xsize-1)];
}
*/
