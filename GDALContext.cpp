#include "StdAfx.h"
#include "GDALContext.h"


GDALContext::GDALContext(void)
{
	
	_hDataset = NULL;
	_hDriver = NULL;
	_adfGeoTransform =  (double *) CPLMalloc(sizeof(double)*6);
	_bVerbose = TRUE;
    _hTransform = NULL;

}


GDALContext::~GDALContext(void)
{
}


int
GDALContext::OpenTiff(std::string filename)
{
	//http://www.gdal.org/gdalinfo_8c.html

	GDALRasterBandH		hBand= NULL;
	char              **papszFileList;
	char			  **papszMetadata = NULL;
	int                 bListMDD = FALSE;
    const char		   *pszProjection = NULL;
    OGRCoordinateTransformationH hTransform = NULL;
    int             bShowFileList = TRUE;
	bool bComputeMinMax = TRUE;
	
	/* -------------------------------------------------------------------- */
	/*      Open dataset.                                                   */
	/* -------------------------------------------------------------------- */
	_hDataset = GDALOpen( filename.c_str(), GA_ReadOnly );

    if( _hDataset == NULL )
    {
        fprintf( stderr,
                 "gdalinfo failed - unable to open '%s'.\n",
                 filename );
		return 0;
	}
    
	/* -------------------------------------------------------------------- */
	/*      Report general info.                                            */
	/* -------------------------------------------------------------------- */
	_hDriver = GDALGetDatasetDriver( _hDataset );
    printf( "Driver: %s/%s\n",
            GDALGetDriverShortName( _hDriver ),
            GDALGetDriverLongName( _hDriver ) );

    papszFileList = GDALGetFileList( _hDataset );
    if( CSLCount(papszFileList) == 0 )
    {
        printf( "Files: none associated\n" );
    }
    else
    {
        printf( "Files: %s\n", papszFileList[0] );
        if( _bVerbose )
        {
            for( int i = 1; papszFileList[i] != NULL; i++ )
                printf( "       %s\n", papszFileList[i] );
        }
    }
    CSLDestroy( papszFileList );

    printf( "Size is %d, %d\n",
            GDALGetRasterXSize( _hDataset ), 
            GDALGetRasterYSize( _hDataset ) );

	
	/* -------------------------------------------------------------------- */
	/*      Report projection.                                              */
	/* -------------------------------------------------------------------- */
    if( GDALGetProjectionRef( _hDataset ) != NULL )
    {
        OGRSpatialReferenceH  hSRS;
        char                  *pszProjection;

        pszProjection = (char *) GDALGetProjectionRef( _hDataset );

        hSRS = OSRNewSpatialReference(NULL);
        if( OSRImportFromWkt( hSRS, &pszProjection ) == CE_None )
        {
            char        *pszPrettyWkt = NULL;

            OSRExportToPrettyWkt( hSRS, &pszPrettyWkt, FALSE );
            printf( "Coordinate System is:\n%s\n", pszPrettyWkt );
            CPLFree( pszPrettyWkt );
        }
        else
            printf( "Coordinate System is `%s'\n",
                    GDALGetProjectionRef( _hDataset ) );

        if ( _bVerbose ) 
        {
            char *pszProj4 = NULL;
            OSRExportToProj4( hSRS, &pszProj4 );
            printf("PROJ.4 string is:\n\'%s\'\n",pszProj4);
            CPLFree( pszProj4 ); 
        }

        OSRDestroySpatialReference( hSRS );
	}

	/* -------------------------------------------------------------------- */
	/*      Report Geotransform.                                            */
	/* -------------------------------------------------------------------- */
    if( GDALGetGeoTransform( _hDataset, _adfGeoTransform ) == CE_None )
    {
        if( _adfGeoTransform[2] == 0.0 && _adfGeoTransform[4] == 0.0 )
        {
            printf( "Origin = (%.15f,%.15f)\n",
                    _adfGeoTransform[0], _adfGeoTransform[3] );

            printf( "Pixel Size = (%.15f,%.15f)\n",
                    _adfGeoTransform[1], _adfGeoTransform[5] );
        }
        else
            printf( "GeoTransform =\n"
                    "  %.16g, %.16g, %.16g\n"
                    "  %.16g, %.16g, %.16g\n", 
                    _adfGeoTransform[0],
                    _adfGeoTransform[1],
                    _adfGeoTransform[2],
                    _adfGeoTransform[3],
                    _adfGeoTransform[4],
                    _adfGeoTransform[5] );
    } else 
	{
		printf("Error: no _adfGeoTransform");
		return 0;
	}

	/* -------------------------------------------------------------------- */
	/*      Report GCPs.                                                    */
	/* -------------------------------------------------------------------- */
    if( _bVerbose && GDALGetGCPCount( _hDataset ) > 0 )
    {
        if (GDALGetGCPProjection(_hDataset) != NULL)
        {
            OGRSpatialReferenceH  hSRS;
            char                      *pszProjection;

            pszProjection = (char *) GDALGetGCPProjection( _hDataset );

            hSRS = OSRNewSpatialReference(NULL);
            if( OSRImportFromWkt( hSRS, &pszProjection ) == CE_None )
            {
                char    *pszPrettyWkt = NULL;

                OSRExportToPrettyWkt( hSRS, &pszPrettyWkt, FALSE );
                printf( "GCP Projection = \n%s\n", pszPrettyWkt );
                CPLFree( pszPrettyWkt );
            }
            else
                printf( "GCP Projection = %s\n",
                        GDALGetGCPProjection( _hDataset ) );

            OSRDestroySpatialReference( hSRS );
        }

        for( int i = 0; i < GDALGetGCPCount(_hDataset); i++ )
        {
            const GDAL_GCP      *psGCP;
            
            psGCP = GDALGetGCPs( _hDataset ) + i;

            printf( "GCP[%3d]: Id=%s, Info=%s\n"
                    "          (%.15g,%.15g) -> (%.15g,%.15g,%.15g)\n", 
                    i, psGCP->pszId, psGCP->pszInfo, 
                    psGCP->dfGCPPixel, psGCP->dfGCPLine, 
                    psGCP->dfGCPX, psGCP->dfGCPY, psGCP->dfGCPZ );
        }
    }

	/* -------------------------------------------------------------------- */
	/*      Report metadata.                                                */
	/* -------------------------------------------------------------------- */ 
    papszMetadata = (_bVerbose) ? GDALGetMetadata( _hDataset, NULL ) : NULL;
    if( papszMetadata && CSLCount(papszMetadata) > 0 )
    {
        printf( "Metadata:\n" );
        for( int i = 0; papszMetadata[i] != NULL; i++ )
        {
            printf( "  %s\n", papszMetadata[i] );
        }
    }

	/* -------------------------------------------------------------------- */
	/*      Report "IMAGE_STRUCTURE" metadata.                              */
	/* -------------------------------------------------------------------- */
    papszMetadata = (_bVerbose) ? GDALGetMetadata( _hDataset, "IMAGE_STRUCTURE" ) : NULL;
    if( papszMetadata && CSLCount(papszMetadata) > 0 )
    {
        printf( "Image Structure Metadata:\n" );
        for( int i = 0; papszMetadata[i] != NULL; i++ )
        {
            printf( "  %s\n", papszMetadata[i] );
        }
    }

	/* -------------------------------------------------------------------- */
	/*      Report subdatasets.                                             */
	/* -------------------------------------------------------------------- */
    papszMetadata = GDALGetMetadata( _hDataset, "SUBDATASETS" );
    if( papszMetadata && CSLCount(papszMetadata) > 0 )
    {
        printf( "Subdatasets:\n" );
        for( int i = 0; papszMetadata[i] != NULL; i++ )
        {
            printf( "  %s\n", papszMetadata[i] );
        }
    }

	/* -------------------------------------------------------------------- */
	/*      Report geolocation.                                             */
	/* -------------------------------------------------------------------- */
	papszMetadata = (_bVerbose) ? GDALGetMetadata( _hDataset, "GEOLOCATION" ) : NULL;
	if( papszMetadata && CSLCount(papszMetadata) > 0 )
	{
		printf( "Geolocation:\n" );
		for( int i = 0; papszMetadata[i] != NULL; i++ )
		{
			printf( "  %s\n", papszMetadata[i] );
		}
	}

	/* -------------------------------------------------------------------- */
	/*      Report RPCs                                                     */
	/* -------------------------------------------------------------------- */
    papszMetadata = (_bVerbose) ? GDALGetMetadata( _hDataset, "RPC" ) : NULL;
    if( papszMetadata && CSLCount(papszMetadata) > 0 )
    {
        printf( "RPC Metadata:\n" );
        for( int i = 0; papszMetadata[i] != NULL; i++ )
        {
            printf( "  %s\n", papszMetadata[i] );
        }
    }

	/* -------------------------------------------------------------------- */
	/*      Setup projected to lat/long transform if appropriate.           */
	/* -------------------------------------------------------------------- */
    if( GDALGetGeoTransform( _hDataset, _adfGeoTransform ) == CE_None )
        pszProjection = GDALGetProjectionRef(_hDataset);

    if( pszProjection != NULL && strlen(pszProjection) > 0 )
    {
        OGRSpatialReferenceH hProj, hLatLong = NULL;

        hProj = OSRNewSpatialReference( pszProjection );
        if( hProj != NULL )
            hLatLong = OSRCloneGeogCS( hProj );

        if( hLatLong != NULL )
        {
            CPLPushErrorHandler( CPLQuietErrorHandler );
            hTransform = OCTNewCoordinateTransformation( hProj, hLatLong );
            CPLPopErrorHandler();
            
            OSRDestroySpatialReference( hLatLong );
        }

        if( hProj != NULL )
            OSRDestroySpatialReference( hProj );
    }

	/* -------------------------------------------------------------------- */
	/*      Report corners.                                                 */
	/* -------------------------------------------------------------------- */
    printf( "Corner Coordinates:\n" );
    GDALInfoReportCorner( _hDataset, hTransform, "Upper Left", 
                          0.0, 0.0 );
    GDALInfoReportCorner( _hDataset, hTransform, "Lower Left", 
                          0.0, GDALGetRasterYSize(_hDataset));
    GDALInfoReportCorner( _hDataset, hTransform, "Upper Right", 
                          GDALGetRasterXSize(_hDataset), 0.0 );
    GDALInfoReportCorner( _hDataset, hTransform, "Lower Right", 
                          GDALGetRasterXSize(_hDataset), 
                          GDALGetRasterYSize(_hDataset) );
    GDALInfoReportCorner( _hDataset, hTransform, "Center", 
                          GDALGetRasterXSize(_hDataset)/2.0, 
                          GDALGetRasterYSize(_hDataset)/2.0 );

    if( hTransform != NULL )
    {
        OCTDestroyCoordinateTransformation( hTransform );
        hTransform = NULL;
    }
	
	/* ==================================================================== */
	/*      Loop over bands.                                                */
	/* ==================================================================== */
	
	int rasterCount = GDALGetRasterCount( _hDataset );
	std::cout << "Raster count : " << rasterCount << std::endl;
    for( int iBand = 0; iBand < rasterCount; iBand++ )
    {
        double      dfMin, dfMax, adfCMinMax[2], dfNoData;
        int         bGotMin, bGotMax, bGotNodata, bSuccess;
        int         nBlockXSize, nBlockYSize, nMaskFlags;
        double      dfMean, dfStdDev;
        GDALColorTableH hTable;
        CPLErr      eErr;
		
		hBand = GDALGetRasterBand( _hDataset, iBand+1 );
		
		GDALGetBlockSize( hBand, &nBlockXSize, &nBlockYSize );
        printf( "Band %d Block=%dx%d Type=%s, ColorInterp=%s\n", iBand+1,
                nBlockXSize, nBlockYSize,
                GDALGetDataTypeName(
                    GDALGetRasterDataType(hBand)),
                GDALGetColorInterpretationName(
                    GDALGetRasterColorInterpretation(hBand)) );

        if( GDALGetDescription( hBand ) != NULL 
            && strlen(GDALGetDescription( hBand )) > 0 )
            printf( "  Description = %s\n", GDALGetDescription(hBand) );

		dfMin = GDALGetRasterMinimum( hBand, &bGotMin );
        dfMax = GDALGetRasterMaximum( hBand, &bGotMax );
        if( bGotMin || bGotMax || bComputeMinMax )
        {
			printf( "  " );
            if( bGotMin )
                printf( "Min=%.3f ", dfMin );
            if( bGotMax )
                printf( "Max=%.3f ", dfMax );
        
            if( bComputeMinMax )
            {
                CPLErrorReset();
                GDALComputeRasterMinMax( hBand, FALSE, adfCMinMax );
                if (CPLGetLastErrorType() == CE_None)
                {
                  printf( "  Computed Min/Max=%.3f,%.3f", 
                          adfCMinMax[0], adfCMinMax[1] );
                }
            }

            printf( "\n" );
        }

		dfNoData = GDALGetRasterNoDataValue( hBand, &bGotNodata );
        if( bGotNodata )
        {
            if (CPLIsNan(dfNoData))
                printf( "  NoData Value=nan\n" );
            else
                printf( "  NoData Value=%.18g\n", dfNoData );
        }

		if( GDALGetOverviewCount(hBand) > 0 )
        {
            int         iOverview;

            printf( "  Overviews: " );
            for( iOverview = 0; 
                 iOverview < GDALGetOverviewCount(hBand);
                 iOverview++ )
            {
                GDALRasterBandH hOverview;
                const char *pszResampling = NULL;

                if( iOverview != 0 )
                    printf( ", " );

                hOverview = GDALGetOverview( hBand, iOverview );
                if (hOverview != NULL)
                {
                    printf( "%dx%d", 
                            GDALGetRasterBandXSize( hOverview ),
                            GDALGetRasterBandYSize( hOverview ) );

                    pszResampling = 
                        GDALGetMetadataItem( hOverview, "RESAMPLING", "" );

                    if( pszResampling != NULL 
                        && EQUALN(pszResampling,"AVERAGE_BIT2",12) )
                        printf( "*" );
                }
                else
                    printf( "(null)" );
            }
            printf( "\n" );
        }

		if( GDALHasArbitraryOverviews( hBand ) )
        {
            printf( "  Overviews: arbitrary\n" );
        }

		nMaskFlags = GDALGetMaskFlags( hBand );
        if( (nMaskFlags & (GMF_NODATA|GMF_ALL_VALID)) == 0 )
        {
            GDALRasterBandH hMaskBand = GDALGetMaskBand(hBand) ;

            printf( "  Mask Flags: " );
            if( nMaskFlags & GMF_PER_DATASET )
                printf( "PER_DATASET " );
            if( nMaskFlags & GMF_ALPHA )
                printf( "ALPHA " );
            if( nMaskFlags & GMF_NODATA )
                printf( "NODATA " );
            if( nMaskFlags & GMF_ALL_VALID )
                printf( "ALL_VALID " );
            printf( "\n" );

            if( hMaskBand != NULL &&
                GDALGetOverviewCount(hMaskBand) > 0 )
            {
                int             iOverview;

                printf( "  Overviews of mask band: " );
                for( iOverview = 0; 
                     iOverview < GDALGetOverviewCount(hMaskBand);
                     iOverview++ )
                {
                    GDALRasterBandH     hOverview;

                    if( iOverview != 0 )
                        printf( ", " );

                    hOverview = GDALGetOverview( hMaskBand, iOverview );
                    printf( "%dx%d", 
                            GDALGetRasterBandXSize( hOverview ),
                            GDALGetRasterBandYSize( hOverview ) );
                }
                printf( "\n" );
            }
        }

		
        if( strlen(GDALGetRasterUnitType(hBand)) > 0 )
        {
            printf( "  Unit Type: %s\n", GDALGetRasterUnitType(hBand) );
        }

        if( GDALGetRasterCategoryNames(hBand) != NULL )
        {
            char **papszCategories = GDALGetRasterCategoryNames(hBand);
            int i;

            printf( "  Categories:\n" );
            for( i = 0; papszCategories[i] != NULL; i++ )
                printf( "    %3d: %s\n", i, papszCategories[i] );
        }

		
        if( GDALGetRasterScale( hBand, &bSuccess ) != 1.0 
            || GDALGetRasterOffset( hBand, &bSuccess ) != 0.0 )
            printf( "  Offset: %.15g,   Scale:%.15g\n",
                    GDALGetRasterOffset( hBand, &bSuccess ),
                    GDALGetRasterScale( hBand, &bSuccess ) );


		
        papszMetadata = (_bVerbose) ? GDALGetMetadata( hBand, NULL ) : NULL;
        if( _bVerbose && CSLCount(papszMetadata) > 0 )
        {
            printf( "  Metadata:\n" );
            for( int i = 0; papszMetadata[i] != NULL; i++ )
            {
                printf( "    %s\n", papszMetadata[i] );
            }
        }

		papszMetadata = (_bVerbose) ? GDALGetMetadata( hBand, "IMAGE_STRUCTURE" ) : NULL;
        if( _bVerbose && CSLCount(papszMetadata) > 0 )
        {
            printf( "  Image Structure Metadata:\n" );
            for( int i = 0; papszMetadata[i] != NULL; i++ )
            {
                printf( "    %s\n", papszMetadata[i] );
            }
        }

		 if( GDALGetRasterColorInterpretation(hBand) == GCI_PaletteIndex 
            && (hTable = GDALGetRasterColorTable( hBand )) != NULL )
        {
            printf( "  Color Table (%s with %d entries)\n", 
                    GDALGetPaletteInterpretationName(
                        GDALGetPaletteInterpretation( hTable )), 
                    GDALGetColorEntryCount( hTable ) );

            if (_bVerbose)
            {
                for( int i = 0; i < GDALGetColorEntryCount( hTable ); i++ )
                {
                    GDALColorEntry      sEntry;
    
                    GDALGetColorEntryAsRGB( hTable, i, &sEntry );
                    printf( "  %3d: %d,%d,%d,%d\n", 
                            i, 
                            sEntry.c1,
                            sEntry.c2,
                            sEntry.c3,
                            sEntry.c4 );
                }
            }
		 }
		 
        if( _bVerbose && GDALGetDefaultRAT( hBand ) != NULL )
        {
            GDALRasterAttributeTableH hRAT = GDALGetDefaultRAT( hBand );
            
            GDALRATDumpReadable( hRAT, NULL );
		}
    }
	
	return 1;
}

int 
GDALContext::Release()
{
	GDALClose( _hDataset );
    GDALDumpOpenDatasets( stderr );
	GDALDestroyDriverManager();
	CPLDumpSharedList( NULL );
    CPLCleanupTLS();
	return 1;
}



/************************************************************************/
/*                        GDALInfoReportCorner()                        */
/************************************************************************/

int 
GDALContext::GDALInfoReportCorner( GDALDatasetH _hDataset, 
                      OGRCoordinateTransformationH hTransform,
                      const char * corner_name,
                      double x, double y )
{
    double      dfGeoX, dfGeoY;
    double      _adfGeoTransform[6];
        
    printf( "%-11s ", corner_name );
    
	/* -------------------------------------------------------------------- */
	/*      Transform the point into georeferenced coordinates.             */
	/* -------------------------------------------------------------------- */
    if( GDALGetGeoTransform( _hDataset, _adfGeoTransform ) == CE_None )
    {
        dfGeoX = _adfGeoTransform[0] + _adfGeoTransform[1] * x
            + _adfGeoTransform[2] * y;
        dfGeoY = _adfGeoTransform[3] + _adfGeoTransform[4] * x
            + _adfGeoTransform[5] * y;
    }
	else
    {
        printf( " no geo transform coordinates (%7.1f,%7.1f) \n", x, y );
        return FALSE;
    }

	/* -------------------------------------------------------------------- */
	/*      Report the georeferenced coordinates.                           */
	/* -------------------------------------------------------------------- */
    if( ABS(dfGeoX) < 181 && ABS(dfGeoY) < 91 )
    {
        printf( "georeferenced coordinates: (%12.7f,%12.7f) ", dfGeoX, dfGeoY );
    }
    else
    {
        printf( "georeferenced coordinates: (%12.3f,%12.3f) ", dfGeoX, dfGeoY );
    }

	/* -------------------------------------------------------------------- */
	/*      Transform to latlong and report.                                */
	/* -------------------------------------------------------------------- */
    if( hTransform != NULL 
        && OCTTransform(hTransform,1,&dfGeoX,&dfGeoY,NULL) )
    {
        
        printf( "  oct transorm: (%s,", GDALDecToDMS( dfGeoX, "Long", 2 ) );
        printf( "%s) ", GDALDecToDMS( dfGeoY, "Lat", 2 ) );
    }

    printf( "\n" );

    return TRUE;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr 
GDALContext::ToPCL(int band, int stride) {

	int xsize = GDALGetRasterXSize(_hDataset);
	int ysize = GDALGetRasterYSize(_hDataset);

	pcl::PointCloud<pcl::PointXYZ>::Ptr pPCLCloud (new pcl::PointCloud<pcl::PointXYZ>);
	pPCLCloud->width = 2 + ((xsize-1) /stride);
	pPCLCloud->height = 2 + ((ysize-1) /stride);

	int lineCount = 0;
	for( int yind=0; yind<ysize-1; yind+=stride) {
		ProcessLine(pPCLCloud, yind, xsize, stride, band);
		lineCount++;
	}

	// Last line
	ProcessLine(pPCLCloud, ysize-1, xsize, stride, band);
	lineCount++;

	pPCLCloud->width = 2 + ((xsize-1) /stride);
	pPCLCloud->height = 2 + ((ysize-1) /stride);

	std::cout << "Tiff file fully processed, " << lineCount << " lines. " << std::endl;
	
	return pPCLCloud;

}

int
GDALContext::ProcessLine(pcl::PointCloud<pcl::PointXYZ>::Ptr pcl, int yind, int xsize, int stride, int band) 
{
	float * pGeoD = new float[3];
	float * pGeoC = new float[3];
	int amin = 10000;
	int amax = 0;

	float * linedata = (float *) CPLMalloc(sizeof(float)*xsize);
	
	GDALRasterBandH hBand = GDALGetRasterBand( _hDataset, 1);

	int nbpoints = 0;
	if ( CE_None == GDALRasterIO(hBand, GF_Read, 0, yind, xsize, 1, linedata, xsize , 1, GDT_Float32, 0, 0 ) ) {
      
		for( int xind=0; xind<xsize-1; xind+=stride) {

			pGeoD[0] = _adfGeoTransform[0] + _adfGeoTransform[1] * xind
				+ _adfGeoTransform[2] * yind;
			pGeoD[1] = _adfGeoTransform[3] + _adfGeoTransform[4] * xind
				+ _adfGeoTransform[5] * yind;
			pGeoD[2] =  (float) linedata[xind];

			GetGeocCoords(pGeoD, pGeoC);

			pcl->push_back(pcl::PointXYZ(pGeoC[0], pGeoC[1], pGeoC[2]));
			nbpoints++;

			amin = (amin < linedata[xind]) ? amin : linedata[xind];
			amax = (amax > linedata[xind]) ? amax : linedata[xind];
		}

		// Add last point
		pGeoD[0] = _adfGeoTransform[0] + _adfGeoTransform[1] * (xsize-1)
			+ _adfGeoTransform[2] * yind;
		pGeoD[1] = _adfGeoTransform[3] + _adfGeoTransform[4] * (xsize-1)
			+ _adfGeoTransform[5] * yind;
		pGeoD[2] =  (float) linedata[(xsize-1)];

		GetGeocCoords(pGeoD, pGeoC);
		
		pcl->push_back(pcl::PointXYZ(pGeoC[0], pGeoC[1], pGeoC[2]));
		nbpoints++;

		amin = (amin < linedata[(xsize-1)]) ? amin : linedata[(xsize-1)];
		amax = (amax > linedata[(xsize-1)]) ? amax : linedata[(xsize-1)];


		CPLFree(linedata);

		std::cout << "Line " << yind << " processed, " << nbpoints << " points added. Alt min: " << amin << ", alt max: " << amax << std::endl;

	} else
	{ 
		std::cout << "Err: Cannot read line " << yind  << std::endl;
	}

	return 0;
}

bool 
GDALContext::GetGeocCoords(const float* iGeodPos, float* oCoords)
{

	// http://www.forumsig.org/archive/index.php/t-9120.html
	// http://geodesie.ign.fr/index.php?page=calculs_sur_un_ellipsoide

	float lat = iGeodPos[0] * DEG_TO_RAD;
	float lon = iGeodPos[1] * DEG_TO_RAD;
	float alt = iGeodPos[2]; /* metres */

	float sinlat = sin(lat);
	float temp1 = WGS84_EARTH_RADIUS / sqrt(1.0f - (WGS84_ECLIPSE_E * (sinlat * sinlat)));
	float temp2 = temp1 * WGS84_C1;
	temp1 += alt;
	temp2 += alt;                    /* equ. A-10a */

	/* projection de l'axe d'horizontal sur le plan d'equateur */
	float w = temp1 * cos(lat);

	/* projection de l'axe vertical sur l'axe polaire */
	oCoords[0] = (w * cos(lon));              /* equ. A-11 */
	oCoords[1] = (w * sin(lon));
	oCoords[2] = (temp2 * sinlat);             /* equ. A-10b */

	return true;
}
