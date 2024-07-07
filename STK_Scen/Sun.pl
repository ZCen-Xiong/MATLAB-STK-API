stk.v.11.0
WrittenBy    STK_v11.2.0

BEGIN Planet

Name        Sun

BEGIN PathDescription

		CentralBody                    Sun
		UseCbEphemeris                 Yes

BEGIN               EphemerisData

	EphemerisSource             JplDE

	JplIndex                    10

	JplSpiceId                  10

	ApplyTDTtoTDBCorrectionForDE     Yes

      OrbitEpoch                  2451545.0000000
      OrbitMeanDist               149597886455.77
      OrbitEcc                    0.016710220000001
      OrbitInc                    23.439340817689
      OrbitRAAN                   359.99996261506
      OrbitPerLong                462.94718913083
      OrbitMeanLong               820.46434913083
      OrbitMeanDistDot            -0.20478832306639
      OrbitEccDot                 -1.0414784394251e-09
      OrbitIncDot                 -3.5013667765767e-07
      OrbitRAANDot                1.7494818152674e-07
      OrbitPerLongDot             9.1275248714007e-06
      OrbitMeanLongDot            0.98560911497639

END     EphemerisData

END PathDescription

	BEGIN PhysicalData

		GM                   1.327122000000e+20
		Radius               6.955080000000e+08
		Magnitude            0.000000000000e+00
		ReferenceDistance    0.000000000000e+00

	END PhysicalData

	BEGIN AutoRename

		AutoRename           Yes

	END AutoRename

BEGIN Extensions
    
    BEGIN ExternData
    END ExternData
    
    BEGIN ADFFileData
    END ADFFileData
    
    BEGIN AccessConstraints
		LineOfSight   IncludeIntervals 
    END AccessConstraints
    
    BEGIN Desc
    END Desc
    
    BEGIN Crdn
    END Crdn
    
    BEGIN Graphics

			BEGIN Attributes

				MarkerColor             #0000ff
				LabelColor              #0000ff
				LineColor               #0000ff
				LineStyle               0
				LineWidth               1.0
				MarkerStyle             2
				FontStyle               0

			END Attributes

			BEGIN Graphics

				Show                     On
				Inherit                  On
				ShowLabel                On
				ShowPlanetPoint          On
				ShowSubPlanetPoint       On
				ShowSubPlanetLabel       On
				ShowOrbit                Off
				NumOrbitPoints           360
				OrbitTime                0.000000000000e+00
				OrbitDisplay             OneOrbit
				TransformTrajectory      On

			END Graphics
    END Graphics
    
    BEGIN VO
    END VO

END Extensions

END Planet

