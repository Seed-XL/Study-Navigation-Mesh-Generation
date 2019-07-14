using System;
using Utility.Logger;
using System.Collections.Generic;


namespace NMGen
{
    public class IntermediateData
    {
        public static long UNDEFINED = -1;
        public long voxelizationTime;
        public long regionGenTime;
        public long contourGenTime;
        public long polyGenTime;
        public long finalMeshGenTime;

        private SolidHeightfield mSolidHeightfield;
        private OpenHeightfield mOpenHeightfield;
        private ContourSet mContours;
        private PolyMeshField mPolyMesh;  


        public ContourSet contours()
        {
            return mContours; 
        }

        public long getTotalGenTime()
        {
            if ( finalMeshGenTime == UNDEFINED )
            {
                return UNDEFINED;
            }

            return voxelizationTime +
                regionGenTime +
                contourGenTime +
                polyGenTime +
                finalMeshGenTime; 
        }

        public OpenHeightfield openHeightfield()
        {
            return mOpenHeightfield; 
        }

        public SolidHeightfield solidHeightfield()
        {
            return mSolidHeightfield;  
        }

        public PolyMeshField polyMesh()
        {
            return mPolyMesh;  
        }

        public void reset()
        {
            voxelizationTime = UNDEFINED;
            regionGenTime = UNDEFINED;
            contourGenTime = UNDEFINED;
            polyGenTime = UNDEFINED;
            finalMeshGenTime = UNDEFINED;
            mSolidHeightfield = null;
            mOpenHeightfield = null;
            mContours = null;
            mPolyMesh = null; 
        }

        public void setContours( ContourSet contours )
        {
            mContours = contours;  
        }

        public void setSolidHeightfield(SolidHeightfield field)
        {
            mSolidHeightfield = field;  
        }

        public void setOpenHeightfield(OpenHeightfield field )
        {
            mOpenHeightfield = field; 
        }

        public void setPolyMesh( PolyMeshField mesh )
        {
            mPolyMesh = mesh; 
        }
    }
}
