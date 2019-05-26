using System;
using Utility.Logger;
using System.Collections.Generic;
using Utility.Logger; 


namespace NMGen
{
    public class DetailMeshBuilder
    {


#region HeightPatch 

        private class HeightPatch
        {
            public static int UNSET = int.MaxValue; 
            
            private int minWidthIndex ;
            private int minDepthIndex;

            private int width;
            private int depth;

            private int[] data; 

            public int getData(int globalWidthIndex, int globalDepthIndex)
            {
                int idx = Math.Min(Math.Max(globalWidthIndex - minWidthIndex, 0), width - 1) * depth
                    + Math.Min(Math.Max(globalDepthIndex - minDepthIndex, 0), depth - 1);

                return data[idx]; 
            }

            public bool isInPatch(int globalWidthIndex,int globalDepthIndex)
            {
                return (globalWidthIndex >= minWidthIndex
                    && globalDepthIndex >= minDepthIndex
                    && globalWidthIndex < minWidthIndex + width 
                    && globalDepthIndex < minDepthIndex + depth ); 
            }
            
            public void resetData()
            {
                if( null == data )
                {
                    return; 
                }

                for(int i = 0; i < data.Length; ++i)
                {
                    data[i] = UNSET; 
                }
            }


            public void setData(int globalWidthInex,int globalDepthIndex,int value )
            {
                data[(globalWidthInex - minWidthIndex) * depth + globalDepthIndex - minDepthIndex] = value;  
            }
        } // HeightPatch 

#endregion

        private static int UNDEFINED = -1;
        private static int HULL = -2;
        private static int MAX_VERTS = 256;
        private static int MAX_EDGES = 64;

        private float mContourSampleDistance;
        private float mContourMaxDeviation;

        public DetailMeshBuilder(float contourSampleDistance ,float contourMaxDeviation )
        {
            mContourSampleDistance = Math.Max(0, contourSampleDistance);
            mContourMaxDeviation = Math.Max(0, contourMaxDeviation);
        }

        public TriangleMesh build(PolyMeshField sourceMesh ,OpenHeightfield heightField)
        {
            if( null == sourceMesh 
                || 0 == sourceMesh.vertCount()
                || 0 == sourceMesh.polyCount() )
            {
                Logger.LogError("[DetailMeshBuilder][build]Param Error"); 
                return null; 
            }

            TriangleMesh mesh = new TriangleMesh();
            int sourcePolyCount = sourceMesh.polyCount();  
        }

    } // builder
}// namespace
