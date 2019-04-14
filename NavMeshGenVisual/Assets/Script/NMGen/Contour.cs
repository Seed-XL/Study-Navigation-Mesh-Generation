using System;
using Utility.Logger;
using System.Collections.Generic;


namespace NMGen
{
    public class Contour
    {
        public int regionID;
        //(x,y,z,regionID),代表的是Detail Contour
        public int[] rawVerts;

        public int rawVertCount;

        //代表的是Simplified contour
        public int[] verts;
        public int vertCount; 

        public Contour(int regionID,
            List<int> rawList,
            List<int> vertList
            )
        {
            if( null == rawList 
                || null == vertList )
            {
                Logger.LogError("[Contour][Ctor]null");
                return; 
            }

            this.regionID = regionID;
            rawVerts = new int[rawList.Count]; 
            for(int i = 0; i < rawVerts.Length; ++i)
            {
                rawVerts[i] = rawList[i]; 
            }
            rawVertCount = rawVerts.Length / 4;

            verts = new int[vertList.Count]; 
            for(int i = 0; i < verts.Length; ++i )
            {
                verts[i] = vertList[i]; 
            }
            vertCount = verts.Length / 4;  
        }

    }
}
