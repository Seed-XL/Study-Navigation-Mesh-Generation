using System;
using Utility.Logger;
using System.Collections.Generic;


namespace NMGen
{
    public class TriangleMesh
    {
        //xyz
        public float[] vertices = null;
        //aindex, bindex,cindex
        public int[] indices = null;

        public int[] triangleRegions = null; 

        public int getTriangleRegion(int index)
        {
            if( index < 0 || index >= triangleRegions.Length )
            {
                return -1 ;   
            }

            return triangleRegions[index];  
        }


        public float[] getTriangleVerts(int index)
        {
            int pTriangle = index * 3; 
            if( index < 0 || pTriangle >= indices.Length )
            {
                return null; 
            }

            float[] result = new float[9]; 
            for(int i = 0; i < 3; ++i)
            {
                int pVert = indices[pTriangle + i] * 3;
                result[i * 3] = vertices[pVert];
                result[i * 3 + 1] = vertices[pVert + 1];
                result[i * 3 + 2] = vertices[pVert + 2];
            }

            return result; 
        }

        public int triangleCount()
        {
            return (triangleRegions == null ? 0 : triangleRegions.Length); 
        }

        public int vertCount()
        {
            return (vertices == null ? 0 : vertices.Length / 3);   
        }
    }
}
