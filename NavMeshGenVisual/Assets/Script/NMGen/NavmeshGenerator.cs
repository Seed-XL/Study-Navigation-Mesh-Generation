using System;
using Utility.Logger;
using System.Collections.Generic;


namespace NMGen
{
    public class NavmeshGenerator
    {
        private SolidHeightfieldBuilder mSolidHeightFieldBuilder;
        private OpenHeightfieldBuilder mOpenHeightFieldBuilder;
        private ContourSetBuilder mContourSetBuilder;
        private PolyMeshFieldBuilder mPolyMeshBuilder;
        private DetailMeshBuilder mTriangleMeshBuilder; 

        public NavmeshGenerator(float cellSize, float cellHeight,
            float minTraversableHeight,float maxTraversableStep , float maxTraversableSlope,
            bool clipLedges,float traversableAreaBorderSize ,int smoothingThreshold,
            bool useConservativeExpansion ,int minUnconnectedRegionSize , int mergeRegionSize,
            float maxEdgeLength,float edgeMaxDeviation , int maxVertsPerPoly , 
            float contourSampleDistance ,float contourMaxDeviation )
        {
            //world units to voxel units
            int vxMinTraversableHeight = 1; 
            if( minTraversableHeight != 0 )
            {
                vxMinTraversableHeight = (int)Math.Ceiling(
                    Math.Max(float.MinValue, minTraversableHeight) /
                    Math.Max(float.MinValue, cellHeight)); 
            }

            int vxMaxTraversableStep = 0; 
            if( maxTraversableStep != 0 )
            {
                vxMaxTraversableStep = (int)Math.Ceiling( 
                    Math.Max(float.MinValue,maxTraversableStep) / 
                    Math.Max(float.MinValue,cellHeight))  ; 
            }

            int vxTraversableAreaBorderSize = 0; 
            if( traversableAreaBorderSize != 0 )
            {
                vxTraversableAreaBorderSize = (int)Math.Ceiling(
                    Math.Max(float.MinValue,traversableAreaBorderSize) /
                    Math.Max(float.MinValue,cellSize)
                    ); 
            }

            int vxMaxEdgeLength = 0; 
            if( maxEdgeLength != 0 )
            {
                vxMaxEdgeLength = (int)Math.Ceiling(
                    Math.Max(float.MinValue, maxEdgeLength) /
                    Math.Max(float.MinValue, cellSize) 
                    ); 
            }

            //构造实心域
            mSolidHeightFieldBuilder = new SolidHeightfieldBuilder(cellSize,cellHeight,
                vxMinTraversableHeight,vxMaxTraversableStep,
                maxTraversableSlope,clipLedges);

            //一些对Open域处理的算法
            List<IOpenHeightFieldAlgorithm> regionAlgorithms = new List<IOpenHeightFieldAlgorithm>();
            if( vxTraversableAreaBorderSize > 0 )
            {
                regionAlgorithms.Add(new CleanNullRegionBorders(true)); 
            }
            else
            {
                regionAlgorithms.Add(new CleanNullRegionBorders(false)); 
            }
            regionAlgorithms.Add(new FilterOutSmallRegions(minUnconnectedRegionSize, mergeRegionSize));

            mOpenHeightFieldBuilder = new OpenHeightfieldBuilder(vxMinTraversableHeight, vxMaxTraversableStep, vxTraversableAreaBorderSize,
                smoothingThreshold, SpanFlags.WALKABLE, useConservativeExpansion, regionAlgorithms);

            //构建轮廓
            List<IContourAlgorithm> contourAlgorithms = new List<IContourAlgorithm>();
            contourAlgorithms.Add(new MatchNullRegionEdges(edgeMaxDeviation / cellSize));
            contourAlgorithms.Add(new NullRegionMaxEdge(vxMaxEdgeLength));
            mContourSetBuilder = new ContourSetBuilder(contourAlgorithms);

            mPolyMeshBuilder = new PolyMeshFieldBuilder(maxVertsPerPoly);
            mTriangleMeshBuilder = new DetailMeshBuilder(contourSampleDistance, contourMaxDeviation); 
        }


        public TriangleMesh build(float[] vertices,int[] indices,IntermediateData outIntermediateData)
        {
            if( outIntermediateData != null )
            {
                outIntermediateData.reset(); 
            }

            long timerStart = 0;  

            if(outIntermediateData != null )
            {
                timerStart = System.DateTime.Now.Ticks;  
            }

            SolidHeightfield solidField = mSolidHeightFieldBuilder.build(vertices, indices); 
            if( solidField == null 
                || !solidField.hasSpans() )
            {
                return null; 
            }

            if( outIntermediateData != null )
            {
                outIntermediateData.voxelizationTime = System.DateTime.Now.Ticks - timerStart; 
            }

            if( outIntermediateData != null )
            {
                outIntermediateData.setSolidHeightfield(solidField); 
            }

            if( outIntermediateData != null )
            {
                timerStart = System.DateTime.Now.Ticks; 
            }

            OpenHeightfield openField = mOpenHeightFieldBuilder.build(solidField, false);  
            if( null == openField )
            {
                return null; 
            }

            if( outIntermediateData != null )
            {
                outIntermediateData.setOpenHeightfield(openField); 
            }

            mOpenHeightFieldBuilder.generateNeighborLinks(openField);
            mOpenHeightFieldBuilder.generateDistanceField(openField);
            mOpenHeightFieldBuilder.blurDistanceField(openField);
            mOpenHeightFieldBuilder.generateRegions(openField); 

            if( outIntermediateData != null )
            {
                outIntermediateData.regionGenTime = System.DateTime.Now.Ticks - timerStart; 
            }

            if( outIntermediateData != null )
            {
                timerStart = System.DateTime.Now.Ticks; 
            }

            ContourSet contours = mContourSetBuilder.build(openField);  
            if( null == contours )
            {
                return null; 
            }

            if( outIntermediateData != null )
            {
                outIntermediateData.contourGenTime = System.DateTime.Now.Ticks - timerStart;  
            }

            if( outIntermediateData != null )
            {
                outIntermediateData.setContours(contours); 
            }

            if( outIntermediateData != null  )
            {
                timerStart = System.DateTime.Now.Ticks; 
            }

            PolyMeshField polMesh = mPolyMeshBuilder.build(contours); 
            if( null == polMesh )
            {
                return null; 
            }

            if( null != outIntermediateData )
            {
                outIntermediateData.polyGenTime = System.DateTime.Now.Ticks - timerStart;  
            }

            if(outIntermediateData != null )
            {
                outIntermediateData.setPolyMesh(polMesh); 
            }

            if( outIntermediateData != null )
            {
                timerStart = System.DateTime.Now.Ticks;  
            }

            TriangleMesh mesh = mTriangleMeshBuilder.build(polMesh, openField); 

            if( outIntermediateData != null 
                && mesh != null )
            {
                outIntermediateData.finalMeshGenTime = System.DateTime.Now.Ticks - timerStart;  
            }

            return mesh;  
        }

    }
}
