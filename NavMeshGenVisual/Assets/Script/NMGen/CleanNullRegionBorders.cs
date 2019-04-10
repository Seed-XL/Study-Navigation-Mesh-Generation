using System;
using System.Collections;
using System.Collections.Generic;
using Utility.Logger;

namespace NMGen
{
    public class CleanNullRegionBorders : IOpenHeightFieldAlgorithm
    {
        private static readonly int NULL_REGION = OpenHeightSpan.NULL_REGION;
        private bool mUseOnlyNullSpans;

        private Stack<OpenHeightSpan> mwOpenSpans = new Stack<OpenHeightSpan>();
        private Stack<int> mwBorderDistance = new Stack<int>();
        private int[] mwNeighborRegions = new int[8]; 


        public CleanNullRegionBorders(bool useOnlyNullRegionSpans)
        {
            mUseOnlyNullSpans = useOnlyNullRegionSpans; 
        }

        public void apply(OpenHeightfield field)
        {
            if( null == field )
            {
                Logger.LogError("[CLeanNullRegionBorders][apply]field null"); 
                return; 
            }

            int nextRegionID = field.regionCount();
            OpenHeightfield.OpenHeightFieldIterator iter = field.GetEnumerator(); 
            while( iter.MoveNext() )
            {
                OpenHeightSpan span = iter.Current; 
                if( span.flags != 0  )
                {
                    continue; 
                }

                span.flags = 1;

                OpenHeightSpan workingSpan = null;
                int edgeDirection = -1; 

                //找到第一个RegionID为空的Span
                if( NULL_REGION == span.regionID() )
                {
                    //找到第一个非空的邻居
                    edgeDirection = getNonNullBorderDrection(span); 
                    if( -1 == edgeDirection )
                    {
                        continue; 
                    }

                    workingSpan = span.getNeighbor(edgeDirection);
                    edgeDirection = (edgeDirection + 2) & 0x3;
                }
            }
            
        }      

        private static int getNonNullBorderDrection(OpenHeightSpan span)
        {
            for(int dir = 0; dir < 4; ++dir)
            {
                OpenHeightSpan nSpan = span.getNeighbor(dir); 
                if(nSpan != null 
                    && nSpan.regionID() != NULL_REGION )
                {
                    return dir; 
                }
            }
            return -1; 
        }

    }
}
