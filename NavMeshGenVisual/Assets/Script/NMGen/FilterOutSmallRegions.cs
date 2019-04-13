using System;
using System.Collections;
using System.Collections.Generic;
using Utility.Logger;

namespace NMGen
{
    public class FilterOutSmallRegions : IOpenHeightFieldAlgorithm
    {
        private static int NULL_REGION = OpenHeightSpan.NULL_REGION;
        private int mMinUnconnectedRegionSize;
        private int mMergeRegionSize; 

        public FilterOutSmallRegions(int minUnconnectedRegionSize ,
            int mergeRegionSize)
        {
            mMinUnconnectedRegionSize = minUnconnectedRegionSize;
            mMergeRegionSize = Math.Max(0, mergeRegionSize); 
        }

        public void apply( OpenHeightfield field )
        {
            if (null == field)
            {
                Logger.LogError("[FilterOutSmallRegions][apply]field null");
                return;
            }

            if( field.regionCount() < 2)
            {
                Logger.LogError("[FilterOutSmallRegions][apply]RegionCnt|{0}",field.regionCount());
                return; 
            }

            //TODO
        }
    }
}
