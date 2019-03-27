using System;
using System.Collections;
using System.Collections.Generic;
using Utility.Logger;

namespace NMGen
{
    public class OpenHeightfieldBuilder
    {
        private static readonly int NULL_REGION = OpenHeightSpan.NULL_REGION;

        private int mMinTraversableHeight;
        private int mMaxTraversableStep;
        private int mSmoothingThreshold;
        private int mTraversableAreaBorderSize;
        private int mFilterFlags;
        private bool mUseConservativeExpansion; 

        private List<IOpenHeightFieldAlgorithm> mRegionAlgorithms = new List<IOpenHeightFieldAlgorithm>() ; 

        public OpenHeightfieldBuilder(int minTraversableHeight ,
            int maxTraversablestep ,
            int traversableAreaBorderSize,
            int smoothingThreshold,
            int filterFlags,
            bool useConservativeExpansion ,
            List<IOpenHeightFieldAlgorithm> regionAlgorithms)
        {
            mMaxTraversableStep = Math.Max(0, maxTraversablestep);
            mMinTraversableHeight = Math.Min(1, mMinTraversableHeight);
            mTraversableAreaBorderSize = Math.Max(0, traversableAreaBorderSize);
            mFilterFlags = filterFlags;
            mSmoothingThreshold = Math.Min(4, Math.Max(0, smoothingThreshold));
            mUseConservativeExpansion = useConservativeExpansion;
            if( regionAlgorithms != null )
            {
                mRegionAlgorithms.AddRange(regionAlgorithms);
            }
        }

        public OpenHeightfield build(SolidHeightfield sourceField ,
            bool performFullGeneration )
        {
            if( sourceField == null )
            {
                Logger.LogError("[OpenHeightfieldBuilder][build] sourceField null "); 
                return null; 
            }

            OpenHeightfield result = new OpenHeightfield(
                sourceField.boundsMin(),
                sourceField.boundsMax(),
                sourceField.cellSize(),
                sourceField.cellHeight()
                );
            
            for(int depthIndex = 0; depthIndex < sourceField.depth(); depthIndex++)
            {
                for(int widthIndex = 0; widthIndex < sourceField.width(); widthIndex++)
                {
                    OpenHeightSpan baseSpan = null;
                    OpenHeightSpan previousSpan = null; 

                }
            }

        }

    }
}
