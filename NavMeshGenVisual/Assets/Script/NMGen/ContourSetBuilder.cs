using System;
using Utility.Logger;
using System.Collections.Generic;


namespace NMGen
{
    public class ContourSetBuilder
    {
        private static readonly int NULL_REGION = OpenHeightSpan.NULL_REGION;
        private List<IContourAlgorithm> mAlgorithms = new List<IContourAlgorithm>(); 

        public ContourSetBuilder(List<IContourAlgorithm> algorithms)
        {
            if( algorithms == null )
            {
                Logger.LogWarning("[ContourSetBuilder][Ctor]null"); 
                return; 
            }

            this.mAlgorithms.AddRange(algorithms); 
        }


        public ContourSet build(OpenHeightfield sourceField)
        {
            if( null == sourceField
                || 0 == sourceField.regionCount() )
            {
                Logger.LogError("[ContourSetBuilder][build]sourceField Invalid"); 
                return null; 
            }

            ContourSet result = new ContourSet(sourceField.boundsMin(),
                sourceField.boundsMax(),
                sourceField.cellSize(),
                sourceField.cellHeight(),
                sourceField.regionCount());

            int discardedContours = 0;

            //TODO
            return null; 
        }
    }
}
