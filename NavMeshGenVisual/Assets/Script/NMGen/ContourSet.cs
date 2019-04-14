using System;
using Utility.Logger;
using System.Collections.Generic;


namespace NMGen
{
    public class ContourSet : BoundeField
    {
        private List<Contour> mContours; 

        public ContourSet(float[] gridBoundsMin,
            float[] gridBoundsMax,
            float cellSize,
            float cellHeight,
            int initialSize) : base(gridBoundsMin,gridBoundsMax,cellSize,cellHeight)
        {
            mContours = new List<Contour>(initialSize); 
        }

        public void add(Contour contour)
        {
            mContours.Add(contour); 
        }

        public Contour get(int index)
        {
            if( index < 0 || index >= mContours.Count )
            {
                return null; 
            }
            return mContours[index]; 
        }

        public int size()
        {
            return mContours.Count;  
        }

    }
}
