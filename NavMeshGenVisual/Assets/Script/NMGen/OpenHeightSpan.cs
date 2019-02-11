using System;

namespace NMGen
{
    public class OpenHeightSpan
    {
        public static readonly int NULL_REGION = 0;

        public int flags = 0;

        private int mRegionID = 0;
        private int mDistanceToRegionCore = 0;
        private int mDistanceToBorder = 0;

        private int mFloor;
        private int mHeight;

        private OpenHeightSpan mNext = null;
        private OpenHeightSpan mNeighborConnection0 = null;
        private OpenHeightSpan mNeighborConnection1 = null;
        private OpenHeightSpan mNeighborConnection2 = null;
        private OpenHeightSpan mNeighborConnection3 = null;


        public OpenHeightSpan(int floor,int height)
        {
            if( floor < 0 )
            {
                throw new Exception("Floos is less than zero.");
            }
            if( height < 1 ) 
            {
                throw new Exception("Height is less than one.");
            }

            mFloor = floor;
            mHeight = height; 
        }

        public int ceiling()
        {
            return mFloor + mHeight; 
        }

        public int distanceToBorder()
        {
            return mDistanceToBorder;  
        }

    }
}
