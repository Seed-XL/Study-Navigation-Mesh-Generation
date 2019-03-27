using System;
using Utility.Logger; 

namespace NMGen
{

    /// <summary>
    /// 相对于封闭的实体高度域，这个是封装开放的Span
    /// </summary>
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
                Logger.LogError("[OpenHeightSpan][ctor]Floos is less than zero|{0}",floor);
                return; 
            }
            if( height < 1 ) 
            {
                Logger.LogError("[OpenHeightSpan][ctor]Height is less than one.|{0}",height);
                return; 
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

        public int distanceToRegionCore()
        {
            return mDistanceToRegionCore; 
        }

        public int floor()
        {
            return mFloor;  
        }

        public OpenHeightSpan getNeighbor( int direction )
        {
            switch (direction)
            {
                case 0: return mNeighborConnection0;
                case 1: return mNeighborConnection1;
                case 2: return mNeighborConnection2;
                case 3: return mNeighborConnection3;
                default:
                    return null; 
            }

        }

        public int regionID()
        {
            return mRegionID; 
        }

        public void getDetailedRegionMap(ref int[] outData , int insertIndex)
        {
            for(int i = 0; i < 8; ++i)
            {
                outData[insertIndex + i] = NULL_REGION; 
            }

            OpenHeightSpan nSpan = null;
            OpenHeightSpan nnSpan = null; 

            for(int dir = 0; dir < 4; dir++)
            {
                nSpan = getNeighbor(dir); 
                if( nSpan != null )
                {
                    outData[insertIndex + dir] = nSpan.regionID();
                    nnSpan = nSpan.getNeighbor((dir + 1) & 0x3); 
                    if( nnSpan != null )
                    {
                        outData[insertIndex + dir + 4] = nnSpan.regionID(); 
                    }

                    nnSpan = nSpan.getNeighbor((dir + 3) & 0x3); 
                    if( nnSpan != null )
                    {
                        outData[insertIndex+((dir+3)&0x3)+4] = nnSpan.regionID(); 
                    }
                }
            }

        }


        public int height()
        {
            return mHeight; 
        }

        public OpenHeightSpan next()
        {
            return mNext; 
        }

        public void setDistanceToBorder( int value )
        {
            mDistanceToBorder = Math.Max(value, 0); 
        }

        public void setDistanceToRegionCore(int value)
        {
            mDistanceToRegionCore = Math.Max(value, 0);  
        }

        public void setNeighbor(int direction , OpenHeightSpan neighbor)
        {
            switch(direction)
            {
                case 0: mNeighborConnection0 = neighbor; break;
                case 1: mNeighborConnection0 = neighbor; break;
                case 2: mNeighborConnection0 = neighbor; break;
                case 3: mNeighborConnection0 = neighbor; break;
            }
        }

        public void setNext(OpenHeightSpan value)
        {
            mNext = value; 
        }

        public void setRegionID(int value)
        {
            mRegionID = value; 
        }

        public string toString()
        {
            return string.Format("[OpenHeightSpan][toString]{0}|{1}|{2}",mFloor,mHeight,mRegionID); 
        }
    }
}
