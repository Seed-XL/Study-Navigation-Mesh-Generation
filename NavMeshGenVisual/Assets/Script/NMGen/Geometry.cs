using System;
using Utility.Logger;
using System.Collections.Generic;


namespace NMGen
{
    public static class Geometry
    {
        public static float getPointSegmentDistanceSq(
            int px,int py,
            int ax,int ay,
            int bx,int by
            )
        {
            float deltaABx = bx - ax;
            float deltaABy = by - ay;
            float deltaAPx = px - ax;
            float deltaAPy = py - ay;

            float segmentABLengthSq = deltaABx * deltaABx + deltaABy * deltaABy; 
            if( 0 == segmentABLengthSq )
            {
                //不是线段，直接返回两点的距离 
                return deltaAPx * deltaAPx + deltaAPy * deltaAPy; 
            }

            float u = (deltaAPx * deltaABx + deltaAPy * deltaABy) / segmentABLengthSq ; 
            if( u < 0 )
            {
                return deltaAPx * deltaAPx + deltaAPy * deltaAPy; 
            }
            else if( u > 1 )
            {
                return (px-bx) * (px-bx) + (py-by)*(py-by); 
            }

            float deltaX = (ax + u * deltaABx) - px ;
            float deltaY = (ay + u * deltaABy) - py ;
            return deltaX * deltaX + deltaY * deltaY;  
           
        } 


    }
}
