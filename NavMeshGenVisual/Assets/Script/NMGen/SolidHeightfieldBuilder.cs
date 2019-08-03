using System;
using System.Collections;
using System.Collections.Generic;
using Utility.Logger;

namespace NMGen
{
    //体素化Builder
    public class SolidHeightfieldBuilder
    {
        private bool mClipLedges;

        //Span之间允许的最小高度,以体素为单位 
        private int mMinTraversableHeight;

        //Span之间允许的最大落差，以体素为单位 
        private int mMaxTraversableStep;

        //二面角的余弦值 
        private float mMinNormalY;

        private float mCellSize;

        private float mCellHeight;

        public SolidHeightfieldBuilder( float cellSize,
            float cellHeight,
            int minTraversableHeight ,
            int maxTraversableStep,
            float maxTraversableSlope,
            bool clipLedges
            )
        {
            mMinTraversableHeight = Math.Max(1, minTraversableHeight);
            mMaxTraversableStep = Math.Max(0, maxTraversableStep);
            //注意，传进来的值被改变了
            maxTraversableSlope = Math.Min(85, Math.Max(0, maxTraversableSlope));

            mClipLedges = clipLedges;
            mCellSize = cellSize;
            mCellHeight = cellHeight;

            /*
             *  画个图可以试试，随着maxTraversableSlope的值增大，cos里面的角度值越大
             *  然后最后得到的mMinNormalY越小，那么我们要做的就是要比mMinNormal的值大，
             *  代表的就是坡度角比对应的角小(在0-90度的范围内)
             * 
             *   * cos theta = n1 dot n2
             * 
             *   Using:
             * 
             *       n1 = (0, 1, 0) (Represents a flat surface on the (x,z) plane.)
             *       n2 = (x, y, z) Normalized. (A surface on an arbitrary plane.)
             * 
             *   Simplify and solve for y:
             * 
             *       cos theta = 0x + 1y + 0z
             *       y = cos theta
             * 
             */
            mMinNormalY = (float)Math.Cos(Math.Abs(maxTraversableSlope) / 180 * Math.PI);
        }


        //vertices是以(x,y,z)(x,y,z)三组三组为一个顶点
        public SolidHeightfield build( float[] vertices, int[] indices )
        {
            if( vertices == null 
                || indices == null 
                || vertices.Length % 3 != 0 
                || indices.Length % 3 != 0)
            {
                Logger.LogError("[SolidHeightfieldBuilder][build]invalid"); 
                return null; 
            }

            SolidHeightfield result = new SolidHeightfield(mCellSize, mCellHeight);

            //用作分母，方便后面计算的
            float inverseCellSize = 1 / result.cellSize();
            float inverseCellHeight = 1 / result.cellHeight();


            float xmin = vertices[0];
            float ymin = vertices[1];
            float zmin = vertices[2];
            float xmax = vertices[0];
            float ymax = vertices[1];
            float zmax = vertices[2];

            //遍历所有顶点，找出最大的Bounds
            for(int i = 3; i < vertices.Length; i+= 3)
            {
                xmax = Math.Max(vertices[i],xmax);
                ymax = Math.Max(vertices[i+1],ymax);
                zmax = Math.Max(vertices[i+2], zmax);

                xmin = Math.Min(vertices[i], xmin);
                ymin = Math.Min(vertices[i + 1], ymin);
                zmin = Math.Min(vertices[i + 2], zmin);
            }

            result.setBounds(xmin, ymin, zmin, xmax, ymax, zmax);

            //判断哪些多边形的表面是可以行走的,坡度不能太大
            int[] polyFlags = markInputMeshWalkableFlags(vertices, indices);

            //开始对每个面进行体素化
            int polyCount = indices.Length / 3;  
            for(int iPoly = 0; iPoly < polyCount; iPoly++)
            {
                voxelizeTriangle(iPoly,
                    vertices,
                    indices,
                    polyFlags[iPoly],
                    inverseCellSize,
                    inverseCellHeight,
                    result); 
            }

            markLowHeightSpans(result); 

            if( mClipLedges )
            {
                markLedgeSpans(result); 
            }

            return result; 
        }


        /// <summary>
        /// 边缘裁剪，那种像断崖式的Span也不可走的
        /// </summary>
        /// <param name="field"></param>
        private void markLedgeSpans(SolidHeightfield field)
        {
            SolidHeightfield.SolidHeightfieldIterator iter = field.GetEnumerator(); 
            while( iter.MoveNext() )
            {
                HeightSpan span = iter.Current; 

                if( (span.flags() & SpanFlags.WALKABLE) == 0 )
                {
                    continue; 
                }

                int widthIndex = iter.widthIndex();
                int depthIndex = iter.depthIndex();

                int currFloor = span.max();
                int currCeiling = (span.next() != null)
                    ? span.next().min() : int.MaxValue;


                int minDistanceToNeighbor = int.MaxValue; 

                for(int dir = 0; dir < 4; dir++)
                {
                    int nWidthIndex = widthIndex
                        + BoundeField.getDirOffsetWidth(dir);
                    int nDepthIndex = depthIndex
                        + BoundeField.getDirOffsetDepth(dir);


                    HeightSpan nSpan = field.getData(nWidthIndex, nDepthIndex); 
                    if( null == nSpan )
                    {
                        //TODO 这里没有搞懂为啥是 -mMaxTraversableStep - currFloor,是为了比-mMaxTraversableStep更小，以便直接判断不能行走吗？
                        // 用大可行的距离，再减去currFloor，得到的肯定是一个更小的值。
                        // currFloor - mMaxTraversableStep 是一个最大允许的落差地板距离。注意这里只考虑下落的情况
                        minDistanceToNeighbor = Math.Min(minDistanceToNeighbor, -mMaxTraversableStep - currFloor);
                        continue; 
                    }


                    /* 
                    *  先考虑一种特殊情况 ，那就是
                    *  那就是nSpan.min也比currFloor要高，那么对应的
                    *  的邻居相当于也是没有Floor的，所以默认取-mMaxTraversableStep吧。
                    */

                    int nFloor = -mMaxTraversableStep;
                    int nCeiling = nSpan.min(); 

                    //当前Span所处列的currCeiling和邻居的nCeiling相比，取最低的
                    if( Math.Min(currCeiling,nCeiling) - currFloor > mMinTraversableHeight)
                    {
                        minDistanceToNeighbor = Math.Min(minDistanceToNeighbor, (nFloor - currFloor));
                    }

                    for(nSpan = field.getData(nWidthIndex,nDepthIndex); nSpan != null; nSpan = nSpan.next())
                    {
                        nFloor = nSpan.max();  //现在才开始用max考虑真正存在的Floor
                        nCeiling = (nSpan.next() != null)
                            ? nSpan.next().min() : int.MaxValue; 

                        if( Math.Min(currCeiling,nCeiling) - Math.Max(currFloor,nFloor) > mMinTraversableHeight )
                        {
                            minDistanceToNeighbor = Math.Min(minDistanceToNeighbor, (nFloor - currFloor)); 
                        }
                    }

                }

                //如果最近的距离比较最大掉落还小的放在，那么就是不可行走的
                if(minDistanceToNeighbor < -mMaxTraversableStep)
                {
                    span.setFlags(span.flags() & ~SpanFlags.WALKABLE); 
                }
            }
        }

        /// <summary>
        /// 确保垂直方向上两个Span之间的区域不会卡头
        /// </summary>
        /// <param name="field"></param>
        private void markLowHeightSpans(SolidHeightfield field)
        {
            SolidHeightfield.SolidHeightfieldIterator iter = field.GetEnumerator(); 
            while( iter.MoveNext() )
            {
                HeightSpan span = iter.Current; 

                if( (span.flags() & SpanFlags.WALKABLE) == 0 )
                {
                    continue; 
                }

                int spanFloor = span.max(); //SolidSpan的max，其实就是OpenSpan的底部
                int spanCeiling = (span.next() != null)
                    ? span.next().min() : int.MaxValue; 

                if( spanCeiling - spanFloor <= mMinTraversableHeight  )
                {
                    span.setFlags(span.flags() & ~SpanFlags.WALKABLE);
                }
            }
        }
        
        //按面来体素化？
        private static void voxelizeTriangle( int polyIndex ,
            float[] vertices,
            int[] indices,
            int polyFlags,
            float inverseCellSize ,
            float inverseCellHeight,
            SolidHeightfield inoutField)   //本来就是引用的话，传进来就会被修改
        {
            int pPoly = polyIndex * 3;

            //一个面的三个点
            float[] triVerts = new float[]
            {
                vertices[indices[pPoly]*3],     //VertA x      
                vertices[indices[pPoly]*3+1],   //VertA y  
                vertices[indices[pPoly]*3+2],   //VertA z  
                vertices[indices[pPoly+1]*3],   //VertB x
                vertices[indices[pPoly+1]*3+1], //VertB y
                vertices[indices[pPoly+1]*3+2], //VertB z
                vertices[indices[pPoly+2]*3],   //VertC x
                vertices[indices[pPoly+2]*3+1], //VertC y    
                vertices[indices[pPoly+2]*3+2], //VertC z
            };

            //三角面的xz投影包围盒
            float[] triBoundsMin = new float[] {
                triVerts[0],triVerts[1],triVerts[2]
            };
            float[] triBoundsMax = new float[] {
                triVerts[0],triVerts[1],triVerts[2]
            };

            // int vertPointer = 3  相当于  int i = 1 ，因为是从第二个顶点开始算起，数组长度总共为9
            // Loop through all vertices to determine the actual bounding box.
            for (int vertPointer = 3; vertPointer < 9; vertPointer += 3)
            {
                triBoundsMin[0] = Math.Min(triBoundsMin[0], triVerts[vertPointer]);
                triBoundsMin[1] = Math.Min(triBoundsMin[1], triVerts[vertPointer+1]);
                triBoundsMin[2] = Math.Min(triBoundsMin[2], triVerts[vertPointer+2]);
                triBoundsMax[0] = Math.Min(triBoundsMax[1], triVerts[vertPointer]);
                triBoundsMax[1] = Math.Min(triBoundsMax[2], triVerts[vertPointer+1]);
                triBoundsMax[2] = Math.Min(triBoundsMax[3], triVerts[vertPointer+2]);
            }

            if( !inoutField.overlaps(triBoundsMin,triBoundsMax) )
            {
                return; 
            }

            //将三角形的坐标转换成对应的cell坐标系,就是具体对应到哪个width和depth的Column  
            //两个坐标相差得到距离，再除以cell的大小，得到每个坐标对应落在哪个Cell
            int triWidthMin = (int)((triBoundsMin[0] - inoutField.boundsMin()[0]) * inverseCellSize);
            int triDepthMin = (int)((triBoundsMin[2] - inoutField.boundsMin()[2]) * inverseCellSize);
            int triWidthMax = (int)((triBoundsMax[0] - inoutField.boundsMin()[0]) * inverseCellSize);
            int triDepthMax = (int)((triBoundsMax[2] - inoutField.boundsMin()[2]) * inverseCellSize);

            triWidthMin = clamp(triWidthMin, 0, inoutField.width() - 1);
            triDepthMin = clamp(triDepthMin, 0, inoutField.depth() - 1);
            triWidthMax = clamp(triWidthMax, 0, inoutField.width() - 1);
            triDepthMax = clamp(triWidthMax, 0, inoutField.depth() - 1);
            


            //从论文的图示来看，三角形与矩形的交点组成的凸包最多有7个点。
            float[] inVerts = new float[21];
            float[] outVerts = new float[21];
            float[] inrowVerts = new float[21];


            float fieldHeight = inoutField.boundsMax()[1] - inoutField.boundsMin()[1];

            //http://www.sunshine2k.de/coding/java/SutherlandHodgman/SutherlandHodgman.html
            for ( int depthIndex = triDepthMin; depthIndex <= triDepthMax; ++depthIndex )
            {
                Array.Copy(triVerts, 0, inVerts, 0, triVerts.Length);

                int intermediateVertCount = 3;
                //将体素depth坐标为 depthIndex 对应的 Edge 转变为 笛卡尔坐标的 z.
                //其实就是从体素坐标系转变为 笛卡尔坐标系
                float rowWorldZ = inoutField.boundsMin()[2]
                    + (depthIndex * inoutField.cellSize());

                //用 z = rowWorldZ的直线裁剪三角形
                intermediateVertCount = clipPoly(inVerts,
                    intermediateVertCount,
                    ref outVerts,
                    0,
                    1,
                    -rowWorldZ); 

                if( intermediateVertCount < 3)
                {
                    continue; 
                }

                //用 z = -(rowWorldZ + inoutField.cellSize) 的直线裁剪三角形
                intermediateVertCount = clipPoly(outVerts,
                    intermediateVertCount,
                    ref inrowVerts,
                    0,
                    -1,
                    rowWorldZ + inoutField.cellSize());

                if (intermediateVertCount < 3)
                {
                    continue;
                }

                for (int widthIndex = triWidthMin; widthIndex <= triWidthMax; ++widthIndex)
                {
                    int vertCount = intermediateVertCount;
                    //将体素width坐标为 widthIndex 转变为 笛卡尔坐标的 x
                    float colWorldX = inoutField.boundsMin()[0]
                        + (widthIndex * inoutField.cellSize());

                    //用直线 x = colWorldX的直线进行裁剪 
                    vertCount = clipPoly(inrowVerts, vertCount, ref outVerts, 1, 0, -colWorldX);
                    if( vertCount < 3)
                    {
                        continue;
                    }

                    //用直线 x = -(colWorldX + CellSize) 进行裁剪
                    vertCount = clipPoly(outVerts,
                        vertCount,
                        ref inrowVerts,
                        -1,
                        0,
                        colWorldX + inoutField.cellSize()); 

                    if( vertCount < 3 )
                    {
                        continue; 
                    }

                    float heightMin = inVerts[1];
                    float heightMax = inVerts[1]; 

                    for(int i = 1; i < vertCount; ++i)
                    {
                        heightMin = Math.Min(heightMin, inVerts[i * 3 + 1]);
                        heightMax = Math.Min(heightMax, inVerts[i * 3 + 1]);
                    }

                    
                    //heigtMin和heighMax都是坐标，所以需要将其转换成相对于 inoutField 地板的高度,也就是一个标量
                    heightMin -= inoutField.boundsMin()[1];
                    heightMax -= inoutField.boundsMin()[1]; 

                    if(heightMax < 0.0f  || heightMin > fieldHeight)
                    {
                        Logger.LogWarning("[SolidHeightfieldBuilder][voxelizeTriangle]Invalid|{0}|{1}|{2}|{3}|{4}",widthIndex,depthIndex,heightMin,heightMax,fieldHeight); 
                        continue;         
                    }

                    if( heightMin < 0.0f )
                    {
                        Logger.LogWarning("[SolidHeightfieldBuilder][voxelizeTriangle]heigtMin Change|{0}|{1}|{2}|{3}|", widthIndex, depthIndex, heightMin, inoutField.boundsMin()[1]);
                        heightMin = inoutField.boundsMin()[1]; 
                    }
                    if( heightMax > fieldHeight )
                    {
                        Logger.LogWarning("[SolidHeightfieldBuilder][voxelizeTriangle]heightMax Change|{0}|{1}|{2}|{3}|", widthIndex, depthIndex, heightMax, fieldHeight,inoutField.boundsMin()[1]);
                        heightMax = inoutField.boundsMax()[1]; 
                    }

                    //将坐标重新转换为Grid的坐标，其实也是表示这三角面最低点和最高点，分别属于哪两个体素
                    int heightIndexMin = clamp(
                        (int)Math.Floor(heightMin * inverseCellHeight),
                        0,
                        short.MaxValue
                        );
                    int heightIndexMax = clamp(
                        (int)Math.Ceiling(heightMax * inverseCellHeight),
                        0,
                        short.MaxValue 
                        );

                    inoutField.addData(widthIndex,
                        depthIndex,
                        heightIndexMin,
                        heightIndexMax,
                        polyFlags
                        ); 
                }

            }

        }

        private static int clamp(int value ,int minimum,int maximum)
        {
            return value < minimum 
                ? minimum : ( value > maximum ? maximum :value )  ; 
        }


        /* 
         * 窗口裁剪线段算法
         * Sutherland-Hodgman，一次用窗口的一条边裁剪多边形 ：https://en.wikipedia.org/wiki/Sutherland%E2%80%93Hodgman_algorithm
         * 正确的算法：
         * 遍历修剪区域的所有边，判断需要裁剪多边形的顶点，是否在对应边的内侧( d > 0 )
         *  https://www.youtube.com/watch?v=Euuw72Ymu0M
         *  https://www.geeksforgeeks.org/polygon-clipping-sutherland-hodgman-algorithm-please-change-bmp-images-jpeg-png/
         *  http://www.sunshine2k.de/coding/java/SutherlandHodgman/SutherlandHodgman.html
         *  http://www.cs.mun.ca/av/old/teaching/cg/notes/raster_clip2_inclass.pdf
         *  
         *  pnx ，pnz ,pd  分别对应一般式直线中的a,b,c
         *  所以裁剪边的表达式： pnx * x + pnz * z + pd = 0 
         *  这里是直接忽略了y坐标的，所以可以看作是在xz面上投影的裁剪
         */
        private static int clipPoly(float[] inVerts,
            int inputVertCout ,
            ref float[] outVerts,
            float pnx,
            float pnz,
            float pd)
        {
            /*
            * 这是直线的一般方程式
            * d = ax + bz + c  ====> d = pnx * vx + pnz * vz + pd
            * 用pnx和pnz，以及pd来表示在xz平面的裁剪直线
            * d的值，等于0，表示顶点在直线上，也就是说三角形的顶点在矩形的边上
            * d的值，大于0，表示顶点在直线的内侧。
            * d的值，小于0，表示顶点在直线的外侧。
            */

            float[] valuePerVert = new float[inputVertCout]; 
            for(int vertIndex = 0; vertIndex < inputVertCout; ++vertIndex)
            {
                valuePerVert[vertIndex] = (pnx * inVerts[vertIndex * 3]) 
                    + (pnz * inVerts[vertIndex * 3 + 2]) + pd; 
            }

            int m = 0; 
            for( int current = 0 , previous = valuePerVert.Length - 1;
                current < valuePerVert.Length;
                 previous = current , ++current )
            {
                bool prevVertInside = valuePerVert[previous] >= 0;
                bool currVertInside = valuePerVert[current] >= 0;

                /* 
                 * 两点的连线与矩形直线有交点，一个在inside，一个在outside
                 *  
                 */
                if ( prevVertInside != currVertInside )
                {
                    //插值求交点
                    float s = valuePerVert[previous] / (valuePerVert[previous] - valuePerVert[current]);
                    outVerts[m * 3 + 0] = inVerts[previous * 3 + 0] + (inVerts[current * 3 + 0] - inVerts[previous * 3 + 0]) * s;
                    outVerts[m * 3 + 1] = inVerts[previous * 3 + 1] + (inVerts[current * 3 + 1] - inVerts[previous * 3 + 1]) * s;  //y轴有必要裁剪吗？？？这是二维的吧
                    outVerts[m * 3 + 2] = inVerts[previous * 3 + 2] + (inVerts[current * 3 + 2] - inVerts[previous * 3 + 2]) * s;
                    m++; 
                }
                //如果前面是prev是inside的话，那么currVertInside必然是outside
                if( currVertInside )
                {
                    outVerts[m * 3 + 0] = inVerts[current * 3 + 0];
                    outVerts[m * 3 + 1] = inVerts[current * 3 + 0];
                    outVerts[m * 3 + 2] = inVerts[current * 3 + 0];
                    m++;
                }
            }

            return m;
        }

        //vertices是以3为一组，因为是按 x,y,z,x1,y1,z1,x2,y2,z2这样储存的
        private int[] markInputMeshWalkableFlags(float[] vertices,int[] indices)
        {
            //总有多少个三角形
            int polyCount = indices.Length / 3;

            //每个面对应一个flag
            int[] flags = new int[polyCount];

            float[] diffAB = new float[3];
            float[] diffAC = new float[3];
            float[] crossDiff = new float[3];

           
            for(int iPoly = 0 ; iPoly < polyCount; iPoly++ )
            {
                //indices储存的是顶点的索引,三个一组，组成一个面。这就是iPoly * 3的原因，步进为3。
                //一个点，占三个索引，分别对应xyz，三个点要占9个索引。分别是x1y1z1,x2y2z2,x3y3z3。
                //每个顶点占三个，一个面占9个,vertices里面就是这样的储存方式的。indices数组储存的也是
                //顶点数组的下标值 ，因此按步进3来算，取一下个面的点，也需要 * 3 ，这就是外层 * 3 的原因。
                int pVertA = indices[iPoly * 3] * 3;
                int pVertB = indices[iPoly * 3 + 1] * 3;
                int pVertC = indices[iPoly * 3 + 2] * 3;

                //叉乘获得面的法线
                float[] polyNormal = cross( subtract(pVertB,pVertA,vertices,ref diffAB),
                    subtract(pVertC, pVertA, vertices, ref diffAC),ref crossDiff
                    );
                float normalY = getNormalY(polyNormal);

                //normalY比mMinNormalY要大，那么意味着比对应的角度小
                if ( normalY > mMinNormalY )
                {
                    flags[iPoly] = SpanFlags.WALKABLE; 
                }

            }

            return flags; 
        }

        private static float[] subtract(int pVertA,
            int pVertB,
            float[] vertices,
            ref float[] result)
        {
            result[0] = vertices[pVertA] - vertices[pVertB];
            result[1] = vertices[pVertA + 1] - vertices[pVertB + 1];
            result[2] = vertices[pVertA + 2] - vertices[pVertB + 2];
            return result;  
        }

        /// <summary>
        /// 叉乘：
        /// Reference: http://mathworld.wolfram.com/CrossProduct.html
        /// Reference: http://en.wikipedia.org/wiki/Cross_product
        /// </summary>
        /// <param name="u"></param>
        /// <param name="v"></param>
        /// <param name="outValue"></param>
        /// <returns></returns>
        private static float[] cross( float[] u , float[] v , ref float[] outValue)
        {
            outValue[0] = u[1] * v[2] - u[2] * v[1];
            outValue[1] = -u[0] * v[2] + u[2] * v[0];
            outValue[2] = u[0] * v[1] - u[1] * v[0];

            return outValue; 
        }


        /// <summary>
        /// 获得一个向量中y分量的标准化之后的值
        /// </summary>
        /// <param name="v"></param>
        /// <returns></returns>
        private static float getNormalY(float[] v)
        {
            float epsilon = 0.0001f;

            float length =
                (float)Math.Sqrt((v[0] * v[0]) + (v[1]*v[1]) + (v[2]*v[2])); 
            if( length <= epsilon )
            {
                length = 1; 
            }

            float y = v[1] / length; 
            if( Math.Abs(y) < epsilon )
            {
                y = 0; 
            }

            return y; 
        }

    }
}
