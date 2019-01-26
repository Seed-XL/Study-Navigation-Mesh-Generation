using System;
using System.Collections;
using System.Collections.Generic;

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
            int maxTraversableSlope,
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
             *  代表的就是坡度角比对应的角小
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
                return null; 
            }

            SolidHeightfield result = new SolidHeightfield(mCellSize, mCellHeight);

            float inverseCellSize = 1 / result.cellSize();
            float inverseCellHeight = 1 / result.cellHeight();


            float xmin = vertices[0];
            float ymin = vertices[1];
            float zmin = vertices[2];
            float xmax = vertices[0];
            float ymax = vertices[1];
            float zmax = vertices[2];

            //
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

            int[] polyFlags = markInputMeshWalkableFlags(vertices, indices);


            return result; 
        }

        //vertices是以3为一组，因为是按 x,y,z,x1,y1,z1,x2,y2,z2这样储存的
        private int[] markInputMeshWalkableFlags(float[] vertices,int[] indices)
        {
            //总有多少个三角形
            int polyCount = indices.Length / 3;

            //每三个值为一个点
            int[] flags = new int[polyCount];

            float[] diffAB = new float[3];
            float[] diffAC = new float[3];
            float[] crossDiff = new float[3];

           
            for(int iPoly = 0;iPoly < polyCount; iPoly++ )
            {
                //分别求出一个三角形的三个点,后面乘以3，是因为vertices的是以xyz x1y1z1这样储存的
                int pVertA = indices[iPoly * 3] * 3;
                int pVertB = indices[iPoly * 3 + 1] * 3;
                int pVertC = indices[iPoly * 3 + 2] * 3;

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
