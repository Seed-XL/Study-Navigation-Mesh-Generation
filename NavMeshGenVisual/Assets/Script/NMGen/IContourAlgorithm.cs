using System;
using Utility.Logger;
using System.Collections.Generic;


namespace NMGen
{
    public interface IContourAlgorithm
    {
        //(x,y,z,regionID)
        void apply(List<int> sourceVerts, List<int> resultVerts);
    }
}
