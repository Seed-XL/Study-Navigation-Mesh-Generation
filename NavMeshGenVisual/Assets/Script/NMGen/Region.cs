using System;
using Utility.Logger;
using System.Collections.Generic; 
     

namespace NMGen
{
    public class Region
    {
        public int id = 0;
        public int spanCount = 0;
        public bool remap = false;

        public List<int> connections = new List<int>();
        public List<int> overlappingRegions = new List<int>();

        public Region( int id )
        {
            this.id = id; 
        }

        public void resetWithID( int newRegionID )
        {
            id = newRegionID;
            spanCount = 0;
            connections.Clear();
            overlappingRegions.Clear(); 
        }


        public string toString()
        {
            return string.Format("[Region]{0}|{1}|{2}|{3}",id,spanCount,connections,overlappingRegions); 
        }
    }
}
