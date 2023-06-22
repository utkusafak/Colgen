using System;
namespace RF_HEURISTICS.COLGEN
{
    public class Column
    {
        public Pattern pattern { get; set; }
        public int period;
        public int machine;
        public string id;
        public int start;
        public Column(Pattern _pattern, int l, int t)
        {
            pattern = _pattern;
            period = t;
            machine = l;
            id = "";
            foreach (var a in _pattern.Sequence)
            {
                id = id + a + "-";
            }
            id = id + l + "-" + t;
            start = 0;
        }
    }
}
