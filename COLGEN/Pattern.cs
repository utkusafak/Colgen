using System.Collections.Generic;
namespace RF_HEURISTICS.COLGEN
{
    public class Pattern
    {
        public List<int> Sequence { get; set; }
        public List<int> SetupSequence { get; set; }
        public int ST { get; set; }
        public int FirstPart { get; set; }
        public int LastPart { get; set; }
        public Pattern(List<int> _sequence, ProblemData data)
        {
            FirstPart = _sequence[0];
            LastPart = _sequence[_sequence.Count - 1];
            Sequence = new List<int>(_sequence);
            SetupSequence = new List<int>();
            CalculateST(Sequence, data);
        }

        public void CalculateST(List<int> sequence, ProblemData data)
        {
            ST = 0;
            SetupSequence.Add(0);
            if (sequence.Count > 1)
            {
                for (int i = 0; i < sequence.Count - 1; i++)
                {
                    ST = ST + data.ST_ij[sequence[i], sequence[i + 1]];
                    SetupSequence.Add(data.ST_ij[sequence[i], sequence[i + 1]]);
                }
            }
        }

    }
}
